#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DriverControl Python 实现
使用 Jetson GPIO 库控制步进电机

依赖库：
    - Jetson.GPIO: Jetson 平台的 GPIO 控制库
    - smbus: I2C 通信库（用于 PCA9685）
    - yaml: YAML 配置文件解析
    - threading: 多线程支持
    - rclpy: ROS2 Python 客户端库
    - flex_msgs: 自定义 ROS2 消息包

服务接口：
    - 服务名称: motor_control_service
    - 服务类型: flex_msgs/srv/MotorControl
    - 请求参数:
        * motor_frequency: uint16 - 电机频率
        * motor_direction: bool[4] - 4个电机的方向
        * motor_distance: float32[4] - 4个电机的运动距离
        * lock_block: bool - 锁定块标志
        * reset_mode: uint16 - 复位模式（2=运动模式，其他=复位模式）
    - 响应参数:
        * motor_position: float32[4] - 4个电机的当前位置
"""

import Jetson.GPIO as GPIO
import smbus
import yaml
import time
import threading
import os
import numpy as np
from typing import List, Optional
from collections import namedtuple
import traceback

# ROS2 相关导入
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from flex_msgs.srv import MotorControl

# ==================== 常量定义 ====================
# PCA9685 寄存器地址
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

# PCA9685 配置
PCA9685_I2C_ADDR = 0x40
PCA9685_I2C_BUS = 7

# 中断引脚将在初始化时从参数文件中读取
# 注意：实际的中断引脚由参数文件中的 StepMotor_interrupt 定义

# 电机控制常量
NUM_MOTORS = 4
RESET_MODE_MOTION = 2  # 运动模式
RESET_MODE_UNLOCK = 1  # 解锁模式
RESET_MODE_LOCK = 3    # 锁定模式

# 时间计算常量
TIME_CALCULATION_FACTOR = 200000.0  # 时间计算公式中的常数
DISTANCE_THRESHOLD = 0.01  # 最小运动距离阈值

# PWM 控制常量
PWM_ON_VALUE = 2048  # PWM 开启值
PWM_OFF_VALUE = 0     # PWM 关闭值

# 复位相关常量
RESET_DTIME = 20      # 每次运行时间（毫秒）
RESET_DURING = 2000   # 初始运行持续时间（毫秒）
RESET_DURING_INC = 2000  # 持续时间增量
RESET_MAX_TIME = 20000  # 最大复位时间（毫秒）

# PCA9685 频率限制
FREQ_MIN = 24
FREQ_MAX = 1526
FREQ_CORRECTION = 0.9615  # 频率修正系数
OSC_FREQ = 25000000.0     # 25MHz 振荡器频率
PWM_RESOLUTION = 4096.0   # 12-bit 分辨率

# GPIO 配置
GPIO_BOUNCE_TIME = 10  # 中断防抖时间（毫秒）

# 参数文件路径模板
# 支持两种路径：相对路径（开发时）和绝对路径（安装后）
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 判断脚本是在开发目录还是安装目录
if 'install' in _SCRIPT_DIR:
    # 从安装目录运行：install/flex_core/lib/flex_core/ -> install/flex_core/share/flex_core/params/
    # 需要上两级目录：lib/flex_core/ -> lib/ -> install/flex_core/ -> share/flex_core/params/
    _INSTALL_PARAM_PATH = os.path.join(_SCRIPT_DIR, '../../share/flex_core/params/StepMotor_{}_param.yaml')
    # 开发路径（可能不存在）
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../../..')
    PARAM_FILE_TEMPLATE = os.path.join(_WORKSPACE_ROOT, "src/flex_core/params/StepMotor_{}_param.yaml")
else:
    # 从开发目录运行：src/flex_core/scripts/ -> src/flex_core/params/
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../..')
    PARAM_FILE_TEMPLATE = os.path.join(_WORKSPACE_ROOT, "params/StepMotor_{}_param.yaml")
    # 安装路径（可能不存在）
    _INSTALL_PARAM_PATH = os.path.join(_WORKSPACE_ROOT, "../../install/flex_core/share/flex_core/params/StepMotor_{}_param.yaml")

# ==================== 全局变量 ====================
# 全局中断标志（对应 C++ 中的 volatile 变量）
motor_reset_flags = [True] * NUM_MOTORS


# ==================== 辅助数据结构 ====================
MotorGroup = namedtuple('MotorGroup', ['motor1', 'motor2', 'group_id', 'interrupt_idx1', 'interrupt_idx2'])


class MotorParam:
    """电机参数类，对应 C++ 中的 MotorParam 结构体"""
    def __init__(self, motor_params_file: str):
        # 从 YAML 文件加载参数
        # 过滤掉 OpenCV 格式的指令行（%YAML:1.0），因为 PyYAML 不支持
        with open(motor_params_file, 'r') as f:
            lines = f.readlines()
            # 过滤掉以 %YAML 开头的指令行
            filtered_lines = [line for line in lines if not line.strip().startswith('%YAML')]
            yaml_content = ''.join(filtered_lines)
            config = yaml.safe_load(yaml_content)
        
        self.StepMotor_dir = config['StepMotor_dir']
        self.StepMotor_interrupt = config['StepMotor_interrupt']
        self.StepMotor_Reset_Frequency = config['StepMotor_Reset_Frequency']
        self.Slide_block_compensation = config['Slide_block_compensation']
        self.StepMotor_Position = 0.0
        self.motor_direction_flag = config.get('motor_direction_flag', 1)


class PCA9685:
    """PCA9685 PWM 驱动芯片控制类"""
    def __init__(self, i2c_bus: int = PCA9685_I2C_BUS, address: int = PCA9685_I2C_ADDR):
        # 初始化 I2C 总线
        self.bus = smbus.SMBus(i2c_bus)
        self.i2c_addr = address
        
        # 添加线程锁保护 I2C 操作（smbus 不是线程安全的）
        self._i2c_lock = threading.Lock()
        
        # 重置 PCA9685
        self.reset()
    
    def reset(self):
        """完整重置 PCA9685 芯片"""
        with self._i2c_lock:
            try:
                # 软件复位：写入 MODE1 寄存器，设置 RESTART 位为 0，SLEEP 位为 1
                # 0x10 = SLEEP 模式，0x80 = RESTART 位清零
                self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, 0x10)
                time.sleep(0.01)  # 等待复位完成
                
                # 退出睡眠模式，启用自动增量
                # 0x80 = RESTART 位，0x20 = AI (自动增量)
                self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, 0xA0)
                time.sleep(0.01)  # 等待稳定
            except Exception as e:
                print(f"Failed to reset PCA9685: {e}")
    
    def stopAllPWM(self):
        """停止所有 PWM 通道输出（16个通道）"""
        with self._i2c_lock:
            try:
                # 停止所有通道：设置所有通道的 ON 和 OFF 寄存器
                # 对于每个通道，设置 ON=0, OFF=0 表示始终关闭
                for channel in range(16):
                    base_reg = LED0_ON_L + 4 * channel
                    self.bus.write_byte_data(self.i2c_addr, base_reg, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 1, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 2, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 3, 0x00)
            except Exception as e:
                print(f"Failed to stop all PWM channels: {e}")
    
    def write8(self, reg: int, value: int):
        """写入 8 位寄存器（线程安全）"""
        with self._i2c_lock:
            try:
                self.bus.write_byte_data(self.i2c_addr, reg, value)
            except Exception as e:
                print(f"Failed to write to I2C device: {e}")
    
    def read8(self, reg: int) -> int:
        """读取 8 位寄存器（线程安全）"""
        with self._i2c_lock:
            try:
                return self.bus.read_byte_data(self.i2c_addr, reg)
            except Exception as e:
                print(f"Failed to read from I2C device: {e}")
                return 0
    
    def setPWMFreq(self, freq: float):
        """设置 PWM 频率（线程安全）"""
        # 限制频率范围
        freq = max(FREQ_MIN, min(FREQ_MAX, freq))
        
        # 计算预分频值
        prescaleval = OSC_FREQ / PWM_RESOLUTION
        freq_corrected = freq * FREQ_CORRECTION
        prescaleval = prescaleval / freq_corrected - 1.0
        prescale = int(prescaleval + 0.5)
        
        # 使用锁保护整个频率设置过程（包含多个 I2C 操作）
        with self._i2c_lock:
            # 读取当前模式并进入睡眠模式
            oldmode = self.bus.read_byte_data(self.i2c_addr, PCA9685_MODE1)
            newmode = (oldmode & 0x7F) | 0x10
            
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, newmode)
            self.bus.write_byte_data(self.i2c_addr, PCA9685_PRESCALE, prescale)
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, oldmode)
            
            time.sleep(0.005)  # 等待振荡器稳定
            
            # 启用自动增量
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, oldmode | 0x80)
    
    def setPWM(self, channel: int, on: int, off: int):
        """设置 PWM 输出（线程安全，原子操作）"""
        base_reg = LED0_ON_L + 4 * channel
        # 使用锁保护整个 PWM 设置过程，确保原子性
        with self._i2c_lock:
            try:
                self.bus.write_byte_data(self.i2c_addr, base_reg, on & 0xFF)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 1, on >> 8)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 2, off & 0xFF)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 3, off >> 8)
            except Exception as e:
                print(f"Failed to set PWM channel {channel}: {e}")


class DriverControl(Node):
    """电机驱动控制类，对应 C++ 中的 DriverControl 类，继承自 ROS2 Node"""
    
    def __init__(self, name: str = "driver_control"):
        super().__init__(name)
        
        # 初始化 GPIO，使用 BCM 编码
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 初始化 PCA9685
        self.pca9685 = PCA9685()
        
        # 重置 PCA9685 并停止所有电机
        self.pca9685.reset()
        self.pca9685.stopAllPWM()
        self.get_logger().info("PCA9685 已重置，所有电机已停止")
        
        # 添加位置更新锁（虽然每个线程操作不同的 motor，但为安全起见添加锁）
        self._position_lock = threading.Lock()
        
        # 加载电机参数（支持开发路径和安装路径）
        self.motors = []
        for i in range(NUM_MOTORS):
            # 优先使用安装路径（如果从安装目录运行），否则使用开发路径
            if 'install' in _SCRIPT_DIR:
                param_file = _INSTALL_PARAM_PATH.format(i + 1)
                # 如果安装路径不存在，尝试开发路径
                if not os.path.exists(param_file):
                    param_file = PARAM_FILE_TEMPLATE.format(i + 1)
            else:
                param_file = PARAM_FILE_TEMPLATE.format(i + 1)
                # 如果开发路径不存在，尝试安装路径
                if not os.path.exists(param_file):
                    param_file = _INSTALL_PARAM_PATH.format(i + 1)
            
            if not os.path.exists(param_file):
                raise FileNotFoundError(f"无法找到参数文件: {param_file}")
            
            self.motors.append(MotorParam(param_file))
        
        self.get_logger().info("Param init")
        
        # 初始化 GPIO
        for motor in self.motors:
            self._gpio_init(motor)
        
        self.get_logger().info("GPIO init")
        
        # 设置中断回调（使用参数文件中定义的中断引脚）
        for i, motor in enumerate(self.motors):
            GPIO.add_event_detect(
                motor.StepMotor_interrupt,
                GPIO.FALLING,
                callback=lambda ch, idx=i: self._interrupt_callback(idx),
                bouncetime=GPIO_BOUNCE_TIME
            )
        
        self.get_logger().info("Interrupt Init")
        
        # 创建 ROS2 服务服务器（串行处理，一次处理一个请求）
        self.motor_control_service = self.create_service(
            MotorControl,
            'motor_control_service',
            self.motor_control_callback
        )
        
        # Clock 对象通过 get_clock() 方法获取
        
        self.get_logger().info("Motor Control Service 已创建（串行处理模式）")
    
    def _gpio_init(self, motor: MotorParam):
        """GPIO 初始化"""
        GPIO.setup(motor.StepMotor_dir, GPIO.OUT)
        # 注意：Jetson.GPIO 不支持 pull_up_down 参数，因此省略
        GPIO.setup(motor.StepMotor_interrupt, GPIO.IN)
        self.pca9685.setPWMFreq(motor.StepMotor_Reset_Frequency)
    
    def _interrupt_callback(self, motor_idx: int):
        """统一的中断回调函数（优化：合并4个重复函数）"""
        global motor_reset_flags
        # 使用参数文件中定义的中断引脚
        interrupt_pin = self.motors[motor_idx].StepMotor_interrupt
        
        GPIO.setup(interrupt_pin, GPIO.IN)
        motor_reset_flags[motor_idx] = False
    
    def _reverse_direction(self, motor: MotorParam):
        """反转电机方向（优化：提取重复逻辑）"""
        current_state = GPIO.input(motor.StepMotor_dir)
        motor.motor_direction_flag = 1 if current_state == GPIO.LOW else 0
        GPIO.output(motor.StepMotor_dir, motor.motor_direction_flag)
    
    def _get_direction_multiplier(self, direction_flag: int) -> int:
        """获取方向乘数（优化：避免重复计算）"""
        return -1 if direction_flag == 1 else 1
    
    def Interrupt_RESET(self):
        """中断复位"""
        global motor_reset_flags
        
        motor_reset_flags = [True] * NUM_MOTORS
    
    def Calculation_time(self, frequency: int, distance: float) -> float:
        """计算运动时间（毫秒）"""
        return TIME_CALCULATION_FACTOR * distance / frequency
    
    def Calculation_distance(self, frequency: int, time_ms: int) -> float:
        """计算运动距离"""
        return (frequency * time_ms) / TIME_CALCULATION_FACTOR
    
    def Motor_Control(self, motor_id: int, motor: MotorParam, frequency: float, distance: float):
        """电机控制（线程安全）"""
        try:
            if distance > DISTANCE_THRESHOLD:
                time_duration_ms = self.Calculation_time(frequency, distance)
                # 启动电机
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_ON_VALUE)
                # 等待指定时间
                time.sleep(time_duration_ms / 1000.0)
                # 停止电机
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            else:
                # 即使距离很小，也要确保电机停止
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            
            # 更新位置（使用锁保护，确保线程安全）
            with self._position_lock:
                direction_mult = self._get_direction_multiplier(motor.motor_direction_flag)
                motor.StepMotor_Position += direction_mult * distance
        except Exception as e:
            # 确保即使发生异常，电机也能停止
            try:
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            except:
                pass
            self.get_logger().error(f"电机 {motor_id} 控制发生错误: {e}")
            raise
    
    def Motor_Run_time(self, time_duration_ms: int, motor_id: int):
        """电机运行指定时间"""
        self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_ON_VALUE)
        self.pca9685.setPWM(motor_id + 2, PWM_OFF_VALUE, PWM_ON_VALUE)
        time.sleep(time_duration_ms / 1000.0)
        self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
        self.pca9685.setPWM(motor_id + 2, PWM_OFF_VALUE, PWM_OFF_VALUE)
    
    def Switch_Motor(self, motor: MotorParam):
        """切换电机位置"""
        sign = -1.0 if motor.motor_direction_flag == 0 else 1.0
        motor.StepMotor_Position = sign * motor.Slide_block_compensation
    
    def _control_motors_parallel(self, motor_directions: List[int], 
                                  frequency: float, motor_distances: List[float]):
        """并行控制多个电机（线程安全版本）"""
        # 设置方向和标志
        for i, motor in enumerate(self.motors):
            GPIO.output(motor.StepMotor_dir, motor_directions[i])
            motor.motor_direction_flag = motor_directions[i]
        
        # 设置 PWM 频率（所有电机共享同一个频率，这是 PCA9685 硬件限制）
        self.pca9685.setPWMFreq(frequency)
        
        # 记录每个电机的运动参数（用于调试）
        self.get_logger().debug(f"并行控制电机 - 频率: {frequency} Hz")
        for i, (direction, distance) in enumerate(zip(motor_directions, motor_distances)):
            if distance > DISTANCE_THRESHOLD:
                time_ms = self.Calculation_time(frequency, distance)
                self.get_logger().debug(
                    f"电机 {i}: 方向={direction}, 距离={distance:.4f}mm, "
                    f"预计时间={time_ms:.2f}ms"
                )
        
        # 创建并启动线程
        threads = []
        for i, motor in enumerate(self.motors):
            thread = threading.Thread(
                target=self._motor_control_wrapper,
                args=(i, motor, frequency, motor_distances[i])
            )
            threads.append(thread)
        
        # 启动所有线程
        for thread in threads:
            thread.start()
        
        # 等待所有线程完成
        for i, thread in enumerate(threads):
            thread.join()
            self.get_logger().debug(f"电机 {i} 线程完成")
    
    def _motor_control_wrapper(self, motor_id: int, motor: MotorParam, 
                               frequency: float, distance: float):
        """电机控制包装函数（用于线程，添加日志）"""
        try:
            self.get_logger().debug(f"电机 {motor_id} 开始运行，距离: {distance:.4f}mm")
            self.Motor_Control(motor_id, motor, frequency, distance)
            self.get_logger().debug(f"电机 {motor_id} 运行完成，当前位置: {motor.StepMotor_Position:.4f}mm")
        except Exception as e:
            self.get_logger().error(f"电机 {motor_id} 运行失败: {e}")
            raise
    
    def Position_Reset(self, mode: int):
        """位置复位"""
        # 重置中断标志，确保复位操作从干净的状态开始
        self.Interrupt_RESET()
        
        if mode == RESET_MODE_LOCK:
            # 锁定模式：电机1和3一组，电机2和4一组
            groups = [
                MotorGroup(self.motors[0], self.motors[2], 1, 0, 2),
                MotorGroup(self.motors[1], self.motors[3], 2, 1, 3)
            ]
            
            threads = [
                threading.Thread(
                    target=self.Position_RESET_Lock,
                    args=(group.motor1, group.motor2, group.group_id, 
                          group.interrupt_idx1, group.interrupt_idx2)
                )
                for group in groups
            ]
            
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            
            self.get_logger().info(f"锁定模式复位完成")
        
        elif mode == RESET_MODE_UNLOCK:
            # 解锁模式：每个电机独立复位
            threads = [
                threading.Thread(
                    target=self.Position_RESET_Unlock,
                    args=(motor, i + 1, i)
                )
                for i, motor in enumerate(self.motors)
            ]
            
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            
            self.get_logger().info(f"解锁模式复位完成")
    
    def Position_RESET_Lock(self, motor_1: MotorParam, motor_2: MotorParam, 
                            group_id: int, interrupt_idx1: int, interrupt_idx2: int):
        """锁定模式位置复位（优化：简化逻辑）"""
        time_elapsed = 0
        during = RESET_DURING
        
        motor_interrupt_flags = [True, True]
        compensation_flags = [False, False]
        motors = [motor_1, motor_2]
        
        while ((any(motor_interrupt_flags) or not all(compensation_flags)) and 
               time_elapsed <= RESET_MAX_TIME):
            
            # 检查并补偿电机
            for i in range(2):
                if not (motor_interrupt_flags[i] ^ compensation_flags[i]):
                    self.Switch_Motor(motors[i])
                    self.pca9685.setPWMFreq(motors[i].StepMotor_Reset_Frequency)
                    self.Motor_Control(
                        group_id if i == 0 else group_id + 2,
                        motors[i],
                        motors[i].StepMotor_Reset_Frequency,
                        -motors[i].StepMotor_Position
                    )
                    compensation_flags[i] = True
            
            # 如果两个电机方向不同
            if motor_1.motor_direction_flag ^ motor_2.motor_direction_flag:
                interrupt_sum = sum(motor_interrupt_flags)
                
                if interrupt_sum == 2:
                    # 两个电机都未触发中断
                    if time_elapsed <= during:
                        self.Motor_Run_time(RESET_DTIME, group_id)
                        time_elapsed += RESET_DTIME
                    else:
                        self._reverse_direction(motor_1)
                        self._reverse_direction(motor_2)
                        time_elapsed = 0
                        during += RESET_DURING_INC
                
                elif interrupt_sum == 1:
                    # 只有一个电机触发中断
                    active_motor_idx = 0 if motor_interrupt_flags[0] else 1
                    inactive_motor_idx = 1 - active_motor_idx
                    
                    if time_elapsed <= during:
                        self.Motor_Run_time(RESET_DTIME, group_id)
                        
                        # 更新未触发中断电机的位置
                        inactive_motor = motors[inactive_motor_idx]
                        direction_mult = self._get_direction_multiplier(inactive_motor.motor_direction_flag)
                        inactive_motor.StepMotor_Position += direction_mult * self.Calculation_distance(
                            inactive_motor.StepMotor_Reset_Frequency, RESET_DTIME
                        )
                        
                        time_elapsed += RESET_DTIME
                    else:
                        self._reverse_direction(motor_1)
                        self._reverse_direction(motor_2)
                        time_elapsed = 0
                        during += RESET_DURING_INC
            else:
                # 两个电机方向相同，反转电机1方向
                motor_1.motor_direction_flag = not motor_1.motor_direction_flag
            
            # 更新中断标志
            motor_interrupt_flags[0] = motor_reset_flags[interrupt_idx1]
            motor_interrupt_flags[1] = motor_reset_flags[interrupt_idx2]
    
    def Position_RESET_Unlock(self, motor: MotorParam, motor_id: int, interrupt_idx: int):
        """解锁模式位置复位（优化：简化逻辑）"""
        time_elapsed = 0
        during = RESET_DURING
        motor_interrupt_flag = True
        compensation_flag = False
        
        while ((motor_interrupt_flag or not compensation_flag) and 
               time_elapsed <= RESET_MAX_TIME):
            
            if not (motor_interrupt_flag ^ compensation_flag):
                self.Switch_Motor(motor)
                self.pca9685.setPWMFreq(motor.StepMotor_Reset_Frequency)
                self.Motor_Control(
                    motor_id, motor,
                    motor.StepMotor_Reset_Frequency,
                    -motor.StepMotor_Position
                )
                compensation_flag = True
                break
            
            if time_elapsed <= during:
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_ON_VALUE)
                time.sleep(RESET_DTIME / 1000.0)
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
                time_elapsed += RESET_DTIME
            else:
                self._reverse_direction(motor)
                time_elapsed = 0
                during += RESET_DURING_INC
            
            motor_interrupt_flag = motor_reset_flags[interrupt_idx]
    
    def motor_control_callback(self, request: MotorControl.Request, 
                               response: MotorControl.Response):
        """ROS2 服务回调函数"""
        try:
            # 解析请求参数
            reset_mode = int(request.reset_mode)
            motor_directions = [int(bool(d)) for d in request.motor_direction]
            motor_frequency = float(request.motor_frequency)
            motor_distances = [float(d) for d in request.motor_distance]
            
            # 执行电机控制
            if reset_mode == RESET_MODE_MOTION:
                self._control_motors_parallel(motor_directions, motor_frequency, motor_distances)
            else:
                # 复位模式：等待复位操作完成
                self.get_logger().info(f"开始复位操作，模式: {reset_mode}")
                self.Position_Reset(reset_mode)
                self.get_logger().info(f"复位操作完成，模式: {reset_mode}")
            
            # 获取当前位置
            positions = [motor.StepMotor_Position for motor in self.motors]
            
            # 设置响应对象的 header（不重新创建，直接修改现有对象）
            response.header.stamp = self.get_clock().now().to_msg()
            if not hasattr(response.header, 'frame_id') or response.header.frame_id is None:
                response.header.frame_id = ''
            
            # 填充 motor_position 数组（使用切片赋值，保持数组对象引用不变）
            # 确保 positions 数组长度为 4
            while len(positions) < NUM_MOTORS:
                positions.append(0.0)
            positions = positions[:NUM_MOTORS]
            
            # 使用切片赋值，确保数组可写且保持引用
            motor_pos_array = np.array(positions, dtype=np.float32)
            response.motor_position[:] = motor_pos_array
            
            self.get_logger().info("服务请求处理完成")
            
            # ROS2 Python 服务回调函数必须返回响应对象
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"服务回调函数发生错误: {e}")
            traceback.print_exc()
            # 确保响应被填充，即使发生错误
            try:
                response.header.stamp = self.get_clock().now().to_msg()
                if not hasattr(response.header, 'frame_id') or response.header.frame_id is None:
                    response.header.frame_id = ''
                # 设置默认位置值（使用切片赋值）
                response.motor_position[:] = np.zeros(NUM_MOTORS, dtype=np.float32)
            except Exception as e2:
                self.get_logger().error(f"设置默认响应时发生错误: {e2}")
                traceback.print_exc()
            
            # 即使发生错误，也必须返回响应对象
            return response
    
    def MotorControlService(self, reset_mode: int, 
                           motor_direction: List[int], 
                           motor_frequency: float, 
                           motor_distance: List[float]) -> List[float]:
        """电机控制服务（直接调用版本，用于非 ROS2 环境）"""
        if reset_mode == RESET_MODE_MOTION:
            self._control_motors_parallel(motor_direction, motor_frequency, motor_distance)
        else:
            self.Position_Reset(reset_mode)
        
        return [motor.StepMotor_Position for motor in self.motors]
    
    def cleanup(self):
        """清理资源"""
        GPIO.cleanup()


# ==================== 主程序 ====================
def main():
    """主函数"""
    rclpy.init()
    
    try:
        driver = DriverControl("driver_control")
        driver.get_logger().info("DriverControl 节点已启动，等待服务请求...")
        driver.get_logger().info("服务名称: motor_control_service")
        rclpy.spin(driver)
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
        traceback.print_exc()
    finally:
        if 'driver' in locals():
            driver.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
