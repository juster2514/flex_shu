#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DriverControl - 步进电机驱动控制模块

使用 Jetson GPIO 和 PCA9685 PWM 驱动芯片控制4个步进电机。
提供 ROS2 服务接口用于电机控制和位置复位。

服务接口: motor_control_service (flex_msgs/srv/MotorControl)
"""

import Jetson.GPIO as GPIO
import smbus
import yaml
import time
import threading
import os
import numpy as np
from typing import List
import traceback

# ROS2 相关导入
import rclpy
from rclpy.node import Node
from flex_msgs.srv import MotorControl

# ==================== 常量定义 ====================
# PCA9685 寄存器地址
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06

# PCA9685 配置
PCA9685_I2C_ADDR = 0x40
PCA9685_I2C_BUS = 7

# 电机控制常量
NUM_MOTORS = 4
RESET_MODE_MOTION = 2  # 运动模式
RESET_MODE_LOCK = 3    # 锁定模式复位

# 时间计算常量
TIME_CALCULATION_FACTOR = 200000.0
DISTANCE_THRESHOLD = 0.01  # 最小运动距离阈值（mm）

# PWM 控制常量
PWM_ON_VALUE = 2048
PWM_OFF_VALUE = 0

# 单电机复位参数（秒）
RESET_INITIAL_TIME = 5.0      # 初始运行时间
RESET_TIME_INCREMENT = 1.0    # 超时后增加的时间
RESET_MAX_RUN_TIME = 10.0     # 最大运行时间

# PCA9685 频率限制
FREQ_MIN = 24
FREQ_MAX = 1526
FREQ_CORRECTION = 0.9615
OSC_FREQ = 25000000.0     # 25MHz
PWM_RESOLUTION = 4096.0   # 12-bit

# GPIO 配置
GPIO_BOUNCE_TIME = 10  # 中断防抖时间（毫秒）

# 参数文件路径配置（支持开发目录和安装目录）
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if 'install' in _SCRIPT_DIR:
    _INSTALL_PARAM_PATH = os.path.join(_SCRIPT_DIR, '../../share/flex_core/params/StepMotor_{}_param.yaml')
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../../..')
    PARAM_FILE_TEMPLATE = os.path.join(_WORKSPACE_ROOT, "src/flex_core/params/StepMotor_{}_param.yaml")
else:
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../..')
    PARAM_FILE_TEMPLATE = os.path.join(_WORKSPACE_ROOT, "params/StepMotor_{}_param.yaml")
    _INSTALL_PARAM_PATH = os.path.join(_WORKSPACE_ROOT, "../../install/flex_core/share/flex_core/params/StepMotor_{}_param.yaml")

# ==================== 全局变量 ====================
motor_reset_flags = [True] * NUM_MOTORS  # 中断复位标志


class MotorParam:
    """电机参数类"""
    def __init__(self, motor_params_file: str):
        # 从 YAML 文件加载参数（过滤 OpenCV 格式指令行）
        with open(motor_params_file, 'r') as f:
            lines = f.readlines()
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
        self.bus = smbus.SMBus(i2c_bus)
        self.i2c_addr = address
        self._i2c_lock = threading.Lock()  # I2C 操作线程锁
        self.reset()
    
    def reset(self):
        """重置 PCA9685 芯片"""
        with self._i2c_lock:
            try:
                self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, 0x10)  # SLEEP 模式
                time.sleep(0.01)
                self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, 0xA0)  # 退出睡眠，启用自动增量
                time.sleep(0.01)
            except Exception as e:
                print(f"Failed to reset PCA9685: {e}")
    
    def stopAllPWM(self):
        """停止所有 PWM 通道输出"""
        with self._i2c_lock:
            try:
                for channel in range(16):
                    base_reg = LED0_ON_L + 4 * channel
                    self.bus.write_byte_data(self.i2c_addr, base_reg, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 1, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 2, 0x00)
                    self.bus.write_byte_data(self.i2c_addr, base_reg + 3, 0x00)
            except Exception as e:
                print(f"Failed to stop all PWM channels: {e}")
    
    def write8(self, reg: int, value: int):
        """写入 8 位寄存器"""
        with self._i2c_lock:
            try:
                self.bus.write_byte_data(self.i2c_addr, reg, value)
            except Exception as e:
                print(f"Failed to write to I2C device: {e}")
    
    def read8(self, reg: int) -> int:
        """读取 8 位寄存器"""
        with self._i2c_lock:
            try:
                return self.bus.read_byte_data(self.i2c_addr, reg)
            except Exception as e:
                print(f"Failed to read from I2C device: {e}")
                return 0
    
    def setPWMFreq(self, freq: float):
        """设置 PWM 频率"""
        freq = max(FREQ_MIN, min(FREQ_MAX, freq))
        
        prescaleval = OSC_FREQ / PWM_RESOLUTION
        freq_corrected = freq * FREQ_CORRECTION
        prescaleval = prescaleval / freq_corrected - 1.0
        prescale = int(prescaleval + 0.5)
        
        with self._i2c_lock:
            oldmode = self.bus.read_byte_data(self.i2c_addr, PCA9685_MODE1)
            newmode = (oldmode & 0x7F) | 0x10  # 进入睡眠模式
            
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, newmode)
            self.bus.write_byte_data(self.i2c_addr, PCA9685_PRESCALE, prescale)
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, oldmode)
            time.sleep(0.005)
            self.bus.write_byte_data(self.i2c_addr, PCA9685_MODE1, oldmode | 0x80)  # 启用自动增量
    
    def setPWM(self, channel: int, on: int, off: int):
        """设置 PWM 输出"""
        base_reg = LED0_ON_L + 4 * channel
        with self._i2c_lock:
            try:
                self.bus.write_byte_data(self.i2c_addr, base_reg, on & 0xFF)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 1, on >> 8)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 2, off & 0xFF)
                self.bus.write_byte_data(self.i2c_addr, base_reg + 3, off >> 8)
            except Exception as e:
                print(f"Failed to set PWM channel {channel}: {e}")


class DriverControl(Node):
    """电机驱动控制类"""
    
    def __init__(self, name: str = "driver_control"):
        super().__init__(name)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.pca9685 = PCA9685()
        self.pca9685.reset()
        self.pca9685.stopAllPWM()
        self.get_logger().info("PCA9685 已重置，所有电机已停止")
        
        self._position_lock = threading.Lock()
        
        # 加载电机参数
        self.motors = []
        for i in range(NUM_MOTORS):
            if 'install' in _SCRIPT_DIR:
                param_file = _INSTALL_PARAM_PATH.format(i + 1)
                if not os.path.exists(param_file):
                    param_file = PARAM_FILE_TEMPLATE.format(i + 1)
            else:
                param_file = PARAM_FILE_TEMPLATE.format(i + 1)
                if not os.path.exists(param_file):
                    param_file = _INSTALL_PARAM_PATH.format(i + 1)
            
            if not os.path.exists(param_file):
                raise FileNotFoundError(f"无法找到参数文件: {param_file}")
            
            self.motors.append(MotorParam(param_file))
        
        self.get_logger().info("参数初始化完成")
        
        # 初始化 GPIO
        for motor in self.motors:
            self._gpio_init(motor)
        
        self.get_logger().info("GPIO 初始化完成")
        
        # 创建 ROS2 服务
        self.motor_control_service = self.create_service(
            MotorControl,
            'motor_control_service',
            self.motor_control_callback
        )
        
        self.get_logger().info("Motor Control Service 已创建")
    
    def _gpio_init(self, motor: MotorParam):
        """GPIO 初始化"""
        GPIO.setup(motor.StepMotor_dir, GPIO.OUT)
        
        GPIO.setup(motor.StepMotor_interrupt, GPIO.IN)

        self.pca9685.setPWMFreq(motor.StepMotor_Reset_Frequency)
    
    def _interrupt_callback(self, motor_idx: int):
        """中断回调函数"""
        global motor_reset_flags
        motor_reset_flags[motor_idx] = False
        try:
            self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
        except Exception as e:
            self.get_logger().warn(f"停止电机 {motor_idx} PWM 失败: {e}")
    
    def _remove_interrupt(self, motor_idx: int):
        """移除指定电机的中断检测"""
        motor = self.motors[motor_idx]
        try:
            GPIO.remove_event_detect(motor.StepMotor_interrupt)
        except RuntimeError:
            # 如果中断检测不存在，忽略错误
            pass
    
    def _add_interrupt(self, motor_idx: int):
        """添加指定电机的中断检测（如果已存在则先移除）"""
        motor = self.motors[motor_idx]
        try:
            GPIO.remove_event_detect(motor.StepMotor_interrupt)
        except RuntimeError:
            pass
        
        # 添加新的中断检测
        GPIO.add_event_detect(
            motor.StepMotor_interrupt,
            GPIO.FALLING,
            callback=lambda ch, idx=motor_idx: self._interrupt_callback(idx),
            bouncetime=GPIO_BOUNCE_TIME
        )
    
    def _get_direction_multiplier(self, direction_flag: int) -> int:
        """获取方向乘数"""
        return -1 if direction_flag == 1 else 1
    
    def Interrupt_RESET(self):
        """中断复位"""
        global motor_reset_flags
        
        motor_reset_flags = [True] * NUM_MOTORS
    
    def Calculation_time(self, frequency: int, distance: float) -> float:
        """计算运动时间（毫秒）"""
        return TIME_CALCULATION_FACTOR * distance / frequency
    
    def Motor_Control(self, motor_id: int, motor: MotorParam, frequency: float, distance: float):
        """电机控制"""
        try:
            if distance > DISTANCE_THRESHOLD:
                time_duration_ms = self.Calculation_time(frequency, distance)
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_ON_VALUE)
                time.sleep(time_duration_ms / 1000.0)
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            else:
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            
            with self._position_lock:
                direction_mult = self._get_direction_multiplier(motor.motor_direction_flag)
                motor.StepMotor_Position += direction_mult * distance
        except Exception as e:
            try:
                self.pca9685.setPWM(motor_id, PWM_OFF_VALUE, PWM_OFF_VALUE)
            except:
                pass
            self.get_logger().error(f"电机 {motor_id} 控制发生错误: {e}")
            raise
    
    def _control_motors_parallel(self, motor_directions: List[int], 
                                  frequency: float, motor_distances: List[float]):
        """并行控制多个电机"""
        # 设置方向和频率
        for i, motor in enumerate(self.motors):
            GPIO.output(motor.StepMotor_dir, motor_directions[i])
            motor.motor_direction_flag = motor_directions[i]
        
        self.pca9685.setPWMFreq(frequency)
        
        # 创建并启动线程
        threads = []
        for i, motor in enumerate(self.motors):
            thread = threading.Thread(
                target=self._motor_control_wrapper,
                args=(i, motor, frequency, motor_distances[i])
            )
            threads.append(thread)
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
    
    def _motor_control_wrapper(self, motor_id: int, motor: MotorParam, 
                               frequency: float, distance: float):
        """电机控制包装函数（用于线程）"""
        try:
            self.Motor_Control(motor_id, motor, frequency, distance)
        except Exception as e:
            self.get_logger().error(f"电机 {motor_id} 运行失败: {e}")
            raise
    
    def Position_Reset(self, mode: int):
        """位置复位：单电机顺序复位"""
        self.Interrupt_RESET()
        
        # 为所有电机添加中断检测（复位开始时启用）
        for i in range(NUM_MOTORS):
            self._add_interrupt(i)
        
        success_motors = []
        failed_motors = []
        for i in range(NUM_MOTORS):
            success = self.Position_Reset_Single(i)
            if success:
                success_motors.append(i + 1)
                # 复位成功后移除中断检测
                self._remove_interrupt(i)
            else:
                failed_motors.append(i + 1)
                # 复位失败也移除中断检测
                self._remove_interrupt(i)
            time.sleep(0.2)  # 间隔，避免频繁切换 I2C/GPIO
        
        if success_motors:
            self.get_logger().info(f"复位成功：电机 {success_motors}")
        if failed_motors:
            self.get_logger().warn(f"复位失败：电机 {failed_motors}")
        
        # 如果所有电机都成功复位，执行补偿移动
        if len(success_motors) == NUM_MOTORS:
            self.get_logger().info("所有电机复位成功，开始补偿移动至0.0位置...")
            self._compensation_move_to_zero()
    
    def _compensation_move_to_zero(self):
        """
        补偿移动：将所有电机从补偿后的位置移动到0.0位置
        
        功能：在复位完成后，根据每个电机的当前位置（已应用补偿），
        计算需要移动的距离和方向，并行控制所有电机移动到0.0位置
        """
        # 使用复位频率进行补偿移动
        compensation_frequency = 200.0  # 补偿移动频率（Hz）
        
        # 计算每个电机需要移动的距离和方向
        motor_directions = []
        motor_distances = []
        
        with self._position_lock:
            for i, motor in enumerate(self.motors):
                current_pos = motor.StepMotor_Position
                
                # 计算需要移动的距离（绝对值）
                distance = abs(current_pos)
                
                # 确定运动方向
                # 如果当前位置 > 0，需要反向运动（方向=0）
                # 如果当前位置 < 0，需要正向运动（方向=1）
                # 如果当前位置 = 0，不需要运动
                if current_pos > DISTANCE_THRESHOLD:
                    direction = 0  # 反向运动
                elif current_pos < -DISTANCE_THRESHOLD:
                    direction = 1  # 正向运动
                else:
                    direction = 1  # 默认正向，距离为0时不会运动
                
                motor_directions.append(direction)
                motor_distances.append(distance)
                
                self.get_logger().debug(
                    f"电机 {i + 1}: 当前位置={current_pos:.4f}mm, "
                    f"目标距离={distance:.4f}mm, 方向={'正向' if direction == 1 else '反向'}"
                )
        
        # 检查是否有电机需要移动
        total_distance = sum(motor_distances)
        if total_distance < DISTANCE_THRESHOLD:
            self.get_logger().info("所有电机已在0.0位置，无需补偿移动")
            return
        
        # 并行控制所有电机移动到0.0位置
        self._control_motors_parallel(motor_directions, compensation_frequency, motor_distances)
        
        # 更新所有电机位置为0.0
        with self._position_lock:
            for motor in self.motors:
                motor.StepMotor_Position = 0.0
        
        self.get_logger().info("补偿移动完成，所有电机已到达0.0位置")
    
    def Position_Reset_Single(self, motor_idx: int) -> bool:
        """
        单电机复位
        
        搜索策略：
        1. 如果中断引脚为低电平（电机在行程开关上），先正向运动离开行程开关
        2. 初始正向运行，超时后切换方向并增加运行时间，直到触发或达到最大运行时间
        
        参数：
            motor_idx: 电机索引 (0-3)
        
        返回：
            True: 成功触发行程开关并复位
            False: 达到最大运行时间仍未触发
        """
        global motor_reset_flags
        motor = self.motors[motor_idx]
        
        self.pca9685.setPWMFreq(motor.StepMotor_Reset_Frequency)
        
        # 检查中断引脚状态：如果为低电平，说明电机正在行程开关上
        interrupt_pin_state = GPIO.input(motor.StepMotor_interrupt)
        if interrupt_pin_state == GPIO.LOW:
            self.get_logger().info(f"电机 {motor_idx + 1} 正在行程开关上，先正向运动离开...")
            
            # 移除中断检测，避免混乱
            self._remove_interrupt(motor_idx)
            
            # 设置正向方向
            GPIO.output(motor.StepMotor_dir, GPIO.LOW)
            motor.motor_direction_flag = 1
            
            # 启动电机，正向运动
            self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_ON_VALUE)
            
            # 轮询等待引脚变为高电平（离开行程开关）
            max_wait_time = 5.0  # 最大等待时间（秒）
            wait_start_time = time.time()
            while True:
                current_state = GPIO.input(motor.StepMotor_interrupt)
                if current_state == GPIO.HIGH:
                    # 已离开行程开关，停止电机
                    self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    self.get_logger().info(f"电机 {motor_idx + 1} 已离开行程开关")
                    break
                
                # 检查超时
                if time.time() - wait_start_time >= max_wait_time:
                    self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    self.get_logger().warn(f"电机 {motor_idx + 1} 离开行程开关超时")
                    # 重新添加中断检测
                    self._add_interrupt(motor_idx)
                    return False
                
                time.sleep(0.02)
            
            # 短暂延时，确保稳定
            time.sleep(0.1)
            
            # 重新添加中断检测
            self._add_interrupt(motor_idx)
        
        current_run_time = RESET_INITIAL_TIME
        current_direction = 1  # 1=正向(LOW), 0=反向(HIGH)
        
        while current_run_time <= RESET_MAX_RUN_TIME:
            motor_reset_flags[motor_idx] = True
            
            gpio_value = GPIO.LOW if current_direction == 1 else GPIO.HIGH
            GPIO.output(motor.StepMotor_dir, gpio_value)
            motor.motor_direction_flag = current_direction
            
            sequence_start_time = time.time()
            self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_ON_VALUE)
            
            while True:
                sequence_elapsed = time.time() - sequence_start_time
                
                if sequence_elapsed >= current_run_time:
                    self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    
                    if current_run_time >= RESET_MAX_RUN_TIME:
                        self.get_logger().warn(f"电机 {motor_idx + 1} 复位失败")
                        return False
                    
                    current_direction = 1 - current_direction
                    current_run_time += RESET_TIME_INCREMENT
                    break
                
                if not motor_reset_flags[motor_idx]:
                    self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    
                    with self._position_lock:
                        if current_direction == 1:  # 正向运动
                            motor.StepMotor_Position = 0.0 - motor.Slide_block_compensation
                        else:  # 反向运动
                            motor.StepMotor_Position = 0.0 + motor.Slide_block_compensation

                    return True
                
                time.sleep(0.01)
        
        self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
        self.get_logger().warn(f"电机 {motor_idx + 1} 复位失败")
        return False
    
    def motor_control_callback(self, request: MotorControl.Request, 
                               response: MotorControl.Response):
        """ROS2 服务回调函数"""
        try:
            reset_mode = int(request.reset_mode)
            motor_directions = [int(bool(d)) for d in request.motor_direction]
            motor_frequency = float(request.motor_frequency)
            motor_distances = [float(d) for d in request.motor_distance]
            
            if reset_mode == RESET_MODE_MOTION:
                self._control_motors_parallel(motor_directions, motor_frequency, motor_distances)
            else:
                self.get_logger().info(f"开始复位操作，模式: {reset_mode}")
                self.Position_Reset(reset_mode)
                self.get_logger().info(f"复位操作完成，模式: {reset_mode}")
            
            # 填充响应
            positions = [motor.StepMotor_Position for motor in self.motors]
            while len(positions) < NUM_MOTORS:
                positions.append(0.0)
            positions = positions[:NUM_MOTORS]
            
            response.header.stamp = self.get_clock().now().to_msg()
            if not hasattr(response.header, 'frame_id') or response.header.frame_id is None:
                response.header.frame_id = ''
            
            motor_pos_array = np.array(positions, dtype=np.float32)
            response.motor_position[:] = motor_pos_array
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"服务回调函数发生错误: {e}")
            traceback.print_exc()
            try:
                response.header.stamp = self.get_clock().now().to_msg()
                if not hasattr(response.header, 'frame_id') or response.header.frame_id is None:
                    response.header.frame_id = ''
                response.motor_position[:] = np.zeros(NUM_MOTORS, dtype=np.float32)
            except Exception as e2:
                self.get_logger().error(f"设置默认响应时发生错误: {e2}")
                traceback.print_exc()
            
            return response
    
    def cleanup(self):
        """清理资源"""
        GPIO.cleanup()


def main():
    """主函数"""
    rclpy.init()
    
    try:
        driver = DriverControl("driver_control")
        driver.get_logger().info("DriverControl 节点已启动，等待服务请求...")
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
