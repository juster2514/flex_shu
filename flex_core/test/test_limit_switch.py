#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
行程开关测试脚本
仿照 driver_control.py 中的控制电机方法，顺序测试每个电机的行程开关

功能：
    - 每次控制单个电机运行
    - 当电机触碰到对应的行程开关时，触发下降沿中断
    - 返回触发消息
    - 直到四个电机都触碰到行程开关后停止
    - 返回总运行时间

依赖库：
    - Jetson.GPIO: Jetson 平台的 GPIO 控制库
    - smbus: I2C 通信库（用于 PCA9685）
    - yaml: YAML 配置文件解析
    - threading: 多线程支持
"""

import Jetson.GPIO as GPIO
import smbus
import yaml
import time
import threading
import os

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

# 电机控制常量
NUM_MOTORS = 4

# PWM 控制常量
PWM_ON_VALUE = 2048  # PWM 开启值
PWM_OFF_VALUE = 0     # PWM 关闭值

# PCA9685 频率限制
FREQ_MIN = 24
FREQ_MAX = 1526
FREQ_CORRECTION = 0.9615  # 频率修正系数
OSC_FREQ = 25000000.0     # 25MHz 振荡器频率
PWM_RESOLUTION = 4096.0   # 12-bit 分辨率

# GPIO 配置
GPIO_BOUNCE_TIME = 100  # 中断防抖时间（毫秒）

# 测试电机运行频率（Hz）
TEST_MOTOR_FREQUENCY = 200.0

# 测试运行序列（秒）：[方向, 运行时间]
# 正向（LOW）和反向（HIGH）交替运行，时间递增
TEST_RUN_SEQUENCE = [
    {'name': '正向', 'gpio_value': GPIO.LOW, 'direction_flag': 1, 'timeout': 5.0},
    {'name': '反向', 'gpio_value': GPIO.HIGH, 'direction_flag': 0, 'timeout': 10.0},
    {'name': '正向', 'gpio_value': GPIO.LOW, 'direction_flag': 1, 'timeout': 15.0},
    {'name': '反向', 'gpio_value': GPIO.HIGH, 'direction_flag': 0, 'timeout': 20.0},
]

# 参数文件路径模板
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 判断脚本是在开发目录还是安装目录
if 'install' in _SCRIPT_DIR:
    # 从安装目录运行：install/flex_core/lib/flex_core/test/ -> install/flex_core/share/flex_core/params/
    _INSTALL_PARAM_PATH = os.path.join(_SCRIPT_DIR, '../../share/flex_core/params/StepMotor_{}_param.yaml')
    # 开发路径（可能不存在）
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../../..')
    PARAM_FILE_TEMPLATE = os.path.join(_WORKSPACE_ROOT, "src/flex_core/params/StepMotor_{}_param.yaml")
else:
    # 从开发目录运行：src/flex_core/test/ -> src/flex_core/params/
    PARAM_FILE_TEMPLATE = os.path.join(_SCRIPT_DIR, '../params/StepMotor_{}_param.yaml')
    # 安装路径（可能不存在）
    # 从 test 目录到工作空间根目录：test -> flex_core -> src -> flex_shu_ws
    _WORKSPACE_ROOT = os.path.join(_SCRIPT_DIR, '../../..')
    _INSTALL_PARAM_PATH = os.path.join(_WORKSPACE_ROOT, "install/flex_core/share/flex_core/params/StepMotor_{}_param.yaml")

# ==================== 全局变量 ====================
# 行程开关触发标志（True 表示未触发，False 表示已触发）
limit_switch_flags = [True] * NUM_MOTORS

# 线程锁，用于保护共享变量
_switch_lock = threading.Lock()


# ==================== 类定义 ====================
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


class LimitSwitchTester:
    """行程开关测试类"""
    
    def __init__(self):
        """初始化测试器"""
        # 初始化 GPIO，使用 BCM 编码
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 初始化 PCA9685
        self.pca9685 = PCA9685()
        
        # 重置 PCA9685 并停止所有电机
        self.pca9685.reset()
        self.pca9685.stopAllPWM()
        print("PCA9685 已重置，所有电机已停止")
        
        # 加载电机参数
        self.motors = []
        for i in range(NUM_MOTORS):
            # 优先使用开发路径，如果不存在则使用安装路径
            if 'install' not in _SCRIPT_DIR:
                param_file = PARAM_FILE_TEMPLATE.format(i + 1)
                if not os.path.exists(param_file):
                    param_file = _INSTALL_PARAM_PATH.format(i + 1)
            else:
                param_file = _INSTALL_PARAM_PATH.format(i + 1)
                if not os.path.exists(param_file):
                    param_file = PARAM_FILE_TEMPLATE.format(i + 1)
            
            if not os.path.exists(param_file):
                raise FileNotFoundError(f"无法找到参数文件: {param_file}")
            
            self.motors.append(MotorParam(param_file))
        
        print("电机参数加载完成")
        
        # 初始化 GPIO
        for motor in self.motors:
            self._gpio_init(motor)
        
        print("GPIO 初始化完成")
        
        # 设置中断回调（使用参数文件中定义的中断引脚）
        # 使用下降沿触发：当行程开关接触时，中断引脚从高电平变为低电平
        for i, motor in enumerate(self.motors):
            GPIO.add_event_detect(
                motor.StepMotor_interrupt,
                GPIO.FALLING,  # 下降沿触发：从高电平到低电平
                callback=lambda ch, idx=i: self._interrupt_callback(idx),
                bouncetime=GPIO_BOUNCE_TIME
            )
            print(f"  中断引脚 {motor.StepMotor_interrupt} 已配置为下降沿触发")
        
        print("中断初始化完成（下降沿触发模式）")
    
    def _gpio_init(self, motor: MotorParam):
        """
        GPIO 初始化
        
        中断引脚配置说明（下降沿触发）：
        - 中断引脚应设置为输入模式，并启用上拉电阻（保持高电平）
        - 当行程开关未被接触时，中断引脚通过上拉电阻保持高电平
        - 当行程开关被接触时，行程开关导通，中断引脚与GND相连，电平从高变低
        - 产生下降沿中断，触发中断回调函数
        """
        GPIO.setup(motor.StepMotor_dir, GPIO.OUT)
        
        # 设置中断引脚为输入模式
        # 注意：Jetson.GPIO 不支持 pull_up_down 参数（会被忽略），需要硬件上拉电阻
        # 下降沿触发时，引脚初始应为高电平，必须通过硬件上拉电阻实现
        GPIO.setup(motor.StepMotor_interrupt, GPIO.IN)
        print(f"  中断引脚 {motor.StepMotor_interrupt} 已设置为输入模式（需要硬件上拉电阻保持高电平）")
        
        # 验证中断引脚初始状态（应该是高电平，用于下降沿触发）
        initial_state = GPIO.input(motor.StepMotor_interrupt)
        if initial_state == GPIO.HIGH:
            print(f"  ✓ 中断引脚 {motor.StepMotor_interrupt} 初始状态为高电平（正常，下降沿触发）")
        else:
            print(f"  ⚠ 警告：中断引脚 {motor.StepMotor_interrupt} 初始状态为低电平，可能需要检查上拉电阻")
        
        # 设置初始方向（假设向行程开关方向运行）
        GPIO.output(motor.StepMotor_dir, GPIO.LOW)
        motor.motor_direction_flag = 1
    
    def _interrupt_callback(self, motor_idx: int):
        """
        中断回调函数：当行程开关被触发时调用
        
        触发条件（下降沿触发）：
        - 中断引脚从高电平（未触发）变为低电平（触发）
        - 行程开关接触，导通中断引脚与GND，产生下降沿
        
        重要：在中断回调中立即停止电机，确保安全
        """
        global limit_switch_flags
        
        interrupt_pin = self.motors[motor_idx].StepMotor_interrupt
        
        # 读取当前引脚状态（应该是低电平，因为下降沿触发）
        current_state = GPIO.input(interrupt_pin)
        
        with _switch_lock:
            if limit_switch_flags[motor_idx]:
                limit_switch_flags[motor_idx] = False
                
                # 立即停止电机（在中断回调中直接停止，确保快速响应）
                self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                
                # 重新设置引脚为输入模式（确保状态正确）
                # 注意：Jetson.GPIO 不支持 pull_up_down 参数，需要硬件上拉电阻
                GPIO.setup(interrupt_pin, GPIO.IN)
                
                state_str = "低电平" if current_state == GPIO.LOW else "高电平"
                print(f"[中断] 电机 {motor_idx + 1} 触碰到行程开关！已立即停止电机。引脚: {interrupt_pin}，当前状态: {state_str}")
    
    def reset_flags(self):
        """重置所有行程开关标志"""
        global limit_switch_flags
        with _switch_lock:
            limit_switch_flags = [True] * NUM_MOTORS
        print("行程开关标志已重置")
    
    def test_single_motor(self, motor_idx: int) -> bool:
        """
        测试单个电机的行程开关（递增时间双向搜索）
        
        运行序列：
        1. 正向运行5秒
        2. 反向运行10秒
        3. 正向运行15秒
        4. 反向运行20秒
        
        参数:
            motor_idx: 电机索引 (0-3)
        
        返回:
            True: 成功触发行程开关
            False: 所有序列都未触发
        """
        global limit_switch_flags
        
        motor = self.motors[motor_idx]
        
        # 重置该电机的标志
        with _switch_lock:
            limit_switch_flags[motor_idx] = True
        
        print(f"\n开始测试电机 {motor_idx + 1}")
        print(f"  方向引脚: {motor.StepMotor_dir}")
        print(f"  中断引脚: {motor.StepMotor_interrupt}")
        print(f"  运行频率: {TEST_MOTOR_FREQUENCY} Hz")
        print(f"  运行序列: 正向5秒 -> 反向10秒 -> 正向15秒 -> 反向20秒")
        
        # 设置 PWM 频率
        self.pca9685.setPWMFreq(TEST_MOTOR_FREQUENCY)
        
        # 记录总开始时间
        total_start_time = time.time()
        
        # 依次执行运行序列
        for seq_idx, run_config in enumerate(TEST_RUN_SEQUENCE):
            # 重置该电机的标志（准备新序列的测试）
            with _switch_lock:
                limit_switch_flags[motor_idx] = True
            
            # 设置电机方向
            GPIO.output(motor.StepMotor_dir, run_config['gpio_value'])
            motor.motor_direction_flag = run_config['direction_flag']
            
            # 记录当前序列的开始时间
            sequence_start_time = time.time()
            timeout = run_config['timeout']
            
            # 启动电机
            self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_ON_VALUE)
            print(f"  序列 {seq_idx + 1}/4: 电机 {motor_idx + 1} 向{run_config['name']}运行 {timeout} 秒，等待触发行程开关...")
            
            # 轮询检查中断标志，直到触发或超时
            while True:
                current_time = time.time()
                sequence_elapsed = current_time - sequence_start_time
                
                # 检查当前序列超时
                if sequence_elapsed >= timeout:
                    print(f"  [序列 {seq_idx + 1} 超时] 电机 {motor_idx + 1} 在{run_config['name']}运行 {sequence_elapsed:.2f} 秒内未触发")
                    self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    # 短暂停顿，让电机停止
                    time.sleep(0.2)
                    # 跳出当前序列的循环，尝试下一个序列
                    break
                
                # 检查是否触发中断
                with _switch_lock:
                    if not limit_switch_flags[motor_idx]:
                        # 停止电机
                        self.pca9685.setPWM(motor_idx, PWM_OFF_VALUE, PWM_OFF_VALUE)
                        total_elapsed = time.time() - total_start_time
                        print(f"  [成功] 电机 {motor_idx + 1} 在序列 {seq_idx + 1} ({run_config['name']}) 运行 {total_elapsed:.2f} 秒后触发行程开关")
                        return True
                
                # 短暂休眠，避免 CPU 占用过高
                time.sleep(0.01)
        
        # 如果所有序列都未触发，返回失败
        total_elapsed = time.time() - total_start_time
        print(f"  [失败] 电机 {motor_idx + 1} 在所有运行序列都未触发行程开关，总耗时: {total_elapsed:.2f} 秒")
        return False
    
    def test_all_motors(self) -> dict:
        """
        顺序测试所有电机的行程开关
        
        返回:
            dict: 包含测试结果的字典
                - success: bool - 是否所有电机都成功触发
                - total_time: float - 总运行时间（秒）
                - motor_results: list - 每个电机的测试结果
        """
        print("=" * 60)
        print("开始行程开关测试")
        print("=" * 60)
        
        # 重置所有标志
        self.reset_flags()
        
        # 记录总开始时间
        total_start_time = time.time()
        
        # 存储每个电机的测试结果
        motor_results = []
        all_success = True
        
        # 顺序测试每个电机
        for i in range(NUM_MOTORS):
            motor_start_time = time.time()
            success = self.test_single_motor(i)
            motor_elapsed = time.time() - motor_start_time
            
            motor_results.append({
                'motor_id': i + 1,
                'success': success,
                'time': motor_elapsed
            })
            
            if not success:
                all_success = False
            
            # 测试完成后短暂停顿
            time.sleep(0.5)
        
        # 计算总时间
        total_time = time.time() - total_start_time
        
        # 打印测试结果摘要
        print("\n" + "=" * 60)
        print("测试结果摘要")
        print("=" * 60)
        for result in motor_results:
            status = "✓ 成功" if result['success'] else "✗ 失败"
            print(f"电机 {result['motor_id']}: {status} - 耗时: {result['time']:.2f} 秒")
        
        print(f"\n总运行时间: {total_time:.2f} 秒")
        print(f"总体状态: {'✓ 全部成功' if all_success else '✗ 部分失败'}")
        print("=" * 60)
        
        return {
            'success': all_success,
            'total_time': total_time,
            'motor_results': motor_results
        }
    
    def cleanup(self):
        """清理资源"""
        # 停止所有电机
        self.pca9685.stopAllPWM()
        print("\n所有电机已停止")
        
        # 清理 GPIO
        GPIO.cleanup()
        print("GPIO 已清理")


# ==================== 主程序 ====================
def main():
    """主函数"""
    tester = None
    
    try:
        # 创建测试器
        tester = LimitSwitchTester()
        
        # 执行测试
        results = tester.test_all_motors()
        
        # 返回结果
        return results
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        return None
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None
    finally:
        if tester is not None:
            tester.cleanup()


if __name__ == "__main__":
    import sys
    results = main()
    if results:
        print(f"\n测试完成，总耗时: {results['total_time']:.2f} 秒")
        sys.exit(0 if results['success'] else 1)
    else:
        sys.exit(1)
