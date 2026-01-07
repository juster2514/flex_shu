#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
行程开关测试脚本（仅检查40引脚）
基于 test_limit_switch.py，专门用于测试40引脚的行程开关

功能：
    - 控制电机运行
    - 当电机触碰到40引脚的行程开关时，触发下降沿中断
    - 返回触发消息
    - 返回运行时间

依赖库：
    - Jetson.GPIO: Jetson 平台的 GPIO 控制库
    - smbus: I2C 通信库（用于 PCA9685）
    - threading: 多线程支持
"""

import Jetson.GPIO as GPIO
import smbus
import time
import threading

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
GPIO_BOUNCE_TIME = 10  # 中断防抖时间（毫秒）

# 测试配置
TEST_PIN = 21  # 要测试的引脚号（BCM编码）
TEST_MOTOR_CHANNEL = 0  # 使用的电机通道（PCA9685通道0）
TEST_MOTOR_FREQUENCY = 200.0  # 测试电机运行频率（Hz）

# 测试运行序列（秒）：[方向, 运行时间]
# 正向（LOW）和反向（HIGH）交替运行，时间递增
TEST_RUN_SEQUENCE = [
    {'name': '正向', 'gpio_value': GPIO.LOW, 'direction_flag': 1, 'timeout': 5.0},
    {'name': '反向', 'gpio_value': GPIO.HIGH, 'direction_flag': 0, 'timeout': 10.0},
    {'name': '正向', 'gpio_value': GPIO.LOW, 'direction_flag': 1, 'timeout': 15.0},
    {'name': '反向', 'gpio_value': GPIO.HIGH, 'direction_flag': 0, 'timeout': 20.0},
]

# 方向引脚（需要根据实际情况配置）
DIRECTION_PIN = 17  # 默认方向引脚，可根据需要修改

# ==================== 全局变量 ====================
# 行程开关触发标志（True 表示未触发，False 表示已触发）
limit_switch_triggered = True

# 线程锁，用于保护共享变量
_switch_lock = threading.Lock()


# ==================== 类定义 ====================
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


class LimitSwitchTesterPin40:
    """40引脚行程开关测试类"""
    
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
        
        # 初始化 GPIO
        self._gpio_init()
        
        print("GPIO 初始化完成")
        
        # 设置中断回调（40引脚）
        GPIO.add_event_detect(
            TEST_PIN,
            GPIO.FALLING,  # 下降沿触发：从高电平到低电平
            callback=self._interrupt_callback,
            bouncetime=GPIO_BOUNCE_TIME
        )
        print(f"中断引脚 {TEST_PIN} 已配置为下降沿触发")
        
        print("中断初始化完成（下降沿触发模式）")
    
    def _gpio_init(self):
        """
        GPIO 初始化
        
        中断引脚配置说明（下降沿触发）：
        - 中断引脚应设置为输入模式，并启用上拉电阻（保持高电平）
        - 当行程开关未被接触时，中断引脚通过上拉电阻保持高电平
        - 当行程开关被接触时，行程开关导通，中断引脚与GND相连，电平从高变低
        - 产生下降沿中断，触发中断回调函数
        """
        GPIO.setup(DIRECTION_PIN, GPIO.OUT)
        
        # 设置中断引脚为输入模式
        # 注意：Jetson.GPIO 不支持 pull_up_down 参数（会被忽略），需要硬件上拉电阻
        # 下降沿触发时，引脚初始应为高电平，必须通过硬件上拉电阻实现
        GPIO.setup(TEST_PIN, GPIO.IN)
        print(f"中断引脚 {TEST_PIN} 已设置为输入模式（需要硬件上拉电阻保持高电平）")
        
        # 验证中断引脚初始状态（应该是高电平，用于下降沿触发）
        initial_state = GPIO.input(TEST_PIN)
        if initial_state == GPIO.HIGH:
            print(f"✓ 中断引脚 {TEST_PIN} 初始状态为高电平（正常，下降沿触发）")
        else:
            print(f"⚠ 警告：中断引脚 {TEST_PIN} 初始状态为低电平，可能需要检查上拉电阻")
        
        # 设置初始方向（假设向行程开关方向运行）
        GPIO.output(DIRECTION_PIN, GPIO.LOW)
        print(f"方向引脚 {DIRECTION_PIN} 已设置为输出模式")
    
    def _interrupt_callback(self, channel):
        """
        中断回调函数：当行程开关被触发时调用
        
        触发条件（下降沿触发）：
        - 中断引脚从高电平（未触发）变为低电平（触发）
        - 行程开关接触，导通中断引脚与GND，产生下降沿
        
        重要：在中断回调中立即停止电机，确保安全
        """
        global limit_switch_triggered
        
        # 读取当前引脚状态（应该是低电平，因为下降沿触发）
        current_state = GPIO.input(TEST_PIN)
        
        with _switch_lock:
            if limit_switch_triggered:
                limit_switch_triggered = False
                
                # 立即停止电机（在中断回调中直接停止，确保快速响应）
                self.pca9685.setPWM(TEST_MOTOR_CHANNEL, PWM_OFF_VALUE, PWM_OFF_VALUE)
                
                # 重新设置引脚为输入模式（确保状态正确）
                GPIO.setup(TEST_PIN, GPIO.IN)
                
                state_str = "低电平" if current_state == GPIO.LOW else "高电平"
                print(f"[中断] 40引脚行程开关被触发！已立即停止电机。引脚: {TEST_PIN}，当前状态: {state_str}")
    
    def reset_flag(self):
        """重置行程开关标志"""
        global limit_switch_triggered
        with _switch_lock:
            limit_switch_triggered = True
        print("行程开关标志已重置")
    
    def test_limit_switch(self) -> bool:
        """
        测试40引脚的行程开关（递增时间双向搜索）
        
        运行序列：
        1. 正向运行5秒
        2. 反向运行10秒
        3. 正向运行15秒
        4. 反向运行20秒
        
        返回:
            True: 成功触发行程开关
            False: 所有序列都未触发
        """
        global limit_switch_triggered
        
        # 重置标志
        self.reset_flag()
        
        print(f"\n开始测试40引脚行程开关")
        print(f"  方向引脚: {DIRECTION_PIN}")
        print(f"  中断引脚: {TEST_PIN}")
        print(f"  电机通道: {TEST_MOTOR_CHANNEL}")
        print(f"  运行频率: {TEST_MOTOR_FREQUENCY} Hz")
        print(f"  运行序列: 正向5秒 -> 反向10秒 -> 正向15秒 -> 反向20秒")
        
        # 设置 PWM 频率
        self.pca9685.setPWMFreq(TEST_MOTOR_FREQUENCY)
        
        # 记录总开始时间
        total_start_time = time.time()
        
        # 依次执行运行序列
        for seq_idx, run_config in enumerate(TEST_RUN_SEQUENCE):
            # 重置标志（准备新序列的测试）
            with _switch_lock:
                limit_switch_triggered = True
            
            # 设置电机方向
            GPIO.output(DIRECTION_PIN, run_config['gpio_value'])
            
            # 记录当前序列的开始时间
            sequence_start_time = time.time()
            timeout = run_config['timeout']
            
            # 启动电机
            self.pca9685.setPWM(TEST_MOTOR_CHANNEL, PWM_OFF_VALUE, PWM_ON_VALUE)
            print(f"序列 {seq_idx + 1}/4: 电机向{run_config['name']}运行 {timeout} 秒，等待触发行程开关...")
            
            # 轮询检查中断标志，直到触发或超时
            while True:
                current_time = time.time()
                sequence_elapsed = current_time - sequence_start_time
                
                # 检查当前序列超时
                if sequence_elapsed >= timeout:
                    print(f"[序列 {seq_idx + 1} 超时] 电机在{run_config['name']}运行 {sequence_elapsed:.2f} 秒内未触发")
                    self.pca9685.setPWM(TEST_MOTOR_CHANNEL, PWM_OFF_VALUE, PWM_OFF_VALUE)
                    # 短暂停顿，让电机停止
                    time.sleep(0.2)
                    # 跳出当前序列的循环，尝试下一个序列
                    break
                
                # 检查是否触发中断
                with _switch_lock:
                    if not limit_switch_triggered:
                        # 停止电机（虽然中断回调中已经停止，但这里再次确认）
                        self.pca9685.setPWM(TEST_MOTOR_CHANNEL, PWM_OFF_VALUE, PWM_OFF_VALUE)
                        total_elapsed = time.time() - total_start_time
                        print(f"[成功] 电机在序列 {seq_idx + 1} ({run_config['name']}) 运行 {total_elapsed:.2f} 秒后触发行程开关")
                        return True
                
                # 短暂休眠，避免 CPU 占用过高
                time.sleep(0.01)
        
        # 如果所有序列都未触发，返回失败
        total_elapsed = time.time() - total_start_time
        print(f"[失败] 电机在所有运行序列都未触发行程开关，总耗时: {total_elapsed:.2f} 秒")
        return False
    
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
        tester = LimitSwitchTesterPin40()
        
        print("=" * 60)
        print("开始40引脚行程开关测试")
        print("=" * 60)
        
        # 执行测试
        start_time = time.time()
        success = tester.test_limit_switch()
        elapsed_time = time.time() - start_time
        
        # 打印测试结果
        print("\n" + "=" * 60)
        print("测试结果")
        print("=" * 60)
        status = "✓ 成功" if success else "✗ 失败"
        print(f"状态: {status}")
        print(f"总耗时: {elapsed_time:.2f} 秒")
        print("=" * 60)
        
        return success
        
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
    result = main()
    if result is not None:
        sys.exit(0 if result else 1)
    else:
        sys.exit(1)

