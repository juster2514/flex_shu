#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 PWM 测试程序
通过终端输入参数，控制 PCA9685 输出指定时间的 PWM 信号

依赖库：
    - Jetson.GPIO: Jetson 平台的 GPIO 控制库（可选，本程序不使用）
    - smbus: I2C 通信库（用于 PCA9685）

安装依赖：
    pip install smbus

使用方法：
    python3 pwm_test.py
"""

import smbus
import time
import sys
import subprocess
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

# PCA9685 频率限制
FREQ_MIN = 24
FREQ_MAX = 1526
FREQ_CORRECTION = 0.9615  # 频率修正系数
OSC_FREQ = 25000000.0     # 25MHz 振荡器频率
PWM_RESOLUTION = 4096.0   # 12-bit 分辨率

# PWM 控制常量
PWM_ON_VALUE = 0      # PWM 开启值（通常为 0）
PWM_OFF_MAX = 4095    # PWM 关闭值最大值（12-bit）
PWM_DUTY_CYCLE_50 = 2048  # 50% 占空比


def check_i2c_bus_usage(i2c_bus: int):
    """检查 I2C 总线是否被其他进程占用"""
    try:
        # 检查 lsof 命令是否可用
        result = subprocess.run(['which', 'lsof'], 
                              capture_output=True, 
                              text=True, 
                              timeout=2)
        if result.returncode != 0:
            return None, "lsof 命令不可用，无法检查 I2C 设备占用情况"
        
        # 检查是否有进程正在使用 I2C 设备
        device_path = f"/dev/i2c-{i2c_bus}"
        result = subprocess.run(['lsof', device_path], 
                              capture_output=True, 
                              text=True, 
                              timeout=2)
        
        if result.returncode == 0 and result.stdout.strip():
            processes = []
            lines = result.stdout.strip().split('\n')[1:]  # 跳过标题行
            for line in lines:
                parts = line.split()
                if len(parts) >= 2:
                    processes.append(f"  PID {parts[1]}: {parts[0]}")
            return processes, None
        else:
            return [], None
    except subprocess.TimeoutExpired:
        return None, "检查超时"
    except Exception as e:
        return None, f"检查失败: {e}"


class PCA9685:
    """PCA9685 PWM 驱动芯片控制类"""
    
    def __init__(self, i2c_bus: int = PCA9685_I2C_BUS, address: int = PCA9685_I2C_ADDR):
        """
        初始化 PCA9685
        
        Args:
            i2c_bus: I2C 总线编号（默认 1）
            address: I2C 设备地址（默认 0x40）
        """
        try:
            # 初始化 I2C 总线
            self.bus = smbus.SMBus(i2c_bus)
            self.i2c_addr = address
            
            # 重置 PCA9685
            self.write8(PCA9685_MODE1, 0x00)
            print(f"✓ PCA9685 初始化成功 (I2C总线: {i2c_bus}, 地址: 0x{address:02X})")
        except OSError as e:
            error_code = e.errno if hasattr(e, 'errno') else None
            error_msg = str(e)
            
            print(f"✗ PCA9685 初始化失败: {e}")
            print()
            
            # 针对 "Device or resource busy" 错误的特殊处理
            if error_code == 16 or "Device or resource busy" in error_msg or "busy" in error_msg.lower():
                print("⚠  I2C 设备正被其他进程占用！")
                print()
                
                # 检查是否有其他进程在使用 I2C 设备
                processes, check_error = check_i2c_bus_usage(i2c_bus)
                if processes is not None:
                    if processes:
                        print("检测到以下进程正在使用 I2C 设备:")
                        for proc in processes:
                            print(proc)
                        print()
                        print("解决方案:")
                        print("  1. 停止占用 I2C 设备的进程:")
                        print("     sudo pkill -f driver_control.py  # 如果 driver_control 在运行")
                        print("     sudo pkill -f flex_core           # 如果 ROS2 节点在运行")
                        print()
                        print("  2. 或者查找并手动终止进程:")
                        print(f"     lsof /dev/i2c-{i2c_bus}")
                        print("     然后使用: sudo kill <PID>")
                        print()
                        print("  3. 检查 ROS2 节点是否在运行:")
                        print("     ros2 node list")
                        print("     ros2 node kill /driver_control  # 如果存在")
                    else:
                        print("未检测到其他进程占用 I2C 设备，可能是内核驱动占用")
                        print()
                        print("解决方案:")
                        print("  1. 尝试使用 sudo 运行:")
                        print("     sudo python3 pwm_test.py")
                        print()
                        print("  2. 检查 I2C 设备权限:")
                        print(f"     ls -l /dev/i2c-{i2c_bus}")
                        print("     如果权限不足，运行:")
                        print(f"     sudo chmod 666 /dev/i2c-{i2c_bus}")
                else:
                    if check_error:
                        print(f"  无法检查进程占用: {check_error}")
                    print()
                    print("可能的解决方案:")
                    print("  1. 停止可能占用 I2C 的进程（如 driver_control.py）")
                    print("  2. 使用 sudo 运行此程序")
                    print("  3. 检查 I2C 设备权限")
            else:
                # 其他错误
                print("请检查:")
                print("  1. I2C 总线是否正确连接")
                print("  2. 是否已安装 smbus 库: pip install smbus")
                print("  3. 是否有 I2C 访问权限（可能需要 root 权限）")
                print(f"  4. I2C 设备是否存在: ls -l /dev/i2c-{i2c_bus}")
            
            sys.exit(1)
        except Exception as e:
            print(f"✗ PCA9685 初始化失败: {e}")
            print()
            print("请检查:")
            print("  1. I2C 总线是否正确连接")
            print("  2. 是否已安装 smbus 库: pip install smbus")
            print("  3. 是否有 I2C 访问权限（可能需要 root 权限）")
            sys.exit(1)
    
    def write8(self, reg: int, value: int):
        """写入 8 位寄存器"""
        try:
            self.bus.write_byte_data(self.i2c_addr, reg, value)
        except Exception as e:
            print(f"✗ 写入寄存器失败 (0x{reg:02X}): {e}")
            raise
    
    def read8(self, reg: int) -> int:
        """读取 8 位寄存器"""
        try:
            return self.bus.read_byte_data(self.i2c_addr, reg)
        except Exception as e:
            print(f"✗ 读取寄存器失败 (0x{reg:02X}): {e}")
            return 0
    
    def setPWMFreq(self, freq: float):
        """
        设置 PWM 频率
        
        Args:
            freq: PWM 频率 (Hz)，范围: 24-1526 Hz
        """
        # 限制频率范围
        freq = max(FREQ_MIN, min(FREQ_MAX, freq))
        
        print(f"设置 PWM 频率: {freq:.2f} Hz")
        
        # 计算预分频值
        prescaleval = OSC_FREQ / PWM_RESOLUTION
        freq_corrected = freq * FREQ_CORRECTION
        prescaleval = prescaleval / freq_corrected - 1.0
        prescale = int(prescaleval + 0.5)
        
        # 读取当前模式并进入睡眠模式
        oldmode = self.read8(PCA9685_MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        
        self.write8(PCA9685_MODE1, newmode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        
        time.sleep(0.005)  # 等待振荡器稳定
        
        # 启用自动增量
        self.write8(PCA9685_MODE1, oldmode | 0x80)
        
        print(f"✓ 频率设置完成 (预分频值: {prescale})")
    
    def setPWM(self, channel: int, on: int = 0, off: int = 0):
        """
        设置 PWM 输出
        
        Args:
            channel: PWM 通道 (0-15)
            on: PWM 开启点 (0-4095)
            off: PWM 关闭点 (0-4095)
        
        注意: 占空比 = (off - on) / 4096
        当 on=0, off=2048 时，占空比为 50%
        """
        if channel < 0 or channel > 15:
            raise ValueError(f"通道号必须在 0-15 之间，当前值: {channel}")
        
        if on < 0 or on > 4095 or off < 0 or off > 4095:
            raise ValueError(f"PWM 值必须在 0-4095 之间")
        
        base_reg = LED0_ON_L + 4 * channel
        self.write8(base_reg, on & 0xFF)
        self.write8(base_reg + 1, on >> 8)
        self.write8(base_reg + 2, off & 0xFF)
        self.write8(base_reg + 3, off >> 8)
    
    def setDutyCycle(self, channel: int, duty_cycle: float):
        """
        设置占空比（简化接口）
        
        Args:
            channel: PWM 通道 (0-15)
            duty_cycle: 占空比 (0.0-1.0)，0.0 表示 0%，1.0 表示 100%
        """
        if duty_cycle < 0.0 or duty_cycle > 1.0:
            raise ValueError(f"占空比必须在 0.0-1.0 之间，当前值: {duty_cycle}")
        
        off_value = int(duty_cycle * PWM_RESOLUTION)
        self.setPWM(channel, 0, off_value)
    
    def stopPWM(self, channel: int):
        """停止指定通道的 PWM 输出"""
        self.setPWM(channel, 0, 0)
    
    def stopAll(self):
        """停止所有通道的 PWM 输出"""
        for channel in range(16):
            self.stopPWM(channel)
        print("✓ 已停止所有 PWM 输出")


def get_user_input():
    """从终端获取用户输入"""
    print("\n" + "="*50)
    print("PCA9685 PWM 测试程序")
    print("="*50)
    
    # 输入 PWM 通道
    while True:
        try:
            channel = int(input("\n请输入 PWM 通道 (0-15): "))
            if 0 <= channel <= 15:
                break
            else:
                print("✗ 通道号必须在 0-15 之间，请重新输入")
        except ValueError:
            print("✗ 请输入有效的数字")
    
    # 输入 PWM 频率
    while True:
        try:
            freq = float(input("请输入 PWM 频率 (Hz, 24-1526): "))
            if FREQ_MIN <= freq <= FREQ_MAX:
                break
            else:
                print(f"✗ 频率必须在 {FREQ_MIN}-{FREQ_MAX} Hz 之间，请重新输入")
        except ValueError:
            print("✗ 请输入有效的数字")
    
    # 输入占空比
    while True:
        try:
            duty_input = input("请输入占空比 (0.0-1.0，直接回车默认 0.5 即 50%): ")
            if duty_input.strip() == "":
                duty_cycle = 0.5
                break
            else:
                duty_cycle = float(duty_input)
                if 0.0 <= duty_cycle <= 1.0:
                    break
                else:
                    print("✗ 占空比必须在 0.0-1.0 之间，请重新输入")
        except ValueError:
            print("✗ 请输入有效的数字")
    
    # 输入输出时间
    while True:
        try:
            duration = float(input("请输入输出时间 (秒): "))
            if duration > 0:
                break
            else:
                print("✗ 输出时间必须大于 0，请重新输入")
        except ValueError:
            print("✗ 请输入有效的数字")
    
    return channel, freq, duty_cycle, duration


def main():
    """主函数"""
    try:
        # 初始化 PCA9685
        pca9685 = PCA9685()
        
        while True:
            try:
                # 获取用户输入
                channel, freq, duty_cycle, duration = get_user_input()
                
                # 设置 PWM 频率
                pca9685.setPWMFreq(freq)
                
                # 计算并显示占空比信息
                off_value = int(duty_cycle * PWM_RESOLUTION)
                print(f"\n设置 PWM 参数:")
                print(f"  通道: {channel}")
                print(f"  频率: {freq:.2f} Hz")
                print(f"  占空比: {duty_cycle*100:.1f}% (off值: {off_value})")
                print(f"  输出时间: {duration:.3f} 秒")
                
                # 启动 PWM 输出
                print(f"\n开始输出 PWM 信号...")
                pca9685.setPWM(channel, 0, off_value)
                
                # 等待指定时间
                time.sleep(duration)
                
                # 停止 PWM 输出
                pca9685.stopPWM(channel)
                print(f"✓ PWM 输出已停止")
                
                # 询问是否继续
                continue_choice = input("\n是否继续测试? (y/n，直接回车默认 y): ").strip().lower()
                if continue_choice == 'n' or continue_choice == 'no':
                    break
                
            except KeyboardInterrupt:
                print("\n\n程序被用户中断")
                pca9685.stopAll()
                break
            except Exception as e:
                print(f"\n✗ 发生错误: {e}")
                pca9685.stopAll()
                continue
        
        # 清理：停止所有 PWM 输出
        print("\n正在清理资源...")
        pca9685.stopAll()
        print("程序退出")
        
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n✗ 程序发生错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

