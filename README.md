# 柔性机械臂控制系统


基于 ROS2 的柔性机械臂控制系统，运行在 NVIDIA Jetson 平台上。该系统集成了电机驱动控制、MFAC（无模型自适应控制）算法、遥控器解析和硬件接口控制等功能。

## 📋 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [项目结构](#项目结构)
- [安装与编译](#安装与编译)
- [快速开始](#快速开始)
- [使用说明](#使用说明)
- [测试](#测试)
- [API 文档](#api-文档)
- [常见问题](#常见问题)

## ✨ 功能特性

- **电机驱动控制**: 基于 PCA9685 PWM 驱动芯片的电机控制，支持多通道 PWM 输出
- **MFAC 无模型自适应控制**: 实现无模型自适应控制算法，用于柔性机械臂的运动控制
- **遥控器解析**: 支持多通道遥控器数据解析和处理
- **GPIO 控制**: 基于 Jetson.GPIO 的硬件 GPIO 控制，支持中断处理
- **行程开关检测**: 支持行程开关的硬件中断检测和安全保护
- **ROS2 集成**: 完整的 ROS2 节点、话题、服务接口

## 🖥️ 系统要求

### 硬件要求
- **开发板**: NVIDIA Jetson 系列（Jetson Nano/Xavier/Orin 等）
- **PCA9685**: I2C PWM 驱动板（地址：0x40，I2C 总线：7）

### 软件要求
- **操作系统**: Ubuntu 20.04 / Ubuntu 22.04（推荐）
- **ROS2**: ROS2 Humble / ROS2 Foxy（推荐 Humble）
- **Python**: Python 3.8+
- **编译器**: GCC 9+ / Clang 10+

## 📁 项目结构

```
flex_shu_ws/
├── src/
│   ├── README.md               # 本文档
│   ├── flex_core/              # 核心控制包
│   │   ├── include/           # C++ 头文件
│   │   │   └── flex_core/
│   │   │       ├── FlexCore.hpp
│   │   │       └── MFAC.hpp
│   │   ├── src/               # C++ 源文件
│   │   │   ├── FlexCore.cpp
│   │   │   └── MFAC.cpp
│   │   ├── node/              # ROS2 节点
│   │   │   ├── FlexCoreNode.cpp
│   │   │   └── RemoteCoreNode.cpp
│   │   ├── scripts/           # Python 脚本
│   │   │   └── driver_control.py
│   │   ├── test/              # 测试脚本
│   │   │   └── test_limit_switch_pin40.py
│   │   ├── launch/            # Launch 文件
│   │   │   └── driver_control.launch.py
│   │   ├── params/            # 参数配置文件
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── flex_msgs/             # 自定义消息包
│       ├── msg/               # 消息定义
│       ├── srv/               # 服务定义
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                      # 编译输出目录
├── install/                    # 安装目录
└── log/                        # 日志目录
```

## 🔧 安装与编译

### 1. 安装系统依赖

```bash
# 更新系统包
sudo apt update && sudo apt upgrade -y

# 安装 ROS2（如果未安装）
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装构建工具和依赖
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libqt5-widgets-dev \
    libqt5-core-dev \
    libqt5-serialport-dev
```

### 2. 安装 Python 依赖

```bash
# 安装 Jetson GPIO 库（Jetson 平台专用）
sudo pip3 install Jetson.GPIO

# 安装其他 Python 依赖
sudo pip3 install smbus pyyaml
```

### 3. 配置 GPIO 和 I2C 权限

```bash
# 将当前用户添加到 gpio 和 i2c 组
sudo usermod -a -G gpio,i2c $USER

# 注意：需要重新登录才能使权限生效
```

### 4. 克隆并编译工作空间

```bash
# 进入工作空间目录
cd ~/flex_shu_ws

# 编译所有包
colcon build

# 或者只编译特定包
colcon build --packages-select flex_msgs flex_core

# 设置环境变量
source install/setup.bash

# 将环境变量添加到 ~/.bashrc（可选）
echo "source ~/flex_shu_ws/install/setup.bash" >> ~/.bashrc
```

## 🚀 快速开始

### 启动驱动控制节点

**方法 1: 使用 launch 文件（推荐）**
```bash
source install/setup.bash
ros2 launch flex_core driver_control.launch.py
```

**方法 2: 使用 ros2 run**
```bash
source install/setup.bash
ros2 run flex_core driver_control.py
```

**方法 3: 直接运行 Python 脚本**
```bash
cd ~/flex_shu_ws
source install/setup.bash
python3 src/flex_core/scripts/driver_control.py
```

### 验证节点运行

打开另一个终端：

```bash
source install/setup.bash

# 查看节点列表
ros2 node list
# 应该看到: /driver_control

# 查看服务列表
ros2 service list
# 应该看到: /motor_control_service

# 查看话题列表
ros2 topic list
```

### 成功标志

如果看到以下输出，说明节点运行成功：

```
[INFO] [driver_control]: Param init
[INFO] [driver_control]: GPIO init
[INFO] [driver_control]: Interrupt Init
[INFO] [driver_control]: Motor Control Service 已创建
[INFO] [driver_control]: DriverControl 节点已启动，等待服务请求...
```

## 📖 使用说明

### 电机控制服务

通过 ROS2 服务调用控制电机：

```bash
# 调用电机控制服务
ros2 service call /motor_control_service flex_msgs/srv/MotorControl \
  "{channel: 0, frequency: 200.0, direction: 1, enable: true}"
```

### 行程开关测试

运行行程开关测试脚本（测试 40 引脚）：

```bash
cd ~/flex_shu_ws/src/flex_core/test
python3 test_limit_switch_pin40.py
```

测试脚本会执行以下序列：
1. 正向运行 5 秒
2. 反向运行 10 秒
3. 正向运行 15 秒
4. 反向运行 20 秒

当触发行程开关时，电机会立即停止。

### 遥控器数据订阅

系统会自动订阅遥控器数据话题：

```bash
# 查看遥控器数据
ros2 topic echo /remote_ctrl_data
```

## 🧪 测试

### 运行测试

```bash
# 编译测试
colcon build --packages-select flex_core --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select flex_core

# 查看测试结果
colcon test-result --verbose
```

### 硬件测试

1. **I2C 设备检测**
   ```bash
   sudo i2cdetect -y 7
   # 应该看到 0x40 (PCA9685)
   ```

2. **GPIO 测试**
   ```bash
   # 测试 GPIO 引脚状态
   python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO OK')"
   ```

## 📚 API 文档

### ROS2 话题

- `/remote_ctrl_data` (flex_msgs/msg/RemoteControl): 遥控器控制数据
- `/driver_control_data` (flex_msgs/msg/DriverControl): 驱动控制数据
- `/driver_callback_data` (flex_msgs/msg/DriverCallback): 驱动回调数据

### ROS2 服务

- `/motor_control_service` (flex_msgs/srv/MotorControl): 电机控制服务

### 主要类

- **FlexCore**: 核心控制类，实现 MFAC 控制和遥控器解析
- **PCA9685**: PWM 驱动芯片控制类

## ❓ 常见问题

### 权限问题

**问题**: `Permission denied` 或无法访问 GPIO/I2C

**解决**:
```bash
sudo usermod -a -G gpio,i2c $USER
# 重新登录后生效
```

### 找不到模块

**问题**: `ModuleNotFoundError: No module named 'flex_msgs'`

**解决**:
```bash
cd ~/flex_shu_ws
colcon build --packages-select flex_msgs
source install/setup.bash
```

### I2C 设备未找到

**问题**: PCA9685 设备未检测到

**解决**:
1. 检查 I2C 连接和地址
2. 确认 I2C 总线编号（默认使用总线 7）
3. 运行 `sudo i2cdetect -y 7` 检测设备

### GPIO 中断不触发

**问题**: 行程开关中断未触发

**解决**:
1. 确认硬件上拉电阻已连接（下降沿触发需要上拉）
2. 检查引脚配置是否正确
3. 验证中断引脚初始状态应为高电平


### 代码规范

- 遵循 ROS2 C++ 和 Python 编码规范
- 添加适当的注释和文档
- 确保代码通过 lint 检查



**注意**: 本项目专为 Jetson 平台设计，在其他平台上可能需要修改 GPIO 库和相关配置。


