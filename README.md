# 柔性机械臂控制系统


基于 ROS2 的柔性机械臂控制系统，运行在 NVIDIA Jetson 平台上。系统包含：
1) 遥控器 SBUS 串口解析与 ROS2 发布（`remote_ctrl_data`）
2) 4 通道步进电机驱动与行程开关安全保护（`motor_control_service`）
3) MFAC 无模型自适应控制与工作空间坐标输出（`flex_position`）
4) 预设轨迹路径控制（`flex_path_core_node`，可选）

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
- **SBUS 遥控解析**: 从串口解析 SBUS 数据包，生成 10 通道 `channels_value` 并发布到 `remote_ctrl_data`
- **MFAC 无模型自适应控制**: 实现无模型自适应控制算法，用于柔性机械臂的运动控制
- **GPIO 控制**: 基于 Jetson.GPIO 的硬件 GPIO 控制，支持中断处理
- **行程开关检测**: 支持行程开关的硬件中断检测和安全保护
- **路径规划控制（可选）**: 通过遥控器通道选择圆/正方形/三角形/fig8 轨迹
- **ROS2 集成**: 完整的 ROS2 节点、话题、服务接口

## 🖥️ 系统要求

### 硬件要求
- **开发板**: NVIDIA Jetson 系列（Jetson Nano/Xavier/Orin 等）
- **PCA9685**: I2C PWM 驱动板（地址：0x40，I2C 总线：7）
- **遥控器串口适配器**: CH341（默认串口 `/dev/ttyCH341USB0`；若你的串口不同需要修改代码）
- **行程开关**: 4 个电机各自的中断引脚（下降沿触发；依赖硬件上拉电阻）

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
│   │   │       ├── FlexPathCore.hpp
│   │   │       ├── MFAC.hpp
│   │   │       └── RemoteControlParser.hpp
│   │   ├── src/               # C++ 源文件
│   │   │   ├── FlexCore.cpp
│   │   │   ├── FlexPathCore.cpp
│   │   │   ├── MFAC.cpp
│   │   │   └── RemoteControlParser.cpp
│   │   ├── node/              # ROS2 节点
│   │   │   ├── FlexCoreNode.cpp
│   │   │   ├── FlexPathCoreNode.cpp
│   │   │   └── RemoteCoreNode.cpp
│   │   ├── scripts/           # Python 脚本
│   │   │   └── driver_control.py
│   │   ├── test/              # 测试脚本
│   │   │   ├── test_limit_switch.py
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
    i2c-tools \
    libeigen3-dev \
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
sudo pip3 install smbus pyyaml numpy
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

### 启动控制系统（推荐）

**方法 1: 使用 launch 文件（推荐）**
```bash
source install/setup.bash
ros2 launch flex_core driver_control.launch.py
```

该 `launch` 会按顺序启动：
1) `remote_control_core_node`（解析遥控 SBUS，并发布 `remote_ctrl_data`）
2) `driver_control.py`（电机驱动服务端，提供 `motor_control_service`）
3) `flex_control_core_node`（MFAC 控制，发布 `flex_position`）

**方法 2: 仅启动电机驱动服务（用于联调）**
```bash
source install/setup.bash
ros2 run flex_core driver_control.py
```

**方法 3: 直接运行 Python 脚本（仅启动驱动服务）**
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
# 应该至少看到：/driver_control（启动了 launch 时还会看到 remote/flex 节点）

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

### RealSense D435：启动、录制与回放（含机械臂位置信息）

本节以 RealSense D435 + `realsense2_camera` 为例，开启**深度对齐到彩色图**的话题，并使用 `rosbag2` 同步录制：

- 相机彩色图与内参
- 对齐到彩色的深度图与内参（便于后续“彩色像素 \((u,v)\) → 深度”直接同像素读取）
- 机械臂位置信息（常见为 `/joint_states`；如果你的机械臂位姿通过 TF 发布，则需要录 `/tf`）

#### 1) 启动相机（开启对齐深度）

```bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

启动后可用如下命令确认关键话题已出现（示例）：

```bash
ros2 topic list
# /camera/camera/color/image_raw
# /camera/camera/color/camera_info
# /camera/camera/aligned_depth_to_color/image_raw
# /camera/camera/aligned_depth_to_color/camera_info
# /tf_static
```

#### 2) 录制（rosbag2）

推荐录制命令（同时录相机 + 机械臂位置 + TF）：

```bash
mkdir -p ~/bags

ros2 bag record -o ~/bags/d435_arm_$(date +%Y%m%d_%H%M%S) \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /tf /tf_static \
```

停止录制：在录制终端按 `Ctrl+C`。

查看录制信息：

```bash
ros2 bag info ~/bags/录制目录名
```

> 说明：
> - 如果你的系统没有发布 `/joint_states`，请先用 `ros2 topic list | grep joint` 查找实际话题名，并替换录制命令中的 `/joint_states`。

#### 3) 回放（rosbag2）

```bash
ros2 bag play ~/bags/录制目录名 --clock
```

回放验证（示例）：

- 图像话题可用 `rqt_image_view` 订阅 `/camera/camera/color/image_raw`
- 深度话题订阅 `/camera/camera/aligned_depth_to_color/image_raw`

> 提示：对齐深度图通常为 `16UC1`，数值常见单位为“毫米”（离线处理时如需米，可做 `depth_m = depth_raw / 1000.0`；以实际消息 `encoding` 和驱动配置为准）。

### 路径控制 FlexPathCore（可选）

如果希望柔性臂沿预设轨迹运动（圆/正方形/等边三角形/fig8），可启动 `flex_path_core_node`。

#### 1) 启动遥控器 SBUS 解析节点

确保已 source ROS2 环境后启动遥控器解析节点：

```bash
source install/setup.bash
ros2 run flex_core remote_control_core_node
```

启动后应能看到以下关键话题（示例）：

```bash
ros2 topic list
# /remote_ctrl_data
# /rosout
# （flex_control_core_node 启动后）/flex_position
# （flex_path_core_node 启动后）/flex_position
# （仅遥控节点时，其他话题可能不存在）
```

#### 2) 启动电机驱动服务端

启动并提供电机驱动服务端（提供 `/motor_control_service`）：

```bash
source install/setup.bash

ros2 run flex_core driver_control.py

# （本节只需启动驱动服务，不需要录制/回放 rosbag2）
```

启动后可在另一个终端验证：

确认服务类型：

```bash
ros2 service type /motor_control_service
```

#### 3) 启动路径控制节点

```bash
ros2 run flex_core flex_path_core_node
```

说明：
- 同一时间只建议运行一个控制节点：`flex_control_core_node`（MFAC整体控制）或 `flex_path_core_node`（路径控制），避免并发向 `motor_control_service` 发送互斥指令。
- 轨迹选择由遥控器 `channel_5/channel_6` 组合决定：`path_index = (channel_5<<1) | channel_6`。
- fig8 轨迹会从 `flex_core/params/fig8_workspace.csv` 加载轨迹点。

## 📖 使用说明

### 电机控制服务

通过 ROS2 服务调用控制电机：

```bash
# 调用电机控制服务
ros2 service call /motor_control_service flex_msgs/srv/MotorControl \
  "{reset_mode: 2, motor_frequency: 800.0, motor_direction: [true, true, true, true], motor_distance: [1.0, 1.0, 1.0, 1.0], lock_block: false}"
```

### 参数配置

- 电机参数：`src/flex_core/params/StepMotor_{1..4}_param.yaml`（`driver_control.py` / 测试脚本读取），字段包含 `StepMotor_dir`、`StepMotor_interrupt`、`StepMotor_Reset_Frequency`、`motor_direction_flag`、`Slide_block_compensation`
- MFAC 参数：`src/flex_core/params/MFAC_param.yaml`（`MFAC.cpp` 读取，当前使用相对路径读取；建议从工作空间根目录启动），字段包含 `lambda`、`rho`、`mu`、`eta`、`uk_limit`、`Kp`、`Ki`、`integral_limit`
- fig8 轨迹：`src/flex_core/params/fig8_workspace.csv`（`flex_path_core_node` 使用）

### 行程开关测试

运行行程开关测试脚本：

1. 测试四个电机各自行程开关（依次测试）：
```bash
cd ~/flex_shu_ws/src/flex_core/test
python3 test_limit_switch.py
```

2. 仅测试“40 引脚”行程开关（下降沿中断，5/10/15/20 秒递增序列）：
```bash
cd ~/flex_shu_ws/src/flex_core/test
python3 test_limit_switch_pin40.py
```

当触发行程开关时，电机将立即停止。

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
- `/flex_position` (geometry_msgs/msg/PointStamped): 柔性臂末端工作空间坐标（由 `flex_control_core_node` 发布）

### ROS2 服务

- `/motor_control_service` (flex_msgs/srv/MotorControl): 电机控制服务

### 消息/服务字段速查

- `flex_msgs/msg/RemoteControl`：`channels_value: uint16[10]`
- `flex_msgs/srv/MotorControl/Request`：`motor_frequency: uint16`、`motor_direction: bool[4]`、`motor_distance: float32[4]`、`lock_block: bool`、`reset_mode: uint16`
- `flex_msgs/srv/MotorControl/Response`：`motor_position: float32[4]`

### 主要类

- **FlexCore**: 核心控制类，实现 MFAC 控制和遥控器解析
- **FlexPathCore**: 基于路径规划的自动控制类（路径控制节点）
- **RemoteControlDataParser**: 遥控器 SBUS 解析与 `remote_ctrl_data` 发布
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

### 串口遥控器无法打开

**问题**：RemoteControl 解析节点启动后无法打开串口，或没有发布 `remote_ctrl_data`。

**解决**：
1. RemoteControl 解析节点默认读取 `/dev/ttyCH341USB0`（若你的串口不同，需要修改 `RemoteControlParser.cpp` 中的 `InitRemoteCtrlSerialport(...)` 参数）
2. 检查串口设备：`ls /dev/ttyUSB* /dev/ttyCH* 2>/dev/null`
3. 检查权限：必要时给串口设备所在组（如 `dialout`）授予权限，或先用 `sudo` 启动验证串口可读

### MFAC 参数文件找不到

**问题**：启动 `flex_control_core_node` 时提示找不到 `MFAC_param.yaml`。

**解决**：
1. 当前 `MFAC.cpp` 使用相对路径 `./src/flex_core/params/MFAC_param.yaml` 读取参数
2. 建议从工作空间根目录启动：例如 `cd ~/flex_shu_ws` 后再执行 `ros2 launch ...`
3. 若你希望从任意目录启动，需把 `MFAC.cpp` 参数读取逻辑改为使用安装路径（`ament_index_cpp`）


### 代码规范

- 遵循 ROS2 C++ 和 Python 编码规范
- 添加适当的注释和文档
- 确保代码通过 lint 检查



**注意**: 本项目专为 Jetson 平台设计，在其他平台上可能需要修改 GPIO 库和相关配置。


