#include "flex_core/FlexCore.hpp"

/**
 * @brief 构造函数：初始化FlexCore控制节点
 * @param name 节点名称
 * @note 功能：
 *       1. 初始化MFAC无模型自适应控制器
 *       2. 初始化柔性机械臂参数结构体
 *       3. 创建遥控数据订阅者
 *       4. 创建电机控制服务客户端
 *       5. 启动系统监控线程
 */
FlexCore::FlexCore(std::string name): Node(name){

    MFAC_ptr = std::make_shared<MFAC>("MFAC");
    Flex_ptr = std::make_shared<FlexParam>();

    remote_sub = this->create_subscription<flex_msgs::msg::RemoteControl>
    ("remote_ctrl_data",1,std::bind(&FlexCore::RemoteCallback,this,std::placeholders::_1));

    motor_control_client = this->create_client<flex_msgs::srv::MotorControl>("motor_control_service");
    
    std::thread driver_pub_thread([this]{this->SystemMonitor();});
    driver_pub_thread.detach();

}


/**
 * @brief 遥控数据回调函数
 * @param remote_value 遥控器通道数据消息
 * @note 功能：解析遥控器10个通道的数据并存储到成员变量
 * @note 输入参数：
 *       - remote_value: 包含10个通道值的遥控消息
 * @note 输出参数：无（通过修改成员变量）
 * @note 通道映射：
 *       - channel_1: 通道0，用于X方向期望值
 *       - channel_2: 通道1，用于频率控制或方向控制
 *       - channel_3: 通道2，用于Y方向期望值
 *       - channel_4: 通道3，保留
 *       - channel_5: 通道4，电机选择位0（<=210为0，否则为1）
 *       - channel_6: 通道5，电机选择位1（<=210为0，否则为1）
 *       - channel_7: 通道6，复位模式开关（通过SwitchValue解析）
 *       - channel_8: 通道7，使能开关（<=210为0，否则为1）
 *       - channel_9: 通道8，控制模式开关（通过SwitchValue解析）
 *       - channel_10: 通道9，锁定块开关（<=210为0，否则为1）
 */
void FlexCore::RemoteCallback(const flex_msgs::msg::RemoteControl::SharedPtr remote_value){

    channel_1  = remote_value->channels_value[0];
    channel_2  = remote_value->channels_value[1];
    channel_3  = remote_value->channels_value[2];
    channel_4  = remote_value->channels_value[3];
    channel_7  = SwitchValue(remote_value->channels_value[6]);
    channel_9  = SwitchValue(remote_value->channels_value[8]);

    channel_5 = (remote_value->channels_value[4] <= 210) ? 0 : 1;
    channel_6 = (remote_value->channels_value[5] <= 210) ? 0 : 1;
    channel_8 = (remote_value->channels_value[7] <= 210) ? 0 : 1;
    channel_10 = (remote_value->channels_value[9] <= 210) ? 0 : 1;
}


/**
 * @brief 处理电机位置响应数据
 * @param response 电机控制服务的响应消息（包含4个电机的位置）
 * @note 功能：
 *       1. 将电机位置转换为柔性机械臂的线缆长度
 *       2. 计算关节参数（theta, kappa, lq）
 *       3. 计算机械臂末端工作空间坐标
 *       4. 更新当前输出向量y_k
 * @note 输入参数：
 *       - response: 包含motor_position[4]数组的服务响应
 * @note 输出参数：无（通过修改Flex_ptr和y_k成员变量）
 * @note 计算流程：
 *       1. 线缆长度 = 150.0（初始长度）+ 电机位置
 *       2. 计算关节参数（theta角度，kappa曲率）
 *       3. 计算工作空间坐标（x, y, z）
 *       4. 更新y_k = [x, y]（用于MFAC控制）
 */
void FlexCore::ProcessDriverPositionResponse(const flex_msgs::srv::MotorControl::Response::SharedPtr response){

    float lengths[4];

    for (int i = 0; i < 4; ++i) {
        lengths[i] = 150.0 + response->motor_position[i];
    }
    
    Flex_ptr->L1 = lengths[0];
    Flex_ptr->L2 = lengths[1];
    Flex_ptr->L3 = lengths[2];
    Flex_ptr->L4 = lengths[3];

    CalculationJoint(Flex_ptr);

    CalculationWorkspace(Flex_ptr);

    y_k(0) =  Flex_ptr->x;
    y_k(1) =  Flex_ptr->y;
}

/**
 * @brief 计算期望位置
 * @param x_e_ X方向期望值  
 * @param y_e_ Y方向期望值
 * @note 功能：将期望值转换为工作空间的期望坐标
 * @note 输入参数：
 *       - x_e_: X方向期望值
 *       - y_e_: Y方向期望值
 * @note 输出参数：无（通过修改成员变量y_e）
 * @note 转换公式：
 *       - x_e = (x_e_ - 1000) / 25.0 
 *       - y_e = (y_e_ - 1000) / 25.0 
 */
void FlexCore::CalculationExpect(int16_t x_e_,int16_t y_e_){
    y_e(0) = (x_e_ -1000) / 25.0;
    y_e(1) = (y_e_ -1000) / 25.0;

    RCLCPP_INFO(this->get_logger(), "ye_ 0 out : %.2f",y_e(0));
    RCLCPP_INFO(this->get_logger(), "ye_ 1 out : %.2f",y_e(1));

}

/**
 * @brief 计算柔性机械臂关节参数
 * @param Flex 柔性机械臂参数结构体指针（通过引用修改lq, theta, kappa）
 * @note 功能：根据4根线缆的长度计算机械臂的关节参数
 * @note 输入参数：
 *       - Flex: 包含L1, L2, L3, L4（4根线缆长度）和r（半径）
 * @note 输出参数：
 *       - 修改Flex->lq: 平均线缆长度（4根线缆的平均值）
 *       - 修改Flex->theta: 弯曲方向角度（弧度）
 *       - 修改Flex->kappa: 曲率（弯曲程度）
 * @note 计算公式：
 *       1. lq = (L1 + L2 + L3 + L4) / 4  （平均长度）
 *       2. theta = atan2(L2-L4, L1-L3)   （弯曲方向角）
 *       3. kappa = sqrt((L1-L3)² + (L2-L4)²) / (2*r)  （曲率）
 * @note 特殊情况处理：
 *       - 当L1≈L3时，如果L2≈L4则theta=0，否则theta=π/2
 */
void FlexCore::CalculationJoint(std::shared_ptr<FlexParam> Flex){
    Flex->lq = (Flex->L1 + Flex->L2 + Flex->L3 + Flex->L4) * 0.25f;

    if (abs(Flex->L3 - Flex->L1) < 1e-6) {
        Flex->theta = (abs(Flex->L4 - Flex->L2) < 1e-6) ? 0.0f : 3.1415926f * 0.5f;
    } 
    else {
        Flex->theta = atan2((Flex->L2 - Flex->L4), (Flex->L1 - Flex->L3));
    }
    
    float dx = Flex->L1 - Flex->L3;
    float dy = Flex->L2 - Flex->L4;
    Flex->kappa = sqrt(dx * dx + dy * dy) / (2.0f * Flex->r);
}


/**
 * @brief 计算柔性机械臂末端工作空间坐标
 * @param Flex 柔性机械臂参数结构体指针（通过引用修改x, y, z）
 * @note 功能：根据关节参数（lq, kappa, theta）计算机械臂末端在3D空间中的位置
 * @note 输入参数：
 *       - Flex: 包含lq（平均长度）、kappa（曲率）、theta（方向角）
 * @note 输出参数：
 *       - 修改Flex->x: 末端X坐标
 *       - 修改Flex->y: 末端Y坐标
 *       - 修改Flex->z: 末端Z坐标
 * @note 计算原理：
 *       当机械臂弯曲时（kappa > 0.01），使用圆弧模型：
 *       - factor = lq / kappa  （圆弧半径）
 *       - x = factor * (1 - cos(kappa)) * cos(theta)
 *       - y = factor * (1 - cos(kappa)) * sin(theta)
 *       - z = factor * sin(kappa)
 *       当机械臂伸直时（kappa < 0.01），末端在Z轴上：
 *       - x = 0, y = 0, z = lq
 */
void FlexCore::CalculationWorkspace(std::shared_ptr<FlexParam> Flex){
    if (abs(Flex->kappa) < 0.01) {
        Flex->x = 0.0f;
        Flex->y = 0.0f;
        Flex->z = Flex->lq;
    } else {
        float factor = Flex->lq / Flex->kappa;
        float cos_kappa = cos(Flex->kappa);
        Flex->x = factor * (1.0f - cos_kappa) * cos(Flex->theta);
        Flex->y = factor * (1.0f - cos_kappa) * sin(Flex->theta);
        Flex->z = factor * sin(Flex->kappa);
    }
    
    RCLCPP_INFO(get_logger(), "Workspace - X: %.2f, Y: %.2f, Z: %.2f", Flex->x, Flex->y, Flex->z);
}


/**
 * @brief 解析开关通道值
 * @param channel_value 遥控器开关通道的原始值（PWM值）
 * @return int 开关状态值：0=未定义, 1=位置1, 2=位置2, 3=位置3
 * @note 功能：将遥控器开关通道的PWM值转换为离散的开关状态
 * @note 输入参数：
 *       - channel_value: 开关通道的PWM值（范围约172-1811）
 * @note 输出参数：
 *       - 返回开关状态：0（未定义）、1（位置1）、2（位置2）、3（位置3）
 * @note 开关位置映射：
 *       - 位置1: channel_value ≈ 200 或 (100, 300)
 *       - 位置2: channel_value ≈ 1000 或 (900, 1100)
 *       - 位置3: channel_value ≈ 1800 或 (1700, 1900)
 *       - 未定义: 其他值
 */
int FlexCore::SwitchValue(int16_t channel_value){

    if(channel_value == 200 || (100 < channel_value && channel_value < 300)){
        return 1;
    }

    if(channel_value == 1000 || (900 < channel_value && channel_value < 1100)){
        return 2;
    }

    if(channel_value == 1800 || (1700 < channel_value && channel_value < 1900)){
        return 3;
    }
    return 0; 
}

/**
 * @brief 检查并计算电机频率
 * @param channel_value 遥控器通道值（PWM值，中位1000）
 * @return int 电机控制频率
 * @note 功能：将遥控器通道值转换为电机控制频率
 * @note 输入参数：
 *       - channel_value: 遥控器通道原始值（PWM值）
 * @note 输出参数：
 *       - 返回电机频率值
 * @note 转换规则：
 *       - 如果通道值在800-1200之间（接近中位），返回固定频率25
 *       - 否则返回通道值相对于中位1000的绝对值
 */
int FlexCore::CheckFrequency(int16_t channel_value){
    return (channel_value > 800 && channel_value < 1200) ? 25 : std::abs(channel_value - 1000);
}

/**
 * @brief 处理控制模式
 * @param mode 控制模式：1=自动控制, 2=整体控制, 3=单独电机控制
 * @param request 电机控制服务请求消息（通过引用修改）
 * @note 功能：根据控制模式选择相应的控制策略
 * @note 输入参数：
 *       - mode: 控制模式值（来自channel_9）
 *       - request: 电机控制请求消息
 * @note 输出参数：无（通过修改request并调用服务）
 * @note 控制模式说明：
 *       - mode=1: 自动控制模式（暂未实现）
 *       - mode=2: 整体控制模式，使用MFAC控制器进行整体控制
 *       - mode=3: 单独电机控制模式，控制单个电机
 * @note 前置条件：channel_8必须为1（使能开关打开）
 */
void FlexCore::ProcessControlMode(int mode, flex_msgs::srv::MotorControl::Request::SharedPtr request) {
    if (!channel_8) return; 
    
    switch (mode) {
        case 3: // 单独电机控制
            ProcessIndividualControl(request);
            break;
        case 2: // 整体控制
            ProcessGlobalControl(request);
            break;
        case 1: // 自动控制
            break;
        default:
            break;
    }
}

/**
 * @brief 处理单独电机控制模式
 * @param request 电机控制服务请求消息（通过引用修改并发送）
 * @note 功能：根据遥控器输入控制单个电机的运动
 * @note 输入参数：
 *       - request: 电机控制请求消息（需要填充）
 * @note 输出参数：无（通过服务调用获取位置反馈）
 * @note 控制逻辑：
 *       1. 根据channel_5和channel_6选择电机（0-3）
 *          motor_index = (channel_5 << 1) | channel_6
 *       2. 根据channel_2计算频率和方向
 *          - 频率 = CheckFrequency(channel_2)
 *          - 方向 = (channel_2 <= 1000) ? 1 : 0
 *       3. 计算运动距离 = 0.08 + 0.0018 * (freq - 50)
 *       4. 调用电机控制服务并等待响应
 *       5. 处理位置反馈数据
 */
void FlexCore::ProcessIndividualControl(flex_msgs::srv::MotorControl::Request::SharedPtr request) {
    int motor_index = (channel_5 << 1) | channel_6;
    if (motor_index >= 0 && motor_index < 4) {

        int freq = CheckFrequency(channel_2);
        request->motor_frequency = freq;
        request->motor_direction[motor_index] = (channel_2 <= 1000) ? 1 : 0;
        request->motor_distance[motor_index] = 0.08f + 0.0018f * (freq - 50);
        request->lock_block = channel_10;
        
        // 等待服务可用
        while (!motor_control_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "服务调用被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待 motor_control_service 服务可用...");
        }
        
        auto result = motor_control_client->async_send_request(request);
        // 等待服务完成，超时时间设置为30秒（足够处理复位等耗时操作）
        if (result.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            ProcessDriverPositionResponse(result.get());
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
        }
    }
}

/**
 * @brief 处理整体控制模式
 * @param request 电机控制服务请求消息（通过引用修改并发送）
 * @note 功能：使用MFAC控制器进行整体控制，实现位置跟踪
 * @note 输入参数：
 *       - request: 电机控制请求消息（需要填充）
 * @note 输出参数：无（通过服务调用获取位置反馈）
 * @note 控制流程：
 *       1. 计算期望位置（根据channel_1和channel_3）
 *       2. 调用MFAC控制器计算控制量
 *       3. 将控制量转换为4个电机的运动指令
 *       4. 调用电机控制服务并等待响应
 *       5. 处理位置反馈数据（更新y_k）
 * @note MFAC控制：
 *       - 输入：期望位置y_e和当前位置y_k
 *       - 输出：控制量MFAC_output [u_x, u_y]
 *       - 将控制量转换为电机对的控制指令
 */
void FlexCore::ProcessGlobalControl(flex_msgs::srv::MotorControl::Request::SharedPtr request) {
    CalculationExpect(channel_1, channel_3);
    Eigen::Vector2f MFAC_output = (*MFAC_ptr)(y_e, y_k, MFAC_ptr->getMFAC());
    
    CalculationStepOut(request, MFAC_output);
    request->lock_block = channel_10;
    // 整体控制必须使用运动模式（reset_mode = 2）
    request->reset_mode = 2;
    
    // 等待服务可用
    while (!motor_control_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "服务调用被中断");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待 motor_control_service 服务可用...");
    }
    
    auto result = motor_control_client->async_send_request(request);
    // 等待服务完成，超时时间设置为30秒（与单独控制保持一致）
    if (result.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
        ProcessDriverPositionResponse(result.get());
    } else {
        RCLCPP_ERROR(this->get_logger(), "服务调用超时");
    }
}

/**
 * @brief 系统监控线程主循环
 * @note 功能：持续监控系统状态并执行控制逻辑
 * @note 输入参数：无
 * @note 输出参数：无
 * @note 工作流程：
 *       1. 创建电机控制请求消息
 *       2. 初始化请求参数（频率、方向、距离等）
 *       3. 设置复位模式（来自channel_7）
 *       4. 根据控制模式（channel_9）执行相应控制
 *       5. 循环执行，直到ROS2节点关闭
 * @note 运行环境：独立线程中运行（detached thread）
 */
void FlexCore::SystemMonitor(){
    rclcpp::Clock clock;

    const std::chrono::milliseconds loop_delay(50);

    while (rclcpp::ok()){
        // 只有当使能开关打开时才发送请求
        if (channel_8) {
            auto request = std::make_shared<flex_msgs::srv::MotorControl::Request>();
            request->header.stamp = clock.now();

            request->motor_frequency = 0;
            for(int i = 0; i < 4; i++){
                request->motor_direction[i] = 0;
                request->motor_distance[i] = 0.0;
            }
            request->lock_block = 0;
            request->reset_mode = channel_7;

            // 发送请求并阻塞等待服务端处理完成
            ProcessControlMode(channel_9, request);
        }
        
        std::this_thread::sleep_for(loop_delay);
    }
}

/**
 * @brief 将MFAC控制输出转换为电机控制指令
 * @param request 电机控制服务请求消息（通过引用修改）
 * @param MFAC_output MFAC控制器输出的控制量向量 [u_x, u_y]
 * @note 功能：将2D控制量转换为4个电机的运动指令
 * @note 输入参数：
 *       - request: 电机控制请求消息（需要填充）
 *       - MFAC_output: MFAC控制量 [u_x, u_y]
 * @note 输出参数：无（通过修改request）

 * @note 控制逻辑：
 *       - 频率固定为800Hz
 *       - 如果控制量 > 0：电机0/1方向=0，电机2/3方向=1
 *       - 如果控制量 < 0：电机0/1方向=1，电机2/3方向=0
 *       - 运动距离 = |控制量|
 */
void FlexCore::CalculationStepOut(flex_msgs::srv::MotorControl::Request::SharedPtr request, const Eigen::Vector2f& MFAC_output){
    request->motor_frequency = 800;
    const std::array<std::pair<int, int>, 2> motor_pairs = {
        {{0, 2}, {1, 3}}
    };

    // 初始化所有电机的方向和距离
    for (int i = 0; i < 4; ++i) {
        request->motor_direction[i] = 0;
        request->motor_distance[i] = 0.0;
    }

    for (int i = 0; i < 2; ++i) {
        const auto& [idx1, idx2] = motor_pairs[i];
        const float ctrl_value = MFAC_output(i);

        if (ctrl_value > 0) {
            request->motor_direction[idx1] = 0;
            request->motor_direction[idx2] = 1;
        } else if (ctrl_value < 0) {
            request->motor_direction[idx1] = 1;
            request->motor_direction[idx2] = 0;
        }
        
        const float distance = std::abs(ctrl_value);
        request->motor_distance[idx1] = distance;
        request->motor_distance[idx2] = distance;
    }
}