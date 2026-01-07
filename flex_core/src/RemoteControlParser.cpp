#include "flex_core/RemoteControlParser.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/types.h>
#include <unistd.h>

/**
 * @brief 构造函数：初始化遥控数据解析器节点
 * @param name 节点名称
 * @note 创建ROS2节点，用于接收和解析遥控器串口数据
 */
RemoteControlDataParser::RemoteControlDataParser(std::string name):Node(name), has_new_data_(false){
    RCLCPP_INFO(this->get_logger(), "Start %s",name.c_str());
}

/**
 * @brief 析构函数：清理资源
 * @note 停止Qt线程和定时器，并等待其结束，确保资源正确释放
 */
RemoteControlDataParser::~RemoteControlDataParser() {
    if (publish_timer_ && publish_timer_->isActive()) {
        publish_timer_->stop();
    }
    if (qt_thread_ && qt_thread_->isRunning()) {
        qt_thread_->quit();
        qt_thread_->wait();
    }
}

/**
 * @brief 启动遥控数据解析器
 * @note 功能：
 *       1. 创建ROS2话题发布者，发布遥控控制数据
 *       2. 创建Qt线程用于串口通信
 *       3. 初始化串口并连接数据接收回调
 *       4. 创建定时器以50Hz频率发布数据
 * @note 输入参数：无
 * @note 输出参数：无
 */
void RemoteControlDataParser::start() {
    remote_ctrl_msg_pub = this->create_publisher<flex_msgs::msg::RemoteControl>(
    "remote_ctrl_data", rclcpp::QoS(30));
    
    // 创建并启动 Qt 线程
    qt_thread_ = std::make_unique<QThread>();
    this->moveToThread(qt_thread_.get());
    
    connect(qt_thread_.get(), &QThread::started, this, [this]() {
        // 在工作线程中初始化串口
        remote_control_serial_ = std::make_shared<QSerialPort>();
        if (InitRemoteCtrlSerialport("/dev/ttyCH341USB0")) {
            // 连接串口数据就绪信号
            connect(remote_control_serial_.get(), &QSerialPort::readyRead, 
                    this, &RemoteControlDataParser::ReadRemoteControlSerialDataCallback,
                    Qt::QueuedConnection); // 使用 QueuedConnection 确保跨线程安全
            
            // 创建并启动定时器，以50Hz频率发布数据（20ms间隔）
            publish_timer_ = std::make_unique<QTimer>();
            publish_timer_->setInterval(20); // 50Hz = 1000ms / 50 = 20ms
            publish_timer_->setSingleShot(false);
            connect(publish_timer_.get(), &QTimer::timeout, 
                    this, &RemoteControlDataParser::PublishLatestRemoteControlData,
                    Qt::QueuedConnection);
            publish_timer_->start();
            RCLCPP_INFO(this->get_logger(), "Remote control publish rate set to 50Hz (20ms interval)");
        }
    });

    qt_thread_->start();
}

/**
 * @brief 初始化遥控器串口
 * @param port_name 串口设备路径，例如 "/dev/ttyUSB0"
 * @return true: 串口打开成功, false: 串口打开失败
 * @note 功能：配置串口参数并打开串口
 *       - 波特率: 100000
 *       - 数据位: 8
 *       - 校验位: 偶校验
 *       - 停止位: 2
 *       - 流控制: 无
 */
bool RemoteControlDataParser::InitRemoteCtrlSerialport(const std::string port_name)const{
    remote_control_serial_->setPortName(QString::fromStdString(port_name));
    remote_control_serial_->setBaudRate(100000);
    remote_control_serial_->setDataBits(QSerialPort::Data8);
    remote_control_serial_->setParity(QSerialPort::EvenParity);
    remote_control_serial_->setStopBits(QSerialPort::TwoStop);
    remote_control_serial_->setFlowControl(QSerialPort::NoFlowControl);
    if (remote_control_serial_->open(QIODevice::ReadWrite)) {
        RCLCPP_INFO_STREAM(this->get_logger(),"Open Remote Receive Serial Success in: "<<remote_control_serial_->portName().toStdString());
        return true;
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(),"Open Remote Receive Serial Failed in: "<< port_name);
        return false;
    }
}

/**
 * @brief 串口数据接收回调函数
 * @note 功能：
 *       1. 读取串口接收缓冲区中的所有数据
 *       2. 解析SBUS格式的遥控数据
 *       3. 更新缓存的最新遥控数据（不直接发布）
 * @note 输入参数：无（通过Qt信号槽机制自动调用）
 * @note 输出参数：无（数据存储在缓存中，由定时器统一发布）
 */
void RemoteControlDataParser::ReadRemoteControlSerialDataCallback(){
    QByteArray received_buffer = remote_control_serial_->readAll();
    if (!received_buffer.isEmpty())
    {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        rclcpp::Clock clock;
        latest_remote_ctl_msg_.header.stamp = clock.now();
        Parser(received_buffer, latest_remote_ctl_msg_);
        has_new_data_ = true;
    }
}

/**
 * @brief 定时发布回调函数
 * @note 功能：
 *       1. 以50Hz频率定时调用（每20ms）
 *       2. 发布缓存中的最新遥控数据
 *       3. 确保发布频率稳定在50Hz
 * @note 输入参数：无（通过Qt定时器自动调用）
 * @note 输出参数：无（通过ROS2话题发布数据）
 */
void RemoteControlDataParser::PublishLatestRemoteControlData() {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (has_new_data_) {
        remote_ctrl_msg_pub->publish(latest_remote_ctl_msg_);
        has_new_data_ = false;
    }
}


/**
 * @brief 解析SBUS格式的遥控数据
 * @param sbus_buf 输入的SBUS数据缓冲区（25字节，包含起始字节0x0F和结束字节0x00）
 * @param rc_ctrl 输出的遥控控制消息结构体（通过引用返回）
 * @note 功能：将SBUS协议的25字节数据包解析为10个通道的PWM值（每个通道11位，范围0-2047）
 * @note 输入参数：
 *       - sbus_buf: SBUS数据包，格式为 [0x0F, ch1_low, ch1_high, ch2_low, ..., ch10_high, flags, 0x00]
 * @note 输出参数：
 *       - rc_ctrl: 解析后的10个通道值，存储在channels_value数组中
 * @note SBUS协议说明：
 *       - 每个通道占用11位，数据在字节流中交错存储
 *       - 通道值范围：172-1811（对应PWM 1000-2000us）
 */
void RemoteControlDataParser::Parser(const QByteArray &sbus_buf, flex_msgs::msg::RemoteControl &rc_ctrl){
    auto sbus_buf_temp = reinterpret_cast<unsigned char *>(const_cast<char *>(sbus_buf.data()));
    rc_ctrl.channels_value[0] = static_cast<int16_t>((sbus_buf_temp[1] | (sbus_buf_temp[2] << 8)) & 0x07ff);
    rc_ctrl.channels_value[1] = static_cast<int16_t>(((sbus_buf_temp[2] >> 3) | (sbus_buf_temp[3]  << 5)) & 0x07ff);            
    rc_ctrl.channels_value[2] = static_cast<int16_t>(((sbus_buf_temp[3] >> 6) | (sbus_buf_temp[4]  << 2) | (sbus_buf_temp[5] << 10)) & 0x07ff);            
    rc_ctrl.channels_value[3] = static_cast<int16_t>(((sbus_buf_temp[5] >> 1) | (sbus_buf_temp[6]  << 7)) & 0x07ff);            
    rc_ctrl.channels_value[4] = static_cast<int16_t>(((sbus_buf_temp[6] >> 4) | (sbus_buf_temp[7]  << 4)) & 0x07ff);            
    rc_ctrl.channels_value[5] = static_cast<int16_t>(((sbus_buf_temp[7] >> 7) | (sbus_buf_temp[8]  << 1) | (sbus_buf_temp[9] << 9)) & 0x07ff);            
    rc_ctrl.channels_value[6] = static_cast<int16_t>(((sbus_buf_temp[9] >> 2) | (sbus_buf_temp[10] << 6)) & 0x07ff);          
    rc_ctrl.channels_value[7] = static_cast<int16_t>(((sbus_buf_temp[10]>> 5) | (sbus_buf_temp[11] << 3)) & 0x07ff);  
    rc_ctrl.channels_value[8] = static_cast<int16_t>(((sbus_buf_temp[12]<< 0) | (sbus_buf_temp[13] << 8)) & 0x07ff);  
    rc_ctrl.channels_value[9] = static_cast<int16_t>(((sbus_buf_temp[13]>> 3) | (sbus_buf_temp[14] << 5)) & 0x07ff);   
}