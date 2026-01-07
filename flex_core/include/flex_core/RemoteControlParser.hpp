#ifndef  REMOTECONTROLPARSER_HPP
#define REMOTECONTROLPARSER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include <QApplication>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <memory>
#include <thread>
#include <QThread>
#include <QTimer>
#include <mutex>


#include "flex_msgs/msg/remote_control.hpp" 

/**
 * @class RemoteControlDataParser
 * @brief 遥控数据解析器类
 * @note 功能：接收并解析SBUS格式的遥控器串口数据，发布ROS2话题消息
 * @note 继承关系：
 *       - 继承自QObject：用于Qt信号槽机制
 *       - 继承自rclcpp::Node：ROS2节点功能
 * @note 工作流程：
 *       1. 通过串口接收SBUS数据包（25字节）
 *       2. 解析SBUS数据为10个通道的PWM值
 *       3. 发布到ROS2话题 "remote_ctrl_data"
 */
class RemoteControlDataParser : public QObject, public rclcpp::Node {
 Q_OBJECT
 public:
    /**
     * @brief 构造函数：初始化遥控数据解析器节点
     * @param name 节点名称
     */
    RemoteControlDataParser(std::string name);
    
    /**
     * @brief 析构函数：清理资源
     */
    ~RemoteControlDataParser();
    
    /**
     * @brief 启动遥控数据解析器
     * @note 创建发布者、初始化串口、启动Qt线程
     */
    void start();
    
 public slots:
    /**
     * @brief 串口数据接收回调函数（Qt槽函数）
     * @note 当串口有数据到达时自动调用，解析数据并更新缓存
     */
    void ReadRemoteControlSerialDataCallback();
    
    /**
     * @brief 定时发布回调函数（Qt槽函数）
     * @note 以50Hz频率定时发布缓存的最新遥控数据
     */
    void PublishLatestRemoteControlData();
    
 private:
    /**
     * @brief 解析SBUS格式的遥控数据
     * @param sbus_buf 输入的SBUS数据缓冲区（25字节）
     * @param rc_ctrl 输出的遥控控制消息结构体（通过引用返回）
     * @note 将SBUS协议的25字节数据包解析为10个通道的PWM值
     */
    void Parser(const QByteArray &sbus_buf,flex_msgs::msg::RemoteControl &rc_ctrl);
    
    /**
     * @brief 初始化遥控器串口
     * @param port_name 串口设备路径，例如 "/dev/ttyUSB0"
     * @return true: 串口打开成功, false: 串口打开失败
     * @note 配置串口参数：波特率100000，8数据位，偶校验，2停止位
     */
    bool InitRemoteCtrlSerialport(const std::string port_name)const;
    
    /// @brief 串口通信对象指针
    std::shared_ptr<QSerialPort> remote_control_serial_;
    
    /// @brief ROS2话题发布者，发布遥控控制数据到 "remote_ctrl_data"
    rclcpp::Publisher<flex_msgs::msg::RemoteControl>::SharedPtr remote_ctrl_msg_pub;

    /// @brief Qt线程指针，用于串口通信的独立线程
    std::unique_ptr<QThread> qt_thread_;
    
    /// @brief 定时器，用于以50Hz频率发布遥控数据
    std::unique_ptr<QTimer> publish_timer_;
    
    /// @brief 最新解析的遥控数据缓存
    flex_msgs::msg::RemoteControl latest_remote_ctl_msg_;
    
    /// @brief 用于保护数据缓存的互斥锁
    std::mutex msg_mutex_;
    
    /// @brief 是否有新数据可发布的标志
    bool has_new_data_;
};

#endif