#ifndef FLEXCORE_HPP
#define FLEXCORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "flex_msgs/msg/remote_control.hpp"
#include "flex_msgs/msg/driver_control.hpp"
#include "flex_msgs/msg/driver_callback.hpp"
#include "flex_msgs/srv/motor_control.hpp"
#include "flex_core/MFAC.hpp"
#include <functional>
#include <limits>
#include <memory>
#include <chrono>
#include <future>
#include <thread>
#include <atomic>

struct FlexParam{
   FlexParam(){
      x=0.0;
      y=0.0;
      z=0.0;

      lq=150.0;
      theta=0.0;
      kappa=0.0;

      L1=150.0;
      L2=150.0;
      L3=150.0;
      L4=150.0;

      r =28.284;

   }
   ~FlexParam() = default;

   float lq;
   float theta;
   float kappa;

   float x,y,z;

   float L1,L2,L3,L4;

   float r;
};



class FlexCore : public rclcpp::Node{
 public:

    FlexCore(std::string name);
    ~FlexCore() = default;

 private:
    
    void RemoteCallback(const flex_msgs::msg::RemoteControl::SharedPtr remote_value);
    void ProcessDriverPositionResponse(const flex_msgs::srv::MotorControl::Response::SharedPtr response);

    int  SwitchValue(int16_t channel_value);
    int  CheckFrequency(int16_t channel_value);

    void ProcessControlMode(int mode, flex_msgs::srv::MotorControl::Request::SharedPtr request);
    void ProcessIndividualControl(flex_msgs::srv::MotorControl::Request::SharedPtr request);
    void ProcessGlobalControl(flex_msgs::srv::MotorControl::Request::SharedPtr request);
    void ProcessResetMode(flex_msgs::srv::MotorControl::Request::SharedPtr request);

    void SystemMonitor();

    void CalculationJoint(std::shared_ptr<FlexParam> Flex);
    void CalculationWorkspace(std::shared_ptr<FlexParam> Flex);
    void CalculationExpect(int16_t x_e,int16_t y_e);
    void CalculationStepOut(flex_msgs::srv::MotorControl::Request::SharedPtr request, const Eigen::Vector2f& MFAC_output);


    int16_t  channel_1,channel_2,channel_3,channel_4,channel_7,channel_9,channel_10;

    bool channel_5,channel_6,channel_8;

    Eigen::Vector2f y_k,y_e;

    // 复位状态标志：防止在复位过程中重复发送请求
    std::atomic<bool> reset_in_progress{false};
    
    // 上一次的复位模式值：用于检测复位模式的变化，避免重复触发相同的复位操作
    // 初始化为运动模式(2)，表示系统初始状态为运动模式
    std::atomic<int16_t> last_reset_mode{2};


    rclcpp::Subscription<flex_msgs::msg::RemoteControl>::SharedPtr remote_sub;
    rclcpp::Client<flex_msgs::srv::MotorControl>::SharedPtr motor_control_client;

    std::shared_ptr<MFAC> MFAC_ptr;
    std::shared_ptr<FlexParam> Flex_ptr;

    
    
};

#endif