#ifndef FLEXPATHCORE_HPP
#define FLEXPATHCORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "flex_msgs/msg/remote_control.hpp"
#include "flex_msgs/srv/motor_control.hpp"
#include "flex_core/MFAC.hpp"
#include "flex_core/FlexCore.hpp"  // 复用 FlexParam 和 MFAC 定义

#include <Eigen/Dense>
#include <chrono>
#include <future>
#include <memory>
#include <atomic>
#include <vector>

/**
 * @brief 基于路径规划的柔性臂自动控制节点
 * @note 通过电机选择位0/1（channel_5, channel_6）选择4条不同的预设路径，
 *       使用 MFAC 控制器和底层 motor_control_service 实现柔性臂沿路径运动。
 *
 * 路径选择 path_index = (channel_5 << 1) | channel_6 ∈ {0,1,2,3}：
 *  - 0 (00): 圆轨迹（半径30）
 *  - 1 (01): 正方形周界连续轨迹（边长50，与圆相同按相位匀速跑两圈后回原点）
 *  - 2 (10): 等边三角形（外接圆半径30）
 *  - 3 (11): fig8 工作空间轨迹（需加载 CSV）；若未加载则回原点
 */
class FlexPathCore : public rclcpp::Node {
public:
    explicit FlexPathCore(const std::string &name = "flex_path_core");
    ~FlexPathCore() = default;

private:
    // ROS 回调与主循环
    void RemoteCallback(const flex_msgs::msg::RemoteControl::SharedPtr msg);
    void SystemMonitor();

    // 路径生成与控制
    float GetPathOmega(int path_index) const;
    float ComputeTimeStep();
    void ExecuteOneControlStep(float x_d, float y_d);
    void GeneratePathTarget(int path_index, float dt, float &x_d, float &y_d);
    void LoadFig8Workspace();
    bool LoadFig8Waypoints(const std::string &csv_path);
    void ProcessDriverPositionResponse(const flex_msgs::srv::MotorControl::Response::SharedPtr response);

    // 状态机处理
    void HandleIdleState(const std::chrono::steady_clock::time_point &now, int path_index, bool enable_rising_edge);
    void HandleWait5sState(const std::chrono::steady_clock::time_point &now);
    void HandleRunOneLoopState(const std::chrono::steady_clock::time_point &now);
    void HandleReturnToOriginState(const std::chrono::steady_clock::time_point &now);
    void CalculationStepOut(
        flex_msgs::srv::MotorControl::Request::SharedPtr request,
        const Eigen::Vector2f &MFAC_output
    );

    // 遥控通道状态
    int16_t channel_1_{0};
    int16_t channel_2_{0};
    int16_t channel_3_{0};
    int16_t channel_4_{0};
    bool channel_5_{false};  // 电机选择位0
    bool channel_6_{false};  // 电机选择位1
    bool channel_8_{false};  // 使能
    bool channel_10_{false}; // 锁定块

    // 路径状态
    int current_path_index_{-1};
    float path_phase_{0.0f};
    std::chrono::steady_clock::time_point last_path_update_time_{std::chrono::steady_clock::now()};

    // 命令触发后的一次性执行状态机：等待->跑两圈->回原点->停止等待下一次命令更新
    enum class ExecState { IDLE, WAIT_5S, RUN_ONE_LOOP, RETURN_TO_ORIGIN };
    ExecState exec_state_{ExecState::IDLE};

    // 用于检测“命令更新”：下一次触发需要 path_index 变化，或使能开关产生上升沿
    bool channel_8_prev_{false};
    int last_trigger_path_index_{-1};
    int return_settle_count_{0};
    float motion_elapsed_{0.0f};
    std::chrono::steady_clock::time_point state_start_time_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point return_start_time_{std::chrono::steady_clock::now()};

    // 柔性臂与控制器
    std::shared_ptr<MFAC> MFAC_ptr_;
    std::shared_ptr<FlexParam> Flex_ptr_;
    Eigen::Vector2f y_k_{0.0f, 0.0f};
    Eigen::Vector2f y_e_{0.0f, 0.0f};

    // fig8 工作空间轨迹（path_index=3 使用），按时间相位沿路径推进
    std::vector<Eigen::Vector2f> fig8_waypoints_;
    float fig8_phase_{0.0f};

    // ROS 通信
    rclcpp::Subscription<flex_msgs::msg::RemoteControl>::SharedPtr remote_sub_;
    rclcpp::Client<flex_msgs::srv::MotorControl>::SharedPtr motor_control_client_;
};

#endif

