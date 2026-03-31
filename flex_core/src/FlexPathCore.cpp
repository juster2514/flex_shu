#include "flex_core/FlexPathCore.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {
constexpr float TWO_PI = 6.283185307179586f;
constexpr float RETURN_ERROR_THRESHOLD = 1.0f;
constexpr int RETURN_SETTLE_REQUIRED = 10;
constexpr std::chrono::milliseconds LOOP_DELAY(30);
constexpr std::chrono::milliseconds WAIT_5S(5000);
constexpr std::chrono::milliseconds RETURN_TIMEOUT(5000);
constexpr std::chrono::milliseconds SERVICE_TIMEOUT(30000);
constexpr float DT_MIN = 0.01f;
constexpr float DT_MAX = 0.1f;

}  // namespace

FlexPathCore::FlexPathCore(const std::string &name)
    : Node(name) {
    MFAC_ptr_ = std::make_shared<MFAC>("MFAC_Path");
    Flex_ptr_ = std::make_shared<FlexParam>();

    remote_sub_ = this->create_subscription<flex_msgs::msg::RemoteControl>(
        "remote_ctrl_data",
        1,
        std::bind(&FlexPathCore::RemoteCallback, this, std::placeholders::_1));

    motor_control_client_ = this->create_client<flex_msgs::srv::MotorControl>("motor_control_service");

    std::thread monitor_thread([this]() { this->SystemMonitor(); });
    monitor_thread.detach();

    LoadFig8Workspace();

    RCLCPP_INFO(this->get_logger(), "FlexPathCore 节点已初始化");
}

void FlexPathCore::RemoteCallback(const flex_msgs::msg::RemoteControl::SharedPtr msg) {
    channel_1_ = msg->channels_value[0];
    channel_2_ = msg->channels_value[1];
    channel_3_ = msg->channels_value[2];
    channel_4_ = msg->channels_value[3];

    // 电机选择位：<=210 为 0，否则为 1
    channel_5_ = (msg->channels_value[4] > 210);
    channel_6_ = (msg->channels_value[5] > 210);
    channel_8_ = (msg->channels_value[7] > 210);   // 使能
    channel_10_ = (msg->channels_value[9] > 210);  // 锁定块
}

float FlexPathCore::GetPathOmega(int path_index) const {
    switch (path_index) {
    case 0:
        return 0.15f;  // 圆
    case 1:
        return 0.4f;   // 正方形
    case 2:
        return 0.3f;   // 等边三角形
    case 3:
        return 0.4f;   // fig8 前在原位保持
    default:
        return 0.15f;
    }
}

float FlexPathCore::ComputeTimeStep() {
    const auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - last_path_update_time_)
                   .count() /
               1000.0f;
    last_path_update_time_ = now;
    if (dt <= 0.0f) dt = DT_MIN;
    if (dt > DT_MAX) dt = DT_MAX;
    return dt;
}

void FlexPathCore::ExecuteOneControlStep(float x_d, float y_d) {
    rclcpp::Clock clock;
    y_e_(0) = x_d;
    y_e_(1) = y_d;

    auto request = std::make_shared<flex_msgs::srv::MotorControl::Request>();
    request->header.stamp = clock.now();
    request->reset_mode = 2;
    request->motor_frequency = 0;
    for (int i = 0; i < 4; ++i) {
        request->motor_direction[i] = 0;
        request->motor_distance[i] = 0.0f;
    }
    request->lock_block = channel_10_;

    Eigen::Vector2f MFAC_output = (*MFAC_ptr_)(y_e_, y_k_, MFAC_ptr_->getMFAC());
    CalculationStepOut(request, MFAC_output);

    while (!motor_control_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "motor_control_service 被中断");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待 motor_control_service 服务可用...");
    }

    auto result = motor_control_client_->async_send_request(request);
    if (result.wait_for(SERVICE_TIMEOUT) == std::future_status::ready) {
        ProcessDriverPositionResponse(result.get());
    } else {
        RCLCPP_ERROR(this->get_logger(), "FlexPathCore 控制服务调用超时");
    }
}

void FlexPathCore::SystemMonitor() {
    while (rclcpp::ok()) {
        const auto now = std::chrono::steady_clock::now();

        if (!channel_8_) {
            exec_state_ = ExecState::IDLE;
            channel_8_prev_ = false;
            std::this_thread::sleep_for(LOOP_DELAY);
            continue;
        }

        const int path_index = (static_cast<int>(channel_5_) << 1) | static_cast<int>(channel_6_);
        const bool enable_rising_edge = (!channel_8_prev_ && channel_8_);
        channel_8_prev_ = channel_8_;

        switch (exec_state_) {
        case ExecState::IDLE:
            HandleIdleState(now, path_index, enable_rising_edge);
            break;
        case ExecState::WAIT_5S:
            HandleWait5sState(now);
            break;
        case ExecState::RUN_ONE_LOOP:
            HandleRunOneLoopState(now);
            break;
        case ExecState::RETURN_TO_ORIGIN:
            HandleReturnToOriginState(now);
            break;
        }

        std::this_thread::sleep_for(LOOP_DELAY);
    }
}

void FlexPathCore::HandleIdleState(const std::chrono::steady_clock::time_point &now,
                                  int path_index,
                                  bool enable_rising_edge) {
    if (enable_rising_edge || (path_index != last_trigger_path_index_)) {
        last_trigger_path_index_ = path_index;
        current_path_index_ = path_index;
        path_phase_ = 0.0f;
        motion_elapsed_ = 0.0f;
        return_settle_count_ = 0;

        if (path_index == 3 && !fig8_waypoints_.empty()) {
            exec_state_ = ExecState::RETURN_TO_ORIGIN;
            return_start_time_ = now;
            fig8_phase_ = 0.0f;
            last_path_update_time_ = now;
            RCLCPP_INFO(this->get_logger(), "FlexPathCore 8字轨迹触发，直接运行两遍");
        } else {
            exec_state_ = ExecState::WAIT_5S;
            state_start_time_ = now;
            RCLCPP_INFO(this->get_logger(),
                        "FlexPathCore 命令触发：等待5s后运行 path_index=%d",
                        current_path_index_);
        }
    }
}

void FlexPathCore::HandleWait5sState(const std::chrono::steady_clock::time_point &now) {
    if (now - state_start_time_ >= WAIT_5S) {
        exec_state_ = ExecState::RUN_ONE_LOOP;
        path_phase_ = 0.0f;
        motion_elapsed_ = 0.0f;
        last_path_update_time_ = now;

        RCLCPP_INFO(this->get_logger(),
                    "FlexPathCore 开始运行两圈：path_index=%d",
                    current_path_index_);
    }
}

void FlexPathCore::HandleRunOneLoopState(const std::chrono::steady_clock::time_point &now) {
    float dt = ComputeTimeStep();

    motion_elapsed_ += dt;

    float x_d = 0.0f;
    float y_d = 0.0f;
    GeneratePathTarget(current_path_index_, dt, x_d, y_d);

    ExecuteOneControlStep(x_d, y_d);

    const float omega = GetPathOmega(current_path_index_);
    const float loop_total_time_s = 2.0f * TWO_PI / omega;

    if (motion_elapsed_ >= loop_total_time_s) {
        exec_state_ = ExecState::RETURN_TO_ORIGIN;
        return_start_time_ = now;
        return_settle_count_ = 0;
        fig8_phase_ = 0.0f;
        last_path_update_time_ = now;

        if (current_path_index_ == 3 && !fig8_waypoints_.empty()) {
            RCLCPP_INFO(this->get_logger(), "FlexPathCore 两圈结束，开始按 fig8 轨迹运动");
        } else {
            RCLCPP_INFO(this->get_logger(), "FlexPathCore 两圈结束，开始回到 (0,0)");
        }
    }
}

void FlexPathCore::HandleReturnToOriginState(const std::chrono::steady_clock::time_point &now) {
    float dt = ComputeTimeStep();

    float x_d = 0.0f;
    float y_d = 0.0f;

    if (current_path_index_ == 3 && !fig8_waypoints_.empty()) {
        const float omega = GetPathOmega(3);
        fig8_phase_ += omega * dt;

        const size_t n = fig8_waypoints_.size();
        if (n >= 2) {
            const float progress = std::fmod(fig8_phase_, TWO_PI) / TWO_PI;
            const float seg = progress * static_cast<float>(n - 1);
            const size_t idx = static_cast<size_t>(seg);
            const float t = seg - static_cast<float>(idx);

            if (idx < n - 1) {
                x_d = (1.0f - t) * fig8_waypoints_[idx](0) + t * fig8_waypoints_[idx + 1](0);
                y_d = (1.0f - t) * fig8_waypoints_[idx](1) + t * fig8_waypoints_[idx + 1](1);
            } else {
                x_d = fig8_waypoints_.back()(0);
                y_d = fig8_waypoints_.back()(1);
            }
        } else if (n == 1) {
            x_d = fig8_waypoints_[0](0);
            y_d = fig8_waypoints_[0](1);
        }
    } else {
        x_d = 0.0f;
        y_d = 0.0f;
    }

    ExecuteOneControlStep(x_d, y_d);

    bool motion_complete = false;

    if (current_path_index_ == 3 && !fig8_waypoints_.empty()) {
        motion_complete = (fig8_phase_ >= 2.0f * TWO_PI);
    } else {
        const float err = std::sqrt(y_k_(0) * y_k_(0) + y_k_(1) * y_k_(1));
        if (err < RETURN_ERROR_THRESHOLD) {
            return_settle_count_++;
        } else {
            return_settle_count_ = 0;
        }
        const auto elapsed_return = now - return_start_time_;
        if (return_settle_count_ >= RETURN_SETTLE_REQUIRED ||
            elapsed_return >= RETURN_TIMEOUT) {
            motion_complete = true;
        }
    }

    if (motion_complete) {
        exec_state_ = ExecState::IDLE;
        path_phase_ = 0.0f;
        if (current_path_index_ == 3 && !fig8_waypoints_.empty()) {
            RCLCPP_INFO(this->get_logger(), "FlexPathCore fig8 轨迹运动完成，等待下一条命令");
        } else {
            RCLCPP_INFO(this->get_logger(), "FlexPathCore 回原点完成，等待下一条命令");
        }
    }
}

void FlexPathCore::GeneratePathTarget(int path_index, float dt, float &x_d, float &y_d) {
    if (path_index == 3) {
        x_d = 0.0f;
        y_d = 0.0f;
        return;
    }

    const float omega = GetPathOmega(path_index);
    path_phase_ += omega * dt;

    while (path_phase_ >= TWO_PI) path_phase_ -= TWO_PI;
    while (path_phase_ < 0.0f) path_phase_ += TWO_PI;

    const float A = 30.0f;

    switch (path_index) {
    case 0: {
        x_d = A * std::cos(path_phase_);
        y_d = A * std::sin(path_phase_);
        break;
    }
    case 1: {
        constexpr float half = 25.0f;              // mm
        constexpr float side = 2.0f * half;        // 50mm
        constexpr float perimeter = 4.0f * side;   // mm
        constexpr int N = 24;                      // 4 个角点 + 20 个均匀点

        auto point_on_square = [](float s, float &x, float &y) {
            constexpr float half_local = 25.0f;
            constexpr float side_local = 2.0f * half_local;
            constexpr float per_local = 4.0f * side_local;
            while (s >= per_local) s -= per_local;
            while (s < 0.0f) s += per_local;

            // 角点顺序：(25,25)->(-25,25)->(-25,-25)->(25,-25)->(25,25)
            if (s < side_local) {                  // (25,25) -> (-25,25)
                x = half_local - s;
                y = half_local;
            } else if (s < 2.0f * side_local) {    // (-25,25) -> (-25,-25)
                const float u = s - side_local;
                x = -half_local;
                y = half_local - u;
            } else if (s < 3.0f * side_local) {    // (-25,-25) -> (25,-25)
                const float u = s - 2.0f * side_local;
                x = -half_local + u;
                y = -half_local;
            } else {                                // (25,-25) -> (25,25)
                const float u = s - 3.0f * side_local;
                x = half_local;
                y = -half_local + u;
            }
        };

        const float progress = path_phase_ / TWO_PI;          // [0,1)
        const float seg = progress * static_cast<float>(N);   // [0,N)
        const int idx = static_cast<int>(seg) % N;
        const float t = seg - static_cast<float>(idx);        // [0,1)

        const float s0 = (static_cast<float>(idx) / static_cast<float>(N)) * perimeter;
        const float s1 = (static_cast<float>((idx + 1) % N) / static_cast<float>(N)) * perimeter;

        float x0, y0, x1, y1;
        point_on_square(s0, x0, y0);
        point_on_square(s1, x1, y1);

        // 在 4 个角点基础上均匀加 20 个点（总计 24 点），并在相邻点间插值保证连续
        x_d = (1.0f - t) * x0 + t * x1;
        y_d = (1.0f - t) * y0 + t * y1;
        break;
    }
    case 2: {
        constexpr float R = 30.0f;
        constexpr float sqrt3 = 1.7320508075688772f;
        const float a = R * sqrt3;
        const float x0 = 0.0f;
        const float y0 = R;
        const float x1 = -R * sqrt3 * 0.5f;
        const float y1 = -R * 0.5f;
        const float x2 = R * sqrt3 * 0.5f;
        const float y2 = -R * 0.5f;
        const float perimeter = 3.0f * a;
        const float s = (path_phase_ / TWO_PI) * perimeter;

        if (s < a) {
            const float t = s / a;
            x_d = x0 + t * (x1 - x0);
            y_d = y0 + t * (y1 - y0);
        } else if (s < 2.0f * a) {
            const float t = (s - a) / a;
            x_d = x1 + t * (x2 - x1);
            y_d = y1 + t * (y2 - y1);
        } else {
            const float t = (s - 2.0f * a) / a;
            x_d = x2 + t * (x0 - x2);
            y_d = y2 + t * (y0 - y2);
        }
        break;
    }
    default:
        x_d = 0.0f;
        y_d = 0.0f;
        break;
    }
}

void FlexPathCore::ProcessDriverPositionResponse(
    const flex_msgs::srv::MotorControl::Response::SharedPtr response) {
    float lengths[4];
    for (int i = 0; i < 4; ++i) {
        lengths[i] = 150.0f + response->motor_position[i];
    }

    Flex_ptr_->L1 = lengths[0];
    Flex_ptr_->L2 = lengths[1];
    Flex_ptr_->L3 = lengths[2];
    Flex_ptr_->L4 = lengths[3];
    Flex_ptr_->lq = (Flex_ptr_->L1 + Flex_ptr_->L2 + Flex_ptr_->L3 + Flex_ptr_->L4) * 0.25f;

    if (std::abs(Flex_ptr_->L3 - Flex_ptr_->L1) < 1e-6f) {
        Flex_ptr_->theta = (std::abs(Flex_ptr_->L4 - Flex_ptr_->L2) < 1e-6f)
                               ? 0.0f
                               : 3.1415926f * 0.5f;
    } else {
        Flex_ptr_->theta = std::atan2((Flex_ptr_->L2 - Flex_ptr_->L4),
                                      (Flex_ptr_->L1 - Flex_ptr_->L3));
    }

    float dx = Flex_ptr_->L1 - Flex_ptr_->L3;
    float dy = Flex_ptr_->L2 - Flex_ptr_->L4;
    Flex_ptr_->kappa = std::sqrt(dx * dx + dy * dy) / (2.0f * Flex_ptr_->r);

    if (std::abs(Flex_ptr_->kappa) < 0.01f) {
        Flex_ptr_->x = 0.0f;
        Flex_ptr_->y = 0.0f;
        Flex_ptr_->z = Flex_ptr_->lq;
    } else {
        float factor = Flex_ptr_->lq / Flex_ptr_->kappa;
        float cos_kappa = std::cos(Flex_ptr_->kappa);
        Flex_ptr_->x = factor * (1.0f - cos_kappa) * std::cos(Flex_ptr_->theta);
        Flex_ptr_->y = factor * (1.0f - cos_kappa) * std::sin(Flex_ptr_->theta);
        Flex_ptr_->z = factor * std::sin(Flex_ptr_->kappa);
    }

    y_k_(0) = Flex_ptr_->x;
    y_k_(1) = Flex_ptr_->y;
}

void FlexPathCore::CalculationStepOut(
    flex_msgs::srv::MotorControl::Request::SharedPtr request,
    const Eigen::Vector2f &MFAC_output) {
    request->motor_frequency = 800;

    const std::array<std::pair<int, int>, 2> motor_pairs = {{{0, 2}, {1, 3}}};

    for (int i = 0; i < 4; ++i) {
        request->motor_direction[i] = 0;
        request->motor_distance[i] = 0.0f;
    }

    for (int i = 0; i < 2; ++i) {
        const auto &[idx1, idx2] = motor_pairs[i];
        float ctrl_value = MFAC_output(i);

        if (ctrl_value > 0.0f) {
            request->motor_direction[idx1] = 0;
            request->motor_direction[idx2] = 1;
        } else if (ctrl_value < 0.0f) {
            request->motor_direction[idx1] = 1;
            request->motor_direction[idx2] = 0;
        }

        float distance = std::abs(ctrl_value);
        request->motor_distance[idx1] = distance;
        request->motor_distance[idx2] = distance;
    }
}

void FlexPathCore::LoadFig8Workspace() {
    try {
        const std::string share_dir = ament_index_cpp::get_package_share_directory("flex_core");
        const std::string csv_path = share_dir + "/params/fig8_workspace.csv";
        if (LoadFig8Waypoints(csv_path)) {
            RCLCPP_INFO(this->get_logger(), "FlexPathCore 已加载 fig8 轨迹：%zu 个路径点",
                        fig8_waypoints_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "FlexPathCore 加载 fig8_workspace.csv 失败，path_index=3 将回退到回原点");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "FlexPathCore 无法获取包路径: %s", e.what());
    }
}

bool FlexPathCore::LoadFig8Waypoints(const std::string &csv_path) {
    fig8_waypoints_.clear();
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开 fig8 轨迹文件: %s", csv_path.c_str());
        return false;
    }
    std::string line;
    bool first = true;
    while (std::getline(file, line)) {
        if (first) {
            first = false;
            continue;
        }
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string x_str, y_str, z_str;
        if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',') &&
            std::getline(iss, z_str, ',')) {
            try {
                float x = std::stof(x_str);
                float y = std::stof(y_str);
                fig8_waypoints_.emplace_back(x, y);
            } catch (const std::exception &) {
                continue;
            }
        }
    }
    return !fig8_waypoints_.empty();
}
