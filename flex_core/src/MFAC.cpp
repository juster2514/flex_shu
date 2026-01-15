#include "flex_core/MFAC.hpp"
#include <limits>

/**
 * @brief 构造函数：初始化无模型自适应控制器（MFAC）
 * @param name ROS2节点名称
 * @note 功能：从YAML文件加载MFAC控制参数并初始化控制器状态
 * @note 初始化内容：
 *       - 加载控制参数（lambda, rho, mu, eta, Kp, Ki等）
 *       - 初始化伪雅可比矩阵phi为对角矩阵（10, 10）
 *       - 初始化所有状态变量为0
 *       - 初始化时间戳用于计算时间间隔
 */
MFAC::MFAC(std::string name) : Node(name){
    const std::string MFAC_param_file = "./src/flex_core/params/MFAC_param.yaml";
    MFAC_Param_ptr = std::make_shared<MFACParam>(MFAC_param_file);
    last_time_ = this->now();  // 初始化时间为当前时间
}

/**
 * @brief MFAC控制器主函数（函数调用运算符重载）
 * @param y_e_ 期望输出向量 [x_e, y_e]（期望位置）
 * @param y_k_ 当前输出向量 [x_k, y_k]（当前位置）
 * @param MFAC MFAC参数结构体指针
 * @return Eigen::Vector2f 控制输出向量 [u_x, u_y]（控制量）
 * @note 功能：执行无模型自适应控制算法，实现位置跟踪控制
 * @note 执行流程：
 *       1. 更新期望值和当前值（UpdateParams）
 *       2. 自适应更新伪雅可比矩阵phi（UpdatePhi）
 *       3. 计算并更新控制量uk（UpdateUk，包含MFAC+PI控制律）
 * @note 输入参数：
 *       - y_e_: 期望位置坐标 (x, y) [m]
 *       - y_k_: 当前实际位置坐标 (x, y) [m]
 *       - MFAC: MFAC参数结构体（包含lambda, rho, mu, eta等）
 * @note 输出参数：
 *       - 返回控制量向量 [u_x, u_y]，用于控制电机运动
 *       - 控制量已进行限幅处理，范围在 [-uk_limit, uk_limit]
 */
Eigen::Vector2f MFAC::operator()(const Eigen::Vector2f& y_e_,
                                 const Eigen::Vector2f& y_k_,
                                 const std::shared_ptr<MFACParam>& MFAC){
    UpdateParams(y_e_,y_k_,MFAC);
    UpdatePhi(MFAC);
    UpdateUk(MFAC);
    return MFAC->uk;
}

/**
 * @brief MFAC控制量限幅函数
 * @param MFAC_OUT 输入的控制量向量 [u_x, u_y]
 * @param LimitValue 限幅值（绝对值）
 * @return Eigen::Vector2f 限幅后的控制量向量
 * @note 功能：对控制量进行饱和限幅，防止控制量过大导致系统不稳定
 * @note 输入参数：
 *       - MFAC_OUT: 原始控制量向量
 *       - LimitValue: 限幅阈值（正数，通常为uk_limit参数值）
 * @note 输出参数：
 *       - 返回限幅后的控制量，范围在 [-LimitValue, LimitValue]
 * @note 限幅规则：
 *       - 使用向量化操作，同时处理x和y两个方向
 *       - 如果控制量 > LimitValue，则限制为 LimitValue
 *       - 如果控制量 < -LimitValue，则限制为 -LimitValue
 *       - 否则保持原值
 * @note 性能优化：使用Eigen向量化操作，避免分支判断，提高计算效率
 */
Eigen::Vector2f MFAC::MFACLimit(const Eigen::Vector2f& MFAC_OUT, float LimitValue){
    // 使用向量化限幅减少分支开销
    return MFAC_OUT.array()
        .max(-LimitValue)
        .min(LimitValue);
}

/**
 * @brief 更新伪雅可比矩阵phi
 * @param MFAC MFAC参数结构体指针（通过引用修改phi, yk_d, yk_1）
 * @note 功能：根据系统输入输出数据自适应更新伪雅可比矩阵
 *       伪雅可比矩阵phi用于描述系统输入输出之间的动态关系，是MFAC算法的核心
 * @note 输入参数：
 *       - MFAC: 包含uk_d（控制量变化量）, yk（当前输出）, yk_1（上次输出）,
 *               phi（伪雅可比矩阵）, eta（学习率）, mu（正则化参数）等
 * @note 输出参数：
 *       - 修改MFAC->phi: 更新后的伪雅可比矩阵（2x2），描述输入输出动态关系
 *       - 修改MFAC->yk_d: 输出变化量 yk - yk_1
 *       - 修改MFAC->yk_1: 保存当前输出值，用于下次计算
 * @note 更新公式：
 *       phi = phi + (eta * (yk_d - phi*uk_d) * uk_d^T) / (mu + ||uk_d||^2)
 *       其中：
 *       - eta: 学习率，控制更新速度（通常0.5~1.0）
 *       - mu: 正则化参数，防止分母为0，提高数值稳定性（通常1~5）
 *       - uk_d: 控制量变化量 uk - uk_1
 *       - yk_d: 输出变化量 yk - yk_1
 * @note 数值稳定性：
 *       - 当分母 (mu + ||uk_d||) 接近0时，跳过更新，避免除零错误
 *       - 使用noalias()优化矩阵运算，避免不必要的临时矩阵
 */
void MFAC::UpdatePhi(const std::shared_ptr<MFACParam>& MFAC){
    const float uk_d_n = MFAC->uk_d.norm();
    const float denom = MFAC->mu + uk_d_n;
    if (denom <= std::numeric_limits<float>::epsilon()) {
        return;
    }

    MFAC->yk_d = MFAC->yk - MFAC->yk_1;

    // 避免临时矩阵，直接 noalias 写入
    const Eigen::Matrix<float, 2, 1> err = MFAC->yk_d - MFAC->phi * MFAC->uk_d;
    MFAC->phi.noalias() += (MFAC->eta / denom) * (err * MFAC->uk_d.transpose());
    MFAC->yk_1 = MFAC->yk;
}

/**
 * @brief 更新控制量uk
 * @param MFAC MFAC参数结构体指针（通过引用修改uk, uk_1, uk_d, error_integral）
 * @note 功能：根据期望输出和实际输出的误差计算新的控制量，实现MFAC+PI复合控制
 * @note 输入参数：
 *       - MFAC: 包含yk_e（期望输出）, yk（当前输出）, uk_1（上次控制量）,
 *               phi（伪雅可比矩阵）, rho（控制增益）, lambda（正则化参数）,
 *               Kp（比例增益）, Ki（积分增益）等参数
 * @note 输出参数：
 *       - 修改MFAC->uk: 新的控制量向量（经过死区控制和限幅处理）
 *       - 修改MFAC->uk_1: 保存上一次的控制量，用于下次计算
 *       - 修改MFAC->uk_d: 控制量变化量 uk - uk_1，用于更新phi
 *       - 修改MFAC->error_integral: 累加误差（误差的积分），用于消除静态误差
 * @note 控制算法流程：
 *       1. 保存上次控制量：uk_1 = uk
 *       2. 计算时间间隔 dt（基于ROS2时间戳，用于积分计算）
 *       3. 计算位置误差：error = yk_e - yk
 *       4. 死区外积分累加（抗饱和策略）：
 *          - 死区阈值：0.3m
 *          - 只在死区外累加误差积分：error_integral += error * dt（仅在|error| >= 0.3时）
 *          - 对累加误差进行限幅（±integral_limit），防止积分饱和
 *       5. 计算增强误差：enhanced_error = error + Kp*error + Ki*error_integral
 *       6. MFAC+PI控制律：
 *          uk = uk_1 + (rho * phi^T * enhanced_error) / (lambda + ||phi||)
 *       7. 死区控制：如果 |error| < 0.3，则控制量为0（避免微小抖动）
 *       8. 控制量限幅：uk = limit(uk, ±uk_limit)
 *       9. 计算控制量变化量：uk_d = uk - uk_1
 * @note 参数说明：
 *       - rho: MFAC控制增益，影响控制响应速度（通常0.1~0.5）
 *       - lambda: 正则化参数，防止分母为0，提高数值稳定性（通常0.1~1.0）
 *       - Kp: PI控制比例增益，增强系统响应速度（通常0.3~0.8）
 *       - Ki: PI控制积分增益，消除静态误差（通常0.05~0.2）
 *       - error_integral: 累加误差，用于消除静态误差
 *       - integral_limit: 积分限幅值，防止积分饱和（通常5.0~20.0）
 *       - uk_limit: 控制量限幅值，防止控制量过大（通常0.3~1.0）
 * @note 积分抗饱和策略：
 *       - 只在死区外累加积分，避免死区内积分项无限增长但控制量为0的矛盾
 *       - 这样可以确保积分项只在有效控制时累加，提高系统稳定性
 *       - 对积分项进行限幅，防止积分饱和导致系统超调
 * @note 死区控制：
 *       - 当误差小于0.5时，控制量为0，避免微小误差引起的系统抖动
 *       - 死区控制与积分抗饱和策略配合，提高系统稳定性
 */
void MFAC::UpdateUk(const std::shared_ptr<MFACParam>& MFAC){
    MFAC->uk_1 = MFAC->uk;

    // 计算时间间隔 dt
    rclcpp::Time current_time = this->now();
    float dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    Eigen::Vector2f error = (MFAC->yk_e - MFAC->yk);
    Eigen::Vector2f error_Abs = error.cwiseAbs();
    
    // 方案一：只在死区外累加积分，避免死区内积分饱和
    // 死区阈值：0.5
    const float deadzone_threshold = 0.5f;
    Eigen::Array2f outside_deadzone = (error_Abs.array() >= deadzone_threshold).cast<float>();
    
    // 只在死区外累加误差积分
    MFAC->error_integral += error.cwiseProduct(outside_deadzone.matrix()) * dt;
    
    // 对累加误差进行限幅，防止积分饱和（使用 yaml 配置的值）
    MFAC->error_integral = MFAC->error_integral.array()
        .max(-MFAC->integral_limit)
        .min(MFAC->integral_limit);
    
    // MFAC+PI控制律：uk = uk_1 + (rho * phi^T * ((1+Kp)*error + Ki*error_integral)) / (lambda + ||phi||)
    // 注意：使用 error 而不是 yk_e，确保稳态时能收敛到期望值
    Eigen::Vector2f enhanced_error = error + MFAC->Kp.cwiseProduct(error) + MFAC->Ki.cwiseProduct(MFAC->error_integral);
    Eigen::Vector2f uk_calculated = MFAC->uk_1 + 
        ((MFAC->rho * MFAC->phi.transpose() * enhanced_error) / 
        (MFAC->lambda + MFAC->phi.norm()));
    
    // 死区控制：如果误差 < 0.3，则控制量为0
    MFAC->uk = (error_Abs.array() < deadzone_threshold).select(0.0f, uk_calculated);

    MFAC->uk = MFACLimit(MFAC->uk, MFAC->uk_limit);
    MFAC->uk_d = MFAC->uk - MFAC->uk_1;
}

/**
 * @brief 更新MFAC参数：期望输出和当前输出
 * @param y_e_ 期望输出向量 [x_e, y_e]（期望位置）
 * @param y_k_ 当前输出向量 [x_k, y_k]（当前位置）
 * @param MFAC MFAC参数结构体指针（通过引用修改yk_e和yk）
 * @note 功能：更新控制器的期望值和当前值，为后续控制计算提供输入数据
 * @note 输入参数：
 *       - y_e_: 期望位置坐标 (x, y) [m]
 *       - y_k_: 当前实际位置坐标 (x, y) [m]
 *       - MFAC: MFAC参数结构体指针
 * @note 输出参数：
 *       - 修改MFAC->yk_e: 期望输出值，用于计算位置误差
 *       - 修改MFAC->yk: 当前输出值，用于计算位置误差和更新phi矩阵
 * @note 调用时机：在每次控制循环开始时调用，确保使用最新的位置信息
 */
void MFAC::UpdateParams(const Eigen::Vector2f& y_e_,
                        const Eigen::Vector2f& y_k_,
                        const std::shared_ptr<MFACParam>& MFAC){
    MFAC->yk_e = y_e_;
    MFAC->yk = y_k_;
}