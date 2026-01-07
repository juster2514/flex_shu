#include "flex_core/MFAC.hpp"
#include <limits>

/**
 * @brief 构造函数：初始化无模型自适应控制器（MFAC）
 * @param name 节点名称
 * @note 功能：从YAML文件加载MFAC控制参数（lambda, rho, mu, eta等）
 */
MFAC::MFAC(std::string name) : Node(name){
    const std::string MFAC_param_file = "./src/flex_core/params/MFAC_param.yaml";
    MFAC_Param_ptr = std::make_shared<MFACParam>(MFAC_param_file);
}

/**
 * @brief MFAC控制器主函数（函数调用运算符重载）
 * @param y_e_ 期望输出向量 [x_e, y_e]（期望位置）
 * @param y_k_ 当前输出向量 [x_k, y_k]（当前位置）
 * @param MFAC MFAC参数结构体指针
 * @return Eigen::Vector2f 控制输出向量 [u_x, u_y]（控制量）
 * @note 功能：执行无模型自适应控制算法
 *       1. 更新期望值和当前值
 *       2. 更新伪雅可比矩阵phi
 *       3. 计算并更新控制量uk
 * @note 输入参数：
 *       - y_e_: 期望位置坐标 (x, y)
 *       - y_k_: 当前实际位置坐标 (x, y)
 *       - MFAC: MFAC参数结构体（包含lambda, rho, mu, eta等）
 * @note 输出参数：
 *       - 返回控制量向量 [u_x, u_y]，用于控制电机运动
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
 * @note 功能：对控制量进行饱和限幅，防止控制量过大
 * @note 输入参数：
 *       - MFAC_OUT: 原始控制量向量
 *       - LimitValue: 限幅阈值（正数，例如0.20）
 * @note 输出参数：
 *       - 返回限幅后的控制量，范围在 [-LimitValue, LimitValue]
 * @note 限幅规则：
 *       - 如果控制量 > LimitValue，则限制为 LimitValue
 *       - 如果控制量 < -LimitValue，则限制为 -LimitValue
 *       - 否则保持原值
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
 *       伪雅可比矩阵phi用于描述系统输入输出之间的动态关系
 * @note 输入参数：
 *       - MFAC: 包含uk_d, yk, yk_1, phi, eta, mu等参数
 * @note 输出参数：
 *       - 修改MFAC->phi: 更新后的伪雅可比矩阵（2x2）
 *       - 修改MFAC->yk_d: 输出变化量 yk - yk_1
 *       - 修改MFAC->yk_1: 保存当前输出值，用于下次计算
 * @note 更新公式：
 *       phi = phi + (eta * (yk_d - phi*uk_d) * uk_d^T) / (mu + ||uk_d||)
 *       其中：eta为学习率，mu为正则化参数，uk_d为控制量变化量
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
 * @param MFAC MFAC参数结构体指针（通过引用修改uk, uk_1, uk_d）
 * @note 功能：根据期望输出和实际输出的误差计算新的控制量
 * @note 输入参数：
 *       - MFAC: 包含yk_e, yk, uk_1, phi, rho, lambda等参数
 * @note 输出参数：
 *       - 修改MFAC->uk: 新的控制量向量（经过限幅）
 *       - 修改MFAC->uk_1: 保存上一次的控制量
 *       - 修改MFAC->uk_d: 控制量变化量 uk - uk_1
 * @note 控制算法：
 *       1. 计算位置误差的绝对值
 *       2. 如果误差 < 0.5，则控制量为0（死区控制）
 *       3. 否则使用MFAC控制律计算控制量：
 *          uk = uk_1 + (rho * phi^T * (yk_e - yk)) / (lambda + ||phi||)
 *       4. 对控制量进行限幅（±0.20）
 * @note 参数说明：
 *       - rho: 控制增益
 *       - lambda: 正则化参数，防止分母为0
 */
void MFAC::UpdateUk(const std::shared_ptr<MFACParam>& MFAC){
    MFAC->uk_1 = MFAC->uk;

    Eigen::Vector2f error = (MFAC->yk_e - MFAC->yk).cwiseAbs();
    
    Eigen::Vector2f uk_calculated = MFAC->uk_1 + 
        ((MFAC->rho * MFAC->phi.transpose() * (MFAC->yk_e - MFAC->yk)) / 
        (MFAC->lambda + MFAC->phi.norm()));
    
    MFAC->uk = (error.array() < 0.5f).select(0.0f, uk_calculated);

    MFAC->uk = MFACLimit(MFAC->uk, MFAC->uk_limit);
    MFAC->uk_d = MFAC->uk - MFAC->uk_1;
}

/**
 * @brief 更新MFAC参数：期望输出和当前输出
 * @param y_e_ 期望输出向量 [x_e, y_e]
 * @param y_k_ 当前输出向量 [x_k, y_k]
 * @param MFAC MFAC参数结构体指针（通过引用修改yk_e和yk）
 * @note 功能：更新控制器的期望值和当前值
 * @note 输入参数：
 *       - y_e_: 期望位置坐标
 *       - y_k_: 当前实际位置坐标
 *       - MFAC: MFAC参数结构体
 * @note 输出参数：
 *       - 修改MFAC->yk_e: 期望输出值
 *       - 修改MFAC->yk: 当前输出值
 */
void MFAC::UpdateParams(const Eigen::Vector2f& y_e_,
                        const Eigen::Vector2f& y_k_,
                        const std::shared_ptr<MFACParam>& MFAC){
    MFAC->yk_e = y_e_;
    MFAC->yk = y_k_;
}