/**
 * @file lqr_controller.cpp
 * @brief LQR路径跟踪控制器实现
 * 
 * 对应原代码 straight.cpp 的 lqr 函数
 * 支持差速和四转四驱底盘
 * 支持可选的里程计速度反馈
 */

#include "laser_navigation_refactored/controller/lqr_controller.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include "laser_navigation_refactored/core/optional.hpp"
#include <cmath>

namespace laser_navigation {

//==============================================================================
// StraightLQRController 实现
//==============================================================================

StraightLQRController::StraightLQRController(const LQRParams& params)
    : params_(params) {}

void StraightLQRController::initialize(double target_heading, double line_k, double line_b,
                                       bool is_vertical, double end_x,
                                       ChassisType chassis_type) {
    target_heading_ = target_heading;
    line_k_ = line_k;
    line_b_ = line_b;
    is_vertical_ = is_vertical;
    end_x_ = end_x;
    chassis_type_ = chassis_type;
    is_initialized_ = true;
    last_velocity_.reset();
}

void StraightLQRController::reset() {
    is_initialized_ = false;
    target_heading_ = 0.0;
    line_k_ = 0.0;
    line_b_ = 0.0;
    end_x_ = 0.0;
    is_vertical_ = false;
    last_velocity_.reset();
}

Velocity StraightLQRController::compute(const Pose2D& current_pose, 
                                         const Velocity& reference_velocity,
                                         bool is_forward) {
    if (!is_initialized_) {
        return Velocity(0, 0);
    }

    // 参考速度（绝对值）
    double v_ref = std::abs(reference_velocity.linear);
    
    // 对应原代码 Straight::lqr 函数
    // 构建状态空间模型: dx = Ax + Bu
    // 状态: [x, y, theta]
    // 输入: [v, omega]
    
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    
    // Q矩阵（状态权重）
    Eigen::Matrix3d Q;
    Q << params_.q1, 0, 0,
         0, params_.q2, 0,
         0, 0, params_.q3;
    
    // R矩阵（输入权重）
    Eigen::Matrix2d R;
    R << params_.r1, 0,
         0, params_.r2;
    
    // 系统矩阵A（线性化）
    // 对应原代码:
    // A << 0, 0, -V * sin(Theta0),
    //      0, 0, V * cos(Theta0),
    //      0, 0, 0;
    Eigen::Matrix3d A;
    A << 0, 0, -v_ref * std::sin(target_heading_),
         0, 0, v_ref * std::cos(target_heading_),
         0, 0, 0;
    
    // 输入矩阵B
    // 对应原代码:
    // B << cos(Theta0), 0,
    //      sin(Theta0), 0,
    //      0, 1;
    Eigen::Matrix<double, 3, 2> B;
    B << std::cos(target_heading_), 0,
         std::sin(target_heading_), 0,
         0, 1;
    
    // 离散化: Ad = A*Ts + I, Bd = B*Ts
    double dt = params_.dt;
    Eigen::Matrix3d Ad = A * dt + I;
    Eigen::Matrix<double, 3, 2> Bd = B * dt;
    
    // 求解离散代数Riccati方程
    Eigen::Matrix3d P = solveDARE(Ad, Bd, Q, R);
    
    // 计算LQR增益: K = (R + B'PB)^(-1) * B'PA
    Eigen::Matrix<double, 2, 3> K = 
        (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    
    // 角度处理：如果是倒车，需要调整当前角度
    double current_theta = current_pose.yaw;
    if (!is_forward) {
        current_theta += M_PI;
        current_theta = math::MathUtils::normalizeAngle(current_theta);
    }
    
    // 确保角度差在合理范围内
    double theta_diff = math::MathUtils::angleDifference(target_heading_, current_theta);
    if (theta_diff > M_PI) {
        current_theta += 2 * M_PI;
    } else if (theta_diff < -M_PI) {
        current_theta -= 2 * M_PI;
    }
    
    // 计算参考点（当前位置在直线上的投影）
    Eigen::Vector2d ref_point = computeReferencePoint(current_pose);
    
    // 当前状态
    Eigen::Vector3d X_current(current_pose.x, current_pose.y, current_theta);
    
    // 参考状态
    Eigen::Vector3d X_ref(ref_point(0), ref_point(1), target_heading_);
    
    // 参考输入
    Eigen::Vector2d U_ref(v_ref, 0);
    
    // 计算最优控制: u = u_ref - K*(x - x_ref)
    // 对应原代码: u = Ur - K * (X - Xr)
    Eigen::Vector2d U = U_ref - K * (X_current - X_ref);
    
    // 限幅
    double v_out = math::MathUtils::clamp(U(0), -1.2, 1.2);
    double w_out = math::MathUtils::clamp(U(1), -0.2, 0.2);
    
    // 根据方向调整符号
    if (!is_forward) {
        v_out = -std::abs(v_out);
    }
    
    // 计算横向速度（仅四转四驱）//TODO 后续要优化为直接用LQR计算出 vx vy w
    double vy_out = 0.0;
    if (chassis_type_ == ChassisType::k4WIS4WID) {
        // 计算横向误差
        double lateral_error = computeLateralVelocity(
            (X_current - X_ref).head<2>().norm() * 
            std::sin(std::atan2(X_current(1) - X_ref(1), X_current(0) - X_ref(0)) - target_heading_),
            0.3);  // 最大横向速度 0.3 m/s
        vy_out = lateral_error;
    }
    
    last_velocity_ = Velocity(v_out, vy_out, w_out);
    return last_velocity_;
}

Velocity StraightLQRController::computeWithFeedback(const Pose2D& current_pose,
                                                      const Velocity& reference_velocity,
                                                      bool is_forward,
                                                      const Optional<OdometryFeedback>& odom_feedback) {
    // 首先计算基础控制量
    Velocity base_velocity = compute(current_pose, reference_velocity, is_forward);
    
    // 如果有有效的里程计反馈，进行速度平滑
    if (odom_feedback.has_value() && odom_feedback.value().is_valid) {
        const auto& current_vel = odom_feedback.value().velocity;
        
        // 速度平滑系数（防止速度跳变）
        const double smooth_factor = 0.3;  // 0-1, 越小越平滑
        
        // 使用里程计速度作为参考，平滑过渡
        base_velocity.linear_x = current_vel.linear_x + 
            smooth_factor * (base_velocity.linear_x - current_vel.linear_x);
        base_velocity.angular = current_vel.angular + 
            smooth_factor * (base_velocity.angular - current_vel.angular);
        
        if (chassis_type_ == ChassisType::k4WIS4WID) {
            base_velocity.linear_y = current_vel.linear_y + 
                smooth_factor * (base_velocity.linear_y - current_vel.linear_y);
        }
    }
    
    last_velocity_ = base_velocity;
    return base_velocity;
}

double StraightLQRController::computeLateralVelocity(double lateral_error, double max_vy) const {
    // 比例控制计算横向速度
    const double kp = 1.5;  // 比例增益
    double vy = kp * lateral_error;
    return math::MathUtils::clamp(vy, -max_vy, max_vy);
}

Eigen::Vector2d StraightLQRController::computeReferencePoint(const Pose2D& current_pose) const {
    double ref_x, ref_y;
    
    if (!is_vertical_) {
        // 非垂直线：计算投影点
        // 对应原代码:
        // Xr = (act_x - k * b + k * act_y) / (pow(k, 2) + 1)
        // Yr = (k * act_x + pow(k, 2) * act_y + b) / (pow(k, 2) + 1)
        double k2 = line_k_ * line_k_;
        ref_x = (current_pose.x + line_k_ * (current_pose.y - line_b_)) / (k2 + 1);
        ref_y = (line_k_ * current_pose.x + k2 * current_pose.y + line_b_) / (k2 + 1);
    } else {
        // 垂直线：x坐标固定
        ref_x = end_x_;
        ref_y = current_pose.y;
    }
    
    return Eigen::Vector2d(ref_x, ref_y);
}

Eigen::Matrix3d StraightLQRController::solveDARE(const Eigen::Matrix3d& A,
                                                   const Eigen::Matrix<double, 3, 2>& B,
                                                   const Eigen::Matrix3d& Q,
                                                   const Eigen::Matrix2d& R) {
    // 迭代求解离散代数Riccati方程
    // 对应原代码中的Riccati迭代
    Eigen::Matrix3d P = Q;
    
    for (int i = 0; i < params_.max_iterations; ++i) {
        Eigen::Matrix3d P_new = Q + A.transpose() * P * A -
            A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        
        // 检查收敛
        if ((P_new - P).norm() < 0.01) {
            break;
        }
        P = P_new;
    }
    
    return P;
}

//==============================================================================
// BezierLQRController 实现
//==============================================================================

BezierLQRController::BezierLQRController(const BezierLQRParams& params)
    : params_(params) {}

void BezierLQRController::reset() {
    last_velocity_.reset();
}

Velocity BezierLQRController::compute(const Pose2D& current_pose,
                                       const Eigen::Vector2d& reference_point,
                                       const Velocity& reference_velocity,
                                       double reference_heading,
                                       bool is_forward) {
    // 对应原代码中贝塞尔曲线跟踪的LQR控制
    double v_ref = reference_velocity.linear_x;
    double w_ref = reference_velocity.angular;
    
    // 构建状态空间模型
    Eigen::Matrix3d Q;
    Q << params_.q1, 0, 0,
         0, params_.q2, 0,
         0, 0, params_.q3;
    
    Eigen::Matrix2d R;
    R << params_.r1, 0,
         0, params_.r2;
    
    // 系统矩阵
    Eigen::Matrix3d A;
    A << 0, 0, -v_ref * std::sin(reference_heading),
         0, 0, v_ref * std::cos(reference_heading),
         0, 0, 0;
    
    Eigen::Matrix<double, 3, 2> B;
    B << std::cos(reference_heading), 0,
         std::sin(reference_heading), 0,
         0, 1;
    
    // 离散化
    double dt = params_.dt;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    A = A * dt + I;
    B = B * dt;
    
    // 求解DARE
    Eigen::Matrix3d P = solveDARE(A, B, Q, R);
    
    // 计算增益
    Eigen::Matrix<double, 2, 3> K = 
        (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    
    // 处理倒车时的角度
    double current_theta = current_pose.yaw;
    if (!is_forward) {
        current_theta += M_PI;
        current_theta = math::MathUtils::normalizeAngle(current_theta);
    }
    
    // 状态和参考
    Eigen::Vector3d X_ref(reference_point(0), reference_point(1), reference_heading);
    Eigen::Vector3d X_current(current_pose.x, current_pose.y, current_theta);
    Eigen::Vector2d U_ref(v_ref, w_ref);
    
    // 计算控制量
    Eigen::Vector2d U = U_ref - K * (X_current - X_ref);
    
    double v_out = U(0);
    double w_out = U(1);
    
    // 根据方向调整
    if (!is_forward) {
        v_out = -std::abs(v_out);
    }
    
    // 计算横向速度（仅四转四驱）
    double vy_out = 0.0;
    if (chassis_type_ == ChassisType::k4WIS4WID) {
        // 计算横向误差和航向误差
        double lateral_error = (current_pose.y - reference_point(1)) * std::cos(reference_heading) -
                               (current_pose.x - reference_point(0)) * std::sin(reference_heading);
        double heading_error = math::MathUtils::angleDifference(reference_heading, current_theta);
        vy_out = computeLateralVelocity(lateral_error, heading_error, 0.3);
    }
    
    last_velocity_ = Velocity(v_out, vy_out, w_out);
    return last_velocity_;
}

Velocity BezierLQRController::computeWithFeedback(const Pose2D& current_pose,
                                                    const Eigen::Vector2d& reference_point,
                                                    const Velocity& reference_velocity,
                                                    double reference_heading,
                                                    bool is_forward,
                                                    const Optional<OdometryFeedback>& odom_feedback) {
    // 首先计算基础控制量
    Velocity base_velocity = compute(current_pose, reference_point, reference_velocity, 
                                     reference_heading, is_forward);
    
    // 如果有有效的里程计反馈，进行速度平滑
    if (odom_feedback.has_value() && odom_feedback.value().is_valid) {
        const auto& current_vel = odom_feedback.value().velocity;
        
        // 速度平滑系数
        const double smooth_factor = 0.3;
        
        base_velocity.linear_x = current_vel.linear_x + 
            smooth_factor * (base_velocity.linear_x - current_vel.linear_x);
        base_velocity.angular = current_vel.angular + 
            smooth_factor * (base_velocity.angular - current_vel.angular);
        
        if (chassis_type_ == ChassisType::k4WIS4WID) {
            base_velocity.linear_y = current_vel.linear_y + 
                smooth_factor * (base_velocity.linear_y - current_vel.linear_y);
        }
    }
    
    last_velocity_ = base_velocity;
    return base_velocity;
}

double BezierLQRController::computeLateralVelocity(double lateral_error, 
                                                     double heading_error,
                                                     double max_vy) const {
    // 结合横向误差和航向误差计算横向速度
    const double kp_lateral = 1.2;
    const double kp_heading = 0.3;
    
    double vy = kp_lateral * lateral_error + kp_heading * heading_error;
    return math::MathUtils::clamp(vy, -max_vy, max_vy);
}

Eigen::Matrix3d BezierLQRController::solveDARE(const Eigen::Matrix3d& A,
                                                 const Eigen::Matrix<double, 3, 2>& B,
                                                 const Eigen::Matrix3d& Q,
                                                 const Eigen::Matrix2d& R) {
    Eigen::Matrix3d P = Q;
    const int max_iter = 200;
    const double eps = 0.01;
    
    for (int i = 0; i < max_iter; ++i) {
        Eigen::Matrix3d P_new = Q + A.transpose() * P * A -
            A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        
        if ((P_new - P).norm() < eps) {
            break;
        }
        P = P_new;
    }
    
    return P;
}

}  // namespace laser_navigation
