/**
 * @file s_curve_planner.cpp
 * @brief S曲线速度规划器实现
 * 
 * 对应原代码 straight.cpp 和 srotate.cpp 中的速度规划部分
 */

#include "laser_navigation_refactored/planner/s_curve_planner.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace laser_navigation {

void SCurvePlanner::initializeLinear(double total_distance, double initial_velocity,
                                     const MotionConstraints& constraints) {
    constraints_ = constraints;
    initial_velocity_ = std::abs(initial_velocity);
    current_velocity_ = initial_velocity_;
    current_acceleration_ = 0.0;
    total_distance_ = std::abs(total_distance);
    is_in_final_decel_ = false;
    is_initialized_ = true;

    double max_v = constraints.max_velocity;
    double max_a = constraints.max_acceleration;
    double max_d = constraints.max_deceleration;
    double jerk = constraints.max_jerk;

    // 对应原代码 Straight::Splan 中的计算逻辑
    // 计算加速段参数
    double T1, T2, T5, T6, T7;
    double Sa, Sd;  // 加速距离和减速距离
    
    // 判断是否能达到最大加速度
    // 对应原代码: if (Vmax > pow(acc, 2) / jerk)
    if (max_v > std::pow(max_a, 2) / jerk) {
        T1 = max_a / jerk;
        T2 = max_v / max_a - T1;
    } else {
        T1 = std::sqrt(max_v / jerk);
        T2 = 0;
    }

    // 判断减速阶段
    if (max_v > std::pow(max_d, 2) / jerk) {
        T5 = max_d / jerk;
        T6 = max_v / max_d - T5;
        T7 = T5;
    } else {
        T5 = std::sqrt(max_v / jerk);
        T6 = 0;
        T7 = T5;
    }

    // 计算加速和减速距离
    // 对应原代码: Sa = jerk * T1 * (T1 + T2) * (2 * T1 + T2) / 2
    Sa = jerk * T1 * (T1 + T2) * (2 * T1 + T2) / 2.0;
    Sd = jerk * T5 * (T5 + T6) * (2 * T5 + T6) / 2.0;

    double v_top;
    if (Sa + Sd <= total_distance_) {
        // v_acc_end：s曲线后两段即匀加速到加减速段过渡瞬间的速度 v_dec_start s曲线后两段即匀减速到减减速段过渡瞬间的速度
        params_.v_acc_end = 0.5 * jerk * T1 * T1 + jerk * T1 * T2;//
        params_.v_dec_start = 0.5 * jerk * T7 * T7 + jerk * T7 * T6;
        v_top = max_v;
    } else {
        // 不能达到最大速度，需要重新计算
        // 对应原代码中 total_distance < Sa + Sd 的分支
        double delta = std::pow(max_a, 4) / std::pow(jerk, 2) + max_a * 4 * total_distance_;
        T1 = max_a / jerk;
        T5 = max_d / jerk;
        T7 = T5;
        double Ta = (std::pow(max_a, 2) / jerk + std::sqrt(delta)) / (2 * max_a);
        double Td = (std::pow(max_d, 2) / jerk + std::sqrt(delta)) / (2 * max_d);

        if (Ta >= 2 * T1 && Td >= 2 * T5) {
            v_top = max_a * (Ta - T1);
            params_.v_acc_end = v_top - 0.5 * jerk * std::pow(T1, 2);
            params_.v_dec_start = v_top - 0.5 * jerk * std::pow(T5, 2);
            T2 = (v_top - std::pow(max_a, 2) / jerk) / max_a;
            T6 = (v_top - std::pow(max_d, 2) / jerk) / max_d;
            Sa = jerk * T1 * (T1 + T2) * (2 * T1 + T2) / 2.0;
            Sd = jerk * T5 * (T5 + T6) * (2 * T5 + T6) / 2.0;
        } else {
            // 使用简化模型
            v_top = std::sqrt(2 * max_a * max_d * total_distance_ / (max_a + max_d));
            // 0.8 是经验值，没有精确分段参数T2、T6保证有加减速段缓冲
            params_.v_acc_end = v_top * 0.8;
            params_.v_dec_start = v_top * 0.8;
            Sd = std::pow(v_top, 2) / (2 * max_d);
            T2 = 0;
            T6 = 0;
        }
    }

    params_.v_max = std::min(max_v, v_top);
    params_.s_deceleration = Sd;

    // 保存时间参数
    params_.t1 = T1;
    params_.t2 = T2;
    params_.t5 = T5;
    params_.t6 = T6;
    params_.t7 = T7;
}

void SCurvePlanner::initializeRotation(double total_angle, double initial_angular_velocity,
                                       const MotionConstraints& constraints) {
    // 旋转规划与直线类似，只是参数不同
    // 对应原代码 Srotate::SplanRot
    MotionConstraints rot_constraints;
    rot_constraints.max_velocity = constraints.max_angular_velocity;
    rot_constraints.max_acceleration = constraints.max_angular_acceleration;
    rot_constraints.max_deceleration = constraints.max_angular_deceleration;
    rot_constraints.max_jerk = constraints.max_jerk;
    
    initializeLinear(total_angle, initial_angular_velocity, rot_constraints);
}

void SCurvePlanner::updatePath(double /*remaining_distance*/, double current_velocity,
                               const MotionConstraints& constraints) {
    constraints_ = constraints;
    is_in_final_decel_ = false;
    
    // 重新计算减速距离
    // 对应原代码中路径更新时的逻辑
    double max_d = constraints.max_deceleration;
    double jerk = constraints.max_jerk;
    
    double T5 = max_d / jerk;
    double T6 = std::abs(current_velocity) / max_d - T5;
    if (T6 < 0) T6 = 0;
    
    params_.s_deceleration = jerk * T5 * (T5 + T6) * (2 * T5 + T6) / 2.0;
    params_.v_dec_start = 0.5 * jerk * T5 * T5 + jerk * T5 * T6;
}

double SCurvePlanner::computeVelocity(double remaining_distance, double dt) {
    if (!is_initialized_) return 0.0;

    remaining_distance = std::abs(remaining_distance);
    
    if (remaining_distance <= params_.s_deceleration) {
        // 进入减速段
        return computeDecelerationPhase(remaining_distance, dt);
    } else {
        // 加速/匀速段
        return computeAccelerationPhase(dt);
    }
}

double SCurvePlanner::computeAccelerationPhase(double dt) {
    // 对应原代码 Straight::ScurveAcc
    double jerk = constraints_.max_jerk;
    double max_a = constraints_.max_acceleration;
    
    if (current_velocity_ >= params_.v_acc_end) {
        // 减加速段（加速度减小）
        current_acceleration_ -= jerk * dt;
        if (current_acceleration_ < 0) {
            current_acceleration_ = 0;
        }
    } else {
        // 加加速段或匀加速段
        current_acceleration_ += jerk * dt;
        if (current_acceleration_ > max_a) {
            current_acceleration_ = max_a;
        }
    }
    
    current_velocity_ += current_acceleration_ * dt;
    
    // 限制最大速度
    if (current_velocity_ >= params_.v_max) {
        current_acceleration_ = 0;
        current_velocity_ = params_.v_max;
    }
    
    return current_velocity_;
}

double SCurvePlanner::computeDecelerationPhase(double remaining_distance, double dt) {
    // 对应原代码 Straight::ScurveDec
    // 支持非零终点速度的减速（用于段间平滑过渡）
    double jerk = constraints_.max_jerk;
    double max_d = constraints_.max_deceleration;
    double target_v = target_end_velocity_;  // 目标终点速度
    
    // 计算减速到目标速度需要的距离
    double delta_v = current_velocity_ - target_v;
    double s_needed = 0.0;
    if (delta_v > 0 && max_d > 0.001) {
        s_needed = (current_velocity_ * current_velocity_ - target_v * target_v) / (2 * max_d);
    }
    
    if (remaining_distance <= s_needed || is_in_final_decel_) {
        // 最后的减速段
        if (!is_in_final_decel_) {
            // 计算线性减速系数: v = target_v + k * (s - 0)
            // 在 s = remaining_distance 时，v = current_velocity_
            // 在 s = 0 时，v = target_v
            if (remaining_distance > 0.001) {
                decel_coefficient_ = (current_velocity_ - target_v) / remaining_distance;
            } else {
                decel_coefficient_ = 0;
            }
            is_in_final_decel_ = true;
        }
        
        // v = target_v + k * s
        current_velocity_ = target_v + decel_coefficient_ * remaining_distance;
        current_acceleration_ = 0;  // 简化处理
    } else if (current_velocity_ <= params_.v_dec_start) {
        // S曲线减速中期
        current_acceleration_ -= jerk * dt;
        if (current_acceleration_ <= -max_d) {
            current_acceleration_ = -max_d;
        }
        current_velocity_ += current_acceleration_ * dt;
    } else {
        // S曲线减速初期（加减速段）
        current_acceleration_ -= jerk * dt;
        if (current_acceleration_ <= -max_d) {
            current_acceleration_ = -max_d;
        }
        current_velocity_ += current_acceleration_ * dt;
    }
    
    return std::max(target_v, current_velocity_);
}

double SCurvePlanner::computeAngularVelocity(double remaining_angle, double dt) {
    // 角速度规划与线速度类似
    return computeVelocity(remaining_angle, dt);
}

void SCurvePlanner::reset() {
    current_velocity_ = 0.0;
    current_acceleration_ = 0.0;
    initial_velocity_ = 0.0;
    total_distance_ = 0.0;
    is_in_final_decel_ = false;
    is_initialized_ = false;
    decel_coefficient_ = 0.0;
    target_end_velocity_ = 0.0;
    params_ = SCurveParams();
}

void SCurvePlanner::computeTimeParameters() {
    // 计算各段时间，已在initializeLinear中处理
}

}  // namespace laser_navigation
