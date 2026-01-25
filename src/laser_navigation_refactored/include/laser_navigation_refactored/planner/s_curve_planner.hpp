/**
 * @file s_curve_planner.hpp
 * @brief S曲线速度规划器
 * 
 * 实现7段式S曲线速度规划
 */

#ifndef LASER_NAVIGATION_REFACTORED_PLANNER_S_CURVE_PLANNER_HPP_
#define LASER_NAVIGATION_REFACTORED_PLANNER_S_CURVE_PLANNER_HPP_

#include "laser_navigation_refactored/core/types.hpp"

namespace laser_navigation {

/**
 * @brief S曲线速度规划器
 * 
 * 实现7段式S曲线速度规划，对应原代码中 straight.cpp 和 srotate.cpp 的规划逻辑
 * 
 * 7段式S曲线包含：
 * - T1: 加加速段（jerk > 0）
 * - T2: 匀加速段（jerk = 0, acc = max_acc）
 * - T3: 减加速段（jerk < 0）
 * - T4: 匀速段
 * - T5: 加减速段（jerk < 0）
 * - T6: 匀减速段（jerk = 0, acc = -max_dec）
 * - T7: 减减速段（jerk > 0）
 */
class SCurvePlanner {
public:
    SCurvePlanner() = default;

    //==========================================================================
    // 初始化接口
    //==========================================================================

    /**
     * @brief 初始化直线段的S曲线规划
     * @param total_distance 总距离
     * @param initial_velocity 初始速度
     * @param constraints 运动约束
     * 
     * 对应原代码 Straight::Splan 函数
     */
    void initializeLinear(double total_distance, double initial_velocity,
                          const MotionConstraints& constraints);

    /**
     * @brief 初始化旋转段的S曲线规划
     * @param total_angle 总角度（弧度，绝对值）
     * @param initial_angular_velocity 初始角速度（绝对值）
     * @param constraints 运动约束
     * 
     * 对应原代码 Srotate::SplanRot 函数
     */
    void initializeRotation(double total_angle, double initial_angular_velocity,
                            const MotionConstraints& constraints);

    /**
     * @brief 更新路径参数（用于路径拼接时的热更新）
     * @param remaining_distance 剩余距离
     * @param current_velocity 当前速度
     * @param constraints 运动约束
     * 
     * 对应原代码中路径拼接时重新计算减速点的逻辑
     */
    void updatePath(double remaining_distance, double current_velocity,
                    const MotionConstraints& constraints);

    /**
     * @brief 设置目标终点速度（用于平滑过渡到下一段）
     * @param target_end_velocity 目标终点速度
     * 
     * 默认为0，设置非零值可实现段间平滑过渡
     */
    void setTargetEndVelocity(double target_end_velocity) {
        target_end_velocity_ = target_end_velocity;
    }

    /**
     * @brief 获取目标终点速度
     */
    double getTargetEndVelocity() const { return target_end_velocity_; }

    //==========================================================================
    // 速度计算接口
    //==========================================================================

    /**
     * @brief 计算下一时刻的线速度
     * @param remaining_distance 剩余距离
     * @param dt 时间步长
     * @return 规划的线速度
     * 
     * 对应原代码 Straight::ScurveDec 和 Straight::ScurveAcc 函数
     */
    double computeVelocity(double remaining_distance, double dt);

    /**
     * @brief 计算旋转的角速度
     * @param remaining_angle 剩余角度（绝对值）
     * @param dt 时间步长
     * @return 规划的角速度（绝对值）
     * 
     * 对应原代码 Srotate::ScurveDecRot 和 Srotate::ScurveAccRot 函数
     */
    double computeAngularVelocity(double remaining_angle, double dt);

    //==========================================================================
    // 状态查询
    //==========================================================================

    /**
     * @brief 重置规划器状态
     */
    void reset();

    /**
     * @brief 获取S曲线参数
     */
    const SCurveParams& getParameters() const { return params_; }

    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return is_initialized_; }

    /**
     * @brief 获取当前速度
     */
    double getCurrentVelocity() const { return current_velocity_; }

    /**
     * @brief 获取当前加速度
     */
    double getCurrentAcceleration() const { return current_acceleration_; }

private:
    //==========================================================================
    // 内部计算函数
    //==========================================================================

    /**
     * @brief 计算加速段速度
     * @param dt 时间步长
     * @return 加速后的速度
     */
    double computeAccelerationPhase(double dt);

    /**
     * @brief 计算减速段速度
     * @param remaining_distance 剩余距离
     * @param dt 时间步长
     * @return 减速后的速度
     */
    double computeDecelerationPhase(double remaining_distance, double dt);

    /**
     * @brief 计算S曲线各段时间参数
     */
    void computeTimeParameters();

    //==========================================================================
    // 成员变量
    //==========================================================================

    SCurveParams params_;              ///< S曲线参数
    MotionConstraints constraints_;    ///< 运动约束
    
    // 状态变量
    double current_velocity_{0.0};       ///< 当前速度
    double current_acceleration_{0.0};   ///< 当前加速度
    double initial_velocity_{0.0};       ///< 初始速度
    double total_distance_{0.0};         ///< 总距离
    
    // 减速段状态
    double decel_coefficient_{0.0};      ///< 最后减速段的线性系数
    bool is_in_final_decel_{false};      ///< 是否在最后线性减速段
    bool is_initialized_{false};         ///< 是否已初始化
    double target_end_velocity_{0.0};    ///< 目标终点速度（用于平滑过渡）
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PLANNER_S_CURVE_PLANNER_HPP_
