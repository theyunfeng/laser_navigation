/**
 * @file lqr_controller.hpp
 * @brief LQR路径跟踪控制器
 * 
 * 实现基于LQR的路径跟踪控制
 * 支持差速底盘和四转四驱底盘
 * 支持可选的里程计速度反馈
 */

#ifndef LASER_NAVIGATION_REFACTORED_CONTROLLER_LQR_CONTROLLER_HPP_
#define LASER_NAVIGATION_REFACTORED_CONTROLLER_LQR_CONTROLLER_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include "laser_navigation_refactored/core/optional.hpp"
#include <Eigen/Dense>

namespace laser_navigation {

/**
 * @brief 直线路径LQR跟踪控制器
 * 
 * 对应原代码中 straight.cpp 的 LQR 控制部分
 * 使用离散时间LQR控制器跟踪直线路径
 * 
 * 支持：
 * - 差速底盘：输出 (vx, ω)
 * - 四转四驱：输出 (vx, vy, ω)，可实现斜向跟踪
 * - 可选的里程计速度反馈作为前馈
 */
class StraightLQRController {
public:
    StraightLQRController() = default;
    explicit StraightLQRController(const LQRParams& params);

    /**
     * @brief 初始化控制器
     * @param target_heading 目标航向角
     * @param line_k 直线斜率
     * @param line_b 直线截距
     * @param is_vertical 是否为垂直线
     * @param end_x 终点x坐标（用于垂直线）
     * @param chassis_type 底盘类型
     * 
     * 对应原代码中设置直线参数的逻辑
     */
    void initialize(double target_heading, double line_k, double line_b,
                    bool is_vertical, double end_x,
                    ChassisType chassis_type = ChassisType::kDifferential);

    /**
     * @brief 计算控制量（不使用里程计反馈）
     * @param current_pose 当前位姿
     * @param reference_velocity 参考速度（规划值）
     * @param is_forward 是否前进
     * @return 控制速度指令
     * 
     * 对应原代码 Straight::lqr 函数
     */
    Velocity compute(const Pose2D& current_pose, const Velocity& reference_velocity,
                     bool is_forward);

    /**
     * @brief 计算控制量（使用里程计反馈）
     * @param current_pose 当前位姿
     * @param reference_velocity 参考速度（规划值）
     * @param is_forward 是否前进
     * @param odom_feedback 里程计反馈（可选）
     * @return 控制速度指令
     * 
     * 如果里程计反馈有效，将其作为速度前馈参考，
     * 可以实现更平滑的速度过渡和更好的跟踪性能
     */
    Velocity computeWithFeedback(const Pose2D& current_pose, 
                                  const Velocity& reference_velocity,
                                  bool is_forward,
                                  const Optional<OdometryFeedback>& odom_feedback);

    /**
     * @brief 设置LQR参数
     */
    void setParams(const LQRParams& params) { params_ = params; }

    /**
     * @brief 获取LQR参数
     */
    const LQRParams& getParams() const { return params_; }

    /**
     * @brief 设置底盘类型
     */
    void setChassisType(ChassisType type) { chassis_type_ = type; }

    /**
     * @brief 获取底盘类型
     */
    ChassisType getChassisType() const { return chassis_type_; }

    /**
     * @brief 重置控制器
     */
    void reset();

private:
    /**
     * @brief 求解离散代数Riccati方程
     * @param A 系统矩阵
     * @param B 输入矩阵
     * @param Q 状态权重矩阵
     * @param R 输入权重矩阵
     * @return Riccati方程的解P
     */
    Eigen::Matrix3d solveDARE(const Eigen::Matrix3d& A, 
                               const Eigen::Matrix<double, 3, 2>& B,
                               const Eigen::Matrix3d& Q, 
                               const Eigen::Matrix2d& R);

    /**
     * @brief 计算参考点（投影到直线上）
     * @param current_pose 当前位姿
     * @return 投影点坐标
     */
    Eigen::Vector2d computeReferencePoint(const Pose2D& current_pose) const;

    /**
     * @brief 计算四转四驱的横向速度分量
     * @param lateral_error 横向误差
     * @param max_vy 最大横向速度
     * @return 横向速度
     */
    double computeLateralVelocity(double lateral_error, double max_vy) const;

    LQRParams params_;           ///< LQR参数
    ChassisType chassis_type_{ChassisType::kDifferential}; ///< 底盘类型
    
    double target_heading_{0.0}; ///< 目标航向角
    double line_k_{0.0};         ///< 直线斜率
    double line_b_{0.0};         ///< 直线截距
    double end_x_{0.0};          ///< 终点x坐标
    bool is_vertical_{false};    ///< 是否为垂直线
    bool is_initialized_{false}; ///< 是否已初始化
    
    // 上一次速度（用于平滑）
    Velocity last_velocity_;
};

/**
 * @brief 贝塞尔曲线LQR跟踪控制器
 * 
 * 对应原代码中 correct.cpp 的贝塞尔曲线跟踪LQR控制部分
 * 
 * 支持：
 * - 差速底盘：沿曲线切向行驶
 * - 四转四驱：可实现更优的曲线跟踪（横向补偿）
 */
class BezierLQRController {
public:
    BezierLQRController() = default;
    explicit BezierLQRController(const BezierLQRParams& params);

    /**
     * @brief 设置底盘类型
     */
    void setChassisType(ChassisType type) { chassis_type_ = type; }

    /**
     * @brief 计算控制量（不使用里程计反馈）
     * @param current_pose 当前位姿
     * @param reference_point 参考点（规划点）
     * @param reference_velocity 参考速度
     * @param reference_heading 参考航向
     * @param is_forward 是否前进
     * @return 控制速度指令
     * 
     * 对应原代码中贝塞尔曲线跟踪的LQR控制
     */
    Velocity compute(const Pose2D& current_pose, const Eigen::Vector2d& reference_point,
                     const Velocity& reference_velocity, double reference_heading,
                     bool is_forward);

    /**
     * @brief 计算控制量（使用里程计反馈）
     * @param current_pose 当前位姿
     * @param reference_point 参考点（规划点）
     * @param reference_velocity 参考速度
     * @param reference_heading 参考航向
     * @param is_forward 是否前进
     * @param odom_feedback 里程计反馈（可选）
     * @return 控制速度指令
     */
    Velocity computeWithFeedback(const Pose2D& current_pose, 
                                  const Eigen::Vector2d& reference_point,
                                  const Velocity& reference_velocity, 
                                  double reference_heading,
                                  bool is_forward,
                                  const Optional<OdometryFeedback>& odom_feedback);

    /**
     * @brief 设置参数
     */
    void setParams(const BezierLQRParams& params) { params_ = params; }

    /**
     * @brief 获取参数
     */
    const BezierLQRParams& getParams() const { return params_; }

    /**
     * @brief 获取底盘类型
     */
    ChassisType getChassisType() const { return chassis_type_; }

    /**
     * @brief 重置控制器
     */
    void reset();

private:
    /**
     * @brief 求解离散代数Riccati方程
     */
    Eigen::Matrix3d solveDARE(const Eigen::Matrix3d& A, 
                               const Eigen::Matrix<double, 3, 2>& B,
                               const Eigen::Matrix3d& Q, 
                               const Eigen::Matrix2d& R);

    /**
     * @brief 计算四转四驱的横向速度分量
     */
    double computeLateralVelocity(double lateral_error, 
                                   double heading_error,
                                   double max_vy) const;

    BezierLQRParams params_;  ///< LQR参数
    ChassisType chassis_type_{ChassisType::kDifferential}; ///< 底盘类型
    Velocity last_velocity_;  ///< 上一次速度（用于平滑）
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_CONTROLLER_LQR_CONTROLLER_HPP_
