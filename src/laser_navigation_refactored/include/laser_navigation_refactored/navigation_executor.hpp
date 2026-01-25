/**
 * @file navigation_executor.hpp
 * @brief 导航执行器
 * 
 * 核心导航控制类，整合路径规划、速度规划、轨迹跟踪功能
 * 对应原代码 correct.h/correct.cpp 的重构
 * 
 * 支持：
 * - 差速底盘：输出 (vx, ω)
 * - 四转四驱底盘：输出 (vx, vy, ω)
 * - 可选的里程计速度反馈（用于规划和控制优化）
 */

#ifndef LASER_NAVIGATION_REFACTORED_NAVIGATION_EXECUTOR_HPP_
#define LASER_NAVIGATION_REFACTORED_NAVIGATION_EXECUTOR_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include "laser_navigation_refactored/path/path_segment.hpp"
#include "laser_navigation_refactored/path/straight_segment.hpp"
#include "laser_navigation_refactored/path/bezier_segment.hpp"
#include "laser_navigation_refactored/path/arc_segment.hpp"
#include "laser_navigation_refactored/planner/s_curve_planner.hpp"
#include "laser_navigation_refactored/controller/lqr_controller.hpp"
#include "laser_navigation_refactored/state_machine/navigation_state_machine.hpp"
#include "laser_navigation_refactored/core/optional.hpp"

#include <vector>
#include <memory>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <atomic>

namespace laser_navigation {

/**
 * @brief 导航输入配置
 * 
 * 对应原代码中 configHelper 解析的配置
 */
struct NavigationConfig {
    std::vector<Pose2D> waypoints;              ///< 路径点
    std::vector<MotionConstraints> constraints; ///< 每段的运动约束
    std::vector<PathSegmentType> segment_types; ///< 每段的类型
    std::vector<std::vector<Pose2D>> control_points; ///< 贝塞尔控制点（每段）
    
    bool adjust_start_angle{true};   ///< 是否调整起始角度
    bool adjust_end_angle{true};     ///< 是否调整终点角度
    
    /// 贝塞尔控制点自动生成选项
    bool auto_generate_control_points{true};   ///< 是否自动生成缺失的控制点
    double control_point_extension_factor{0.35}; ///< 控制点延伸系数 (0.1~0.6)
    
    // 底盘配置
    ChassisType chassis_type{ChassisType::kDifferential}; ///< 底盘类型
};

/**
 * @brief 导航执行器
 * 
 * 核心导航控制类，对应原代码 Correct 类的重构
 * 
 * 职责：
 * - 管理路径段
 * - 协调速度规划和轨迹跟踪
 * - 处理障碍物停止和恢复
 * - 管理导航状态
 * 
 * 底盘支持：
 * - 差速底盘 (Differential): 输出 (vx, ω)
 * - 四转四驱 (4WIS4WID): 输出 (vx, vy, ω)
 * 
 * 里程计反馈（可选）：
 * - 如果提供有效的里程计反馈，将用于优化速度规划和LQR跟踪
 * - 如果未提供或数据无效，则使用内部估计值
 */
class NavigationExecutor {
public:
    /// 日志回调函数类型
    using LogCallback = std::function<void(const std::string&)>;

    NavigationExecutor();
    ~NavigationExecutor() = default;

    //==========================================================================
    // 初始化和配置
    //==========================================================================

    /**
     * @brief 初始化导航
     * @param config 导航配置（包含底盘类型）
     * @param lqr_params 直线LQR参数
     * @param bezier_lqr_params 贝塞尔LQR参数
     * @return 0成功，其他失败
     * 
     * 对应原代码 Correct::init 函数
     */
    int initialize(const NavigationConfig& config,
                   const LQRParams& lqr_params,
                   const BezierLQRParams& bezier_lqr_params);

    /**
     * @brief 设置底盘类型
     * @param type 底盘类型
     */
    void setChassisType(ChassisType type);

    /**
     * @brief 获取底盘类型
     */
    ChassisType getChassisType() const { return chassis_type_; }

    /**
     * @brief 热更新路径（追加路径点）
     * @param additional_waypoints 新增的路径点
     * @param additional_constraints 新增的约束
     * @param additional_types 新增的段类型
     * @param additional_control_points 新增的控制点
     * @return 0成功，其他失败
     * 
     * 对应原代码 Correct::updateLine 函数
     * 
     * 热更新时会自动判断新旧路径衔接点：
     * - 如果旧终点到新路径方向转角小于阈值且行驶方向相同，则平滑过渡（不停止）
     * - 否则需要在旧终点停止后再转向
     */
    int update(const std::vector<Pose2D>& additional_waypoints,
               const std::vector<MotionConstraints>& additional_constraints,
               const std::vector<PathSegmentType>& additional_types,
               const std::vector<std::vector<Pose2D>>& additional_control_points);

    //==========================================================================
    // 停止点/过渡点配置和查询
    //==========================================================================

    /**
     * @brief 设置过渡点转向阈值
     * @param threshold 转向角度阈值（弧度），超过此角度需要停止转向
     * 
     * 默认值为 0.1 弧度（约5.7°）
     * 原始 laser_navigation_package 使用 0.05 弧度（约2.9°）用于直线拼接
     */
    void setTransitionAngleThreshold(double threshold);

    /**
     * @brief 获取当前过渡点转向阈值
     */
    double getTransitionAngleThreshold() const { return transition_angle_threshold_; }

    /**
     * @brief 获取停止点列表
     * @return 所有需要停止的点
     */
    const std::vector<Pose2D>& getStopPoints() const { return stop_points_; }

    /**
     * @brief 获取停止点标记
     * @return 每个路径点是否为停止点的标记
     */
    const std::vector<bool>& getStopPointFlags() const { return is_stop_point_; }

    /**
     * @brief 判断指定索引的点是否为停止点
     * @param waypoint_index 路径点索引
     * @return 是否为停止点
     */
    bool isStopPoint(size_t waypoint_index) const;

    /**
     * @brief 获取停止点数量
     */
    size_t getStopPointCount() const { return stop_points_.size(); }

    /**
     * @brief 获取路径段数量
     */
    size_t getSegmentCount() const { return path_segments_.size(); }

    /**
     * @brief 获取路径点数量
     */
    size_t getWaypointCount() const { return waypoints_.size(); }

    //==========================================================================
    // 里程计反馈接口
    //==========================================================================

    /**
     * @brief 更新里程计反馈数据
     * @param pose 当前位姿
     * @param velocity 当前车体速度 (vx, vy, ω)
     * @param timestamp 时间戳 (s)
     * 
     * 调用此函数提供里程计反馈，用于：
     * - 速度规划时考虑当前实际速度（平滑过渡）
     * - LQR跟踪时作为速度前馈参考
     * - 路径重规划的参考依据
     */
    void updateOdometryFeedback(const Pose2D& pose, 
                                 const Velocity& velocity,
                                 double timestamp);

    /**
     * @brief 使里程计反馈无效
     * 
     * 如果里程计话题断开或数据不可信，调用此函数
     */
    void invalidateOdometryFeedback();

    /**
     * @brief 检查里程计反馈是否可用
     * @param current_time 当前时间
     */
    bool isOdometryFeedbackValid(double current_time) const;

    /**
     * @brief 设置里程计超时时间
     * @param timeout 超时阈值 (s)
     */
    void setOdometryTimeout(double timeout);

    //==========================================================================
    // 急停和恢复接口
    //==========================================================================

    /**
     * @brief 触发急停
     * 
     * 急停时会记录当前位姿，用于恢复时判断是否需要重规划
     */
    void emergencyStop();

    /**
     * @brief 从急停/障碍物停止中恢复
     * @param current_pose 当前位姿
     * @return 恢复策略
     * 
     * 会检测停止期间的位置偏移，决定恢复策略：
     * - 偏移较小：继续原路径，平滑恢复速度
     * - 偏移较大：重新规划当前段
     * - 偏移过大或偏离路径：返回最近路径点或中止
     */
    RecoveryStrategy resumeFromStop(const Pose2D& current_pose);

    /**
     * @brief 设置位置偏移阈值
     * @param replan_distance 触发重规划的距离阈值 (m)
     * @param replan_angle 触发重规划的角度阈值 (rad)
     * @param abort_distance 触发中止的距离阈值 (m)
     */
    void setDeviationThresholds(double replan_distance, double replan_angle, 
                                 double abort_distance);

    /**
     * @brief 设置实时偏离检测参数
     * @param enable 是否启用实时偏离检测
     * @param threshold 偏离阈值 (m)
     */
    void setRealtimeDeviationCheck(bool enable, double threshold = 0.15);

    /**
     * @brief 检测位置偏移
     * @param current_pose 当前位姿
     * @return 位置偏移检测结果
     */
    PositionDeviation checkPositionDeviation(const Pose2D& current_pose) const;

    /**
     * @brief 重新规划当前路径段
     * @param current_pose 当前位姿
     * @return 0成功，其他失败
     */
    int replanCurrentSegment(const Pose2D& current_pose);

    //==========================================================================
    // 执行控制
    //==========================================================================

    /**
     * @brief 执行一个控制周期（不使用里程计反馈）
     * @param current_pose 当前位姿
     * @param obstacle_stop 是否因障碍getPlannedVelocity物停止
     * @param obstacle_deceleration 障碍物停止时的减速度
     * @param cancel 是否取消导航
     * @return 导航输出（速度指令和状态）
     * 
     * 对应原代码 Correct::cubicCurve 函数
     */
    NavigationOutput execute(const Pose2D& current_pose,
                             bool obstacle_stop,
                             double obstacle_deceleration,
                             bool cancel);

    /**
     * @brief 执行一个控制周期（使用里程计反馈）
     * @param current_pose 当前位姿
     * @param obstacle_stop 是否因障碍物停止
     * @param obstacle_deceleration 障碍物停止时的减速度
     * @param cancel 是否取消导航
     * @param current_time 当前时间（用于检查里程计有效性）
     * @return 导航输出（速度指令和状态）
     * 
     * 此版本会自动使用之前通过 updateOdometryFeedback 提供的里程计数据
     */
    NavigationOutput executeWithOdometry(const Pose2D& current_pose,
                                          bool obstacle_stop,
                                          double obstacle_deceleration,
                                          bool cancel,
                                          double current_time);

    /**
     * @brief 取消导航
     */
    void cancel();

    /**
     * @brief 停止导航（带减速）
     * @param max_linear_decel 最大线减速度
     * @param max_angular_decel 最大角减速度
     */
    void stop(double max_linear_decel, double max_angular_decel);

    /**
     * @brief 减速停止
     * @param linear_decel 线减速度
     * @param angular_decel 角减速度
     * @return 当前速度指令
     * 
     * 对应原代码 Correct::decelStop 函数
     */
    Velocity decelerateStop(double linear_decel, double angular_decel);

    //==========================================================================
    // 状态查询
    //==========================================================================

    /**
     * @brief 获取当前状态机状态
     */
    StateMachineState getState() const { return state_machine_.getCurrentState(); }

    /**
     * @brief 获取当前段索引
     */
    int getCurrentSegmentIndex() const { return current_segment_index_; }

    /**
     * @brief 获取总段数
     */
    int getTotalSegments() const { return static_cast<int>(path_segments_.size()); }

    /**
     * @brief 检查是否正在导航
     */
    bool isNavigating() const { return state_machine_.isActive(); }

    /**
     * @brief 检查是否已完成
     */
    bool isCompleted() const { return state_machine_.isCompleted(); }

    /**
     * @brief 获取当前规划速度
     */
    Velocity getPlannedVelocity() const { return last_planned_velocity_; }

    /**
     * @brief 获取当前控制速度
     */
    Velocity getControlVelocity() const { return last_control_velocity_; }

    /**
     * @brief 获取里程计反馈数据
     */
    const OdometryFeedback& getOdometryFeedback() const { return odom_feedback_; }

    //==========================================================================
    // 回调设置
    //==========================================================================

    /**
     * @brief 设置日志回调
     */
    void setLogCallback(LogCallback callback) { log_callback_ = callback; }

private:
    //==========================================================================
    // 内部处理函数
    //==========================================================================

    /**
     * @brief 构建路径段
     * @param config 配置
     */
    void buildPathSegments(const NavigationConfig& config);

    /**
     * @brief 计算停止点（用于路径拼接优化）
     * 
     * 对应原代码中判断哪些点需要停止的逻辑
     * 根据转向角度和行驶方向判断过渡点是否需要停止：
     * - 转向角度 > transition_angle_threshold_ 需要停止
     * - 行驶方向改变（前进/后退切换）需要停止
     * - 直线段接贝塞尔段或反之，根据切线方向判断
     */
    void computeStopPoints();

    /**
     * @brief 判断两段路径是否可以平滑过渡（不停止）
     * @param current_seg 当前段
     * @param next_seg 下一段
     * @return true可以平滑过渡，false需要停止
     */
    bool canSmoothTransition(const PathSegmentPtr& current_seg, 
                              const PathSegmentPtr& next_seg) const;

    /**
     * @brief 处理起始旋转
     */
    NavigationOutput handleStartRotation(const Pose2D& current_pose, 
                                          bool obstacle_stop,
                                          double obstacle_decel);

    /**
     * @brief 处理路径跟踪
     */
    NavigationOutput handlePathFollowing(const Pose2D& current_pose, 
                                          bool obstacle_stop,
                                          double obstacle_decel, 
                                          bool cancel);

    /**
     * @brief 处理终点旋转
     */
    NavigationOutput handleEndRotation(const Pose2D& current_pose, 
                                        bool obstacle_stop,
                                        double obstacle_decel);

    /**
     * @brief 处理直线段跟踪
     */
    NavigationOutput trackStraightSegment(const Pose2D& current_pose, 
                                           StraightSegment* segment,
                                           double remaining_distance,
                                           bool obstacle_stop,
                                           double obstacle_decel,
                                           const Optional<OdometryFeedback>& odom = nullopt);

    /**
     * @brief 处理贝塞尔曲线段跟踪
     */
    NavigationOutput trackBezierSegment(const Pose2D& current_pose,
                                         BezierSegment* segment,
                                         double remaining_distance,
                                         const Optional<OdometryFeedback>& odom = nullopt);

    /**
     * @brief 执行原地旋转
     */
    NavigationOutput executeRotation(const Pose2D& current_pose, 
                                      double target_angle,
                                      bool obstacle_stop, 
                                      double obstacle_decel,
                                      const Optional<OdometryFeedback>& odom = nullopt);

    /**
     * @brief 切换到下一段
     * @return 是否还有下一段
     */
    bool advanceToNextSegment();

    /**
     * @brief 更新目标航向
     */
    void updateTargetHeading();

    /**
     * @brief 检查是否需要原地旋转
     */
    bool needsRotation(double current_angle, double target_angle, double threshold) const;

    /**
     * @brief 处理角度（考虑倒车）
     */
    double adjustAngleForDirection(double angle, bool is_forward) const;

    /**
     * @brief 输出日志
     */
    void log(const std::string& message);

    //==========================================================================
    // 成员变量
    //==========================================================================

    // 状态机
    NavigationStateMachine state_machine_;

    // 路径段
    std::vector<PathSegmentPtr> path_segments_;
    int current_segment_index_{0};
    int current_waypoint_index_{0};

    // 路径点信息
    std::vector<Pose2D> waypoints_;
    std::vector<Pose2D> stop_points_;    ///< 实际停止点
    std::vector<bool> is_stop_point_;    ///< 标记是否为停止点

    // 规划器
    SCurvePlanner linear_planner_;       ///< 直线速度规划器
    SCurvePlanner rotation_planner_;     ///< 旋转速度规划器

    // 控制器
    StraightLQRController straight_controller_;
    BezierLQRController bezier_controller_;

    // 贝塞尔跟踪状态（对应原代码中的 S_plan_bezier, t 等）
    double bezier_arc_length_{0.0};      ///< 当前弧长
    double bezier_time_{0.0};            ///< 时间参数
    Eigen::Vector2d bezier_ref_point_;   ///< 参考点
    double bezier_ref_heading_{0.0};     ///< 参考航向

    // 速度状态
    Velocity last_planned_velocity_;     ///< 上一周期规划速度
    Velocity last_control_velocity_;     ///< 上一周期控制速度

    // 目标状态
    double target_heading_{0.0};         ///< 当前目标航向
    double end_heading_{0.0};            ///< 终点目标航向

    // 配置参数
    bool adjust_start_angle_{true};
    bool adjust_end_angle_{true};
    double control_period_{0.04};        ///< 控制周期
    ChassisType chassis_type_{ChassisType::kDifferential}; ///< 底盘类型
    double transition_angle_threshold_{0.1}; ///< 过渡点转向阈值 (rad)

    // 里程计反馈
    OdometryFeedback odom_feedback_;     ///< 里程计反馈数据
    std::atomic<bool> use_odom_feedback_{false};      ///< 本周期是否使用里程计反馈 (线程安全)
    mutable std::shared_mutex path_mutex_;  ///< 保护路径/状态数据（读写锁）
    mutable std::mutex odom_mutex_;        ///< 保护里程计数据的互斥量

    // 急停和恢复相关
    bool is_stopped_{false};             ///< 是否处于停止状态
    StopReason stop_reason_{StopReason::kNone}; ///< 停止原因
    Pose2D stop_pose_;                   ///< 停止时的位姿
    double stop_velocity_{0.0};          ///< 停止时的速度（用于平滑恢复）
    double recovery_velocity_{0.0};      ///< 恢复过程中的当前速度
    bool is_recovering_{false};          ///< 是否正在恢复中
    RecoveryStrategy current_recovery_strategy_{RecoveryStrategy::kContinue}; ///< 当前恢复策略

    // 位置偏移阈值updateTargetHeading
    double replan_distance_threshold_{0.1};  ///< 触发重规划的距离阈值 (m)
    double replan_angle_threshold_{0.26};    ///< 触发重规划的角度阈值 (rad, ~15°)
    double abort_distance_threshold_{0.5};   ///< 触发中止的距离阈值 (m)

    // 实时偏离检测参数
    bool enable_realtime_deviation_check_{true};   ///< 是否启用实时偏离检测
    double realtime_deviation_threshold_{0.15};    ///< 实时偏离阈值 (m)

    // 恢复加速参数
    double recovery_acceleration_{0.3};  ///< 恢复时的加速度 (m/s²)

    // 标志位
    bool is_initialized_{false};
    bool is_cancelled_{false};
    bool is_rotating_{false};
    bool last_obstacle_stop_{false};

    // 回调
    LogCallback log_callback_;

    //==========================================================================
    // 内部辅助函数
    //==========================================================================

    /**
     * @brief 根据底盘类型调整输出速度
     * @param velocity 原始速度
     * @return 调整后的速度
     * 
     * 对于差速底盘，将 vy 置零
     * 对于四转四驱，保持 vy 值
     */
    Velocity adjustVelocityForChassis(const Velocity& velocity) const;

    /**
     * @brief 使用里程计反馈平滑速度过渡
     * @param target_velocity 目标速度
     * @param current_velocity 当前实际速度（来自里程计）
     * @param max_acceleration 最大加速度
     * @param dt 时间步长
     * @return 平滑后的速度
     */
    Velocity smoothVelocityTransition(const Velocity& target_velocity,
                                       const Velocity& current_velocity,
                                       double max_acceleration,
                                       double dt) const;

    /**
     * @brief 获取可选的里程计反馈
     * @param current_time 当前时间
     * @return 如果有效返回里程计反馈，否则返回 nullopt
     */
    Optional<OdometryFeedback> getValidOdometryFeedback(double current_time) const;
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_NAVIGATION_EXECUTOR_HPP_
