/**
 * @file types.hpp
 * @brief 统一类型定义
 * 
 * 定义导航模块中使用的所有基础数据类型
 * 支持差速底盘和四转四驱(4WIS4WID)底盘
 */

#ifndef LASER_NAVIGATION_REFACTORED_CORE_TYPES_HPP_
#define LASER_NAVIGATION_REFACTORED_CORE_TYPES_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace laser_navigation {

//==============================================================================
// 底盘类型定义
//==============================================================================

/**
 * @brief 底盘类型枚举
 * 
 * - Differential: 差速底盘，输出 (vx, ω)
 * - Swerve4WIS4WID: 四转四驱底盘，输出 (vx, vy, ω)
 */
enum class ChassisType {
    kDifferential,      ///< 差速底盘 (vx, ω)
    k4WIS4WID     ///< 四转四驱底盘 (vx, vy, ω)
};

//==============================================================================
// 基础几何类型
//==============================================================================

/**
 * @brief 2D位姿结构体
 */
struct Pose2D {
    double x{0.0};      ///< x坐标 (m)（必选）
    double y{0.0};      ///< y坐标 (m)（必选）
    double yaw{0.0};    ///< 航向角 (rad) （可选）
    std::string node_id;  ///< 节点ID（必选）

    //针对此对象是实时位姿时的附加信息
    std::string mode; ///< 只辨认实时位姿来源，可能是激光slam位姿 二维码位姿 轮式里程计位姿（如停车点等，可选）
    bool valid{true};  ///< 实时位姿是否有效（可选，默认有效）


    Pose2D() = default;
    Pose2D(double x_, double y_, double yaw_ = 0.0, const std::string& id = "")
        : x(x_), y(y_), yaw(yaw_), node_id(id) {}

    /**
     * @brief 转换为Eigen向量
     */
    Eigen::Vector2d toVector() const { return {x, y}; }
    
    /**
     * @brief 计算到另一点的距离
     */
    double distanceTo(const Pose2D& other) const {
        return std::hypot(x - other.x, y - other.y);
    }

    /**
     * @brief 计算到另一点的角度
     */
    double angleTo(const Pose2D& other) const {
        return std::atan2(other.y - y, other.x - x);
    }
};

/**
 * @brief 2D点结构体（简化版）
 */
struct Point2D {
    double x{0.0};
    double y{0.0};

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    Eigen::Vector2d toVector() const { return {x, y}; }
};

/**
 * @brief 速度指令结构体
 * 
 * 支持差速底盘 (vx, ω) 和四转四驱底盘 (vx, vy, ω)
 * - 差速底盘：仅使用 linear_x 和 angular
 * - 四转四驱：使用 linear_x, linear_y 和 angular
 */
struct Velocity {
    double linear_x{0.0};   ///< x方向线速度 (m/s)，车体前向
    double linear_y{0.0};   ///< y方向线速度 (m/s)，车体左向（仅四转四驱使用）
    double angular{0.0};    ///< 角速度 (rad/s)

    // 兼容旧接口的别名
    double& linear = linear_x;  ///< 兼容旧代码，等同于 linear_x

    Velocity() : linear_x(0.0), linear_y(0.0), angular(0.0), linear(linear_x) {}
    
    /**
     * @brief 差速底盘构造函数
     */
    Velocity(double vx, double w) 
        : linear_x(vx), linear_y(0.0), angular(w), linear(linear_x) {}
    
    /**
     * @brief 四转四驱底盘构造函数
     */
    Velocity(double vx, double vy, double w) 
        : linear_x(vx), linear_y(vy), angular(w), linear(linear_x) {}

    /**
     * @brief 拷贝构造函数
     */
    Velocity(const Velocity& other) 
        : linear_x(other.linear_x), linear_y(other.linear_y), 
          angular(other.angular), linear(linear_x) {}

    /**
     * @brief 赋值运算符
     */
    Velocity& operator=(const Velocity& other) {
        if (this != &other) {
            linear_x = other.linear_x;
            linear_y = other.linear_y;
            angular = other.angular;
        }
        return *this;
    }

    /**
     * @brief 判断速度是否接近零
     */
    bool isZero(double eps = 1e-6) const {
        return std::abs(linear_x) < eps && 
               std::abs(linear_y) < eps && 
               std::abs(angular) < eps;
    }

    /**
     * @brief 速度归零
     */
    void reset() {
        linear_x = 0.0;
        linear_y = 0.0;
        angular = 0.0;
    }

    /**
     * @brief 获取合成线速度大小
     */
    double linearMagnitude() const {
        return std::hypot(linear_x, linear_y);
    }

    /**
     * @brief 获取线速度方向（相对车体）
     */
    double linearDirection() const {
        return std::atan2(linear_y, linear_x);
    }
};

//==============================================================================
// 里程计反馈数据
//==============================================================================

/**
 * @brief 里程计反馈数据
 * 
 * 从里程计话题获取的车体当前状态信息
 * 可用于：
 * - 路径实时规划/重规划
 * - LQR跟踪控制的速度前馈
 * - 速度平滑过渡
 */
struct OdometryFeedback {
    // 位姿信息
    Pose2D pose;                    ///< 当前位姿
    
    // 速度信息
    Velocity velocity;              ///< 当前车体速度
    
    // 时间戳（用于判断数据有效性）
    double timestamp{0.0};          ///< 时间戳 (s)
    
    // 数据有效性
    bool is_valid{false};           ///< 数据是否有效
    double timeout{0.2};            ///< 超时阈值 (s)
    
    OdometryFeedback() = default;
    
    /**
     * @brief 检查数据是否过期
     * @param current_time 当前时间
     * @return true如果数据过期
     */
    bool isExpired(double current_time) const {
        return !is_valid || (current_time - timestamp) > timeout;
    }
    
    /**
     * @brief 更新数据
     */
    void update(const Pose2D& p, const Velocity& v, double t) {
        pose = p;
        velocity = v;
        timestamp = t;
        is_valid = true;
    }
    
    /**
     * @brief 使数据无效
     */
    void invalidate() {
        is_valid = false;
    }
};;

//==============================================================================
// 导航状态枚举
//==============================================================================

/**
 * @brief 导航执行状态 通用于导航内外层
 * 
 * 对应原代码中的返回值: 0=正常行驶, 1=前进中, 2=后退中, 3=停障, 4=旋转, 5=到点, 6=取消
 */
enum class NavigationStatus {
    kIdle = 0,           ///< 空闲
    kRunning = 1,        ///< 直线行驶中（前进）
    kReversing = 2,      ///< 倒车中
    kObstaclePaused = 3, ///< 障碍物暂停
    kRotating = 4,       ///< 原地旋转中
    kGoalReached = 5,    ///< 目标点到达
    kCancelled = 6,      ///< 已取消
    kSegmentReached = 7, ///< 阶段点到达
    kEmergencyStopped = 8, ///< 紧急停止
    kError = 9,           ///< 错误
    kTurnLeft = 10,        ///< 左转中
    kTurnRight= 11,         ///< 右转中
    kPaused = 12            ///< 暂停
};

/**
 * @brief 状态机状态
 */
enum class StateMachineState {
    kIdle,                    ///< 空闲
    kStartRotation,           ///< 起始旋转
    kFollowingPath,           ///< 路径跟踪中
    kEndRotation,             ///< 终点旋转
    kDecelerating,            ///< 减速停止中
    kPaused,                  ///< 暂停
    kCompleted,               ///< 完成
    kError                    ///< 错误
};

//==============================================================================
// 路径段类型
//==============================================================================

/**
 * @brief 路径段类型枚举
 * 
 * 对应原代码中的 Line::type: 0=直线, 1=圆弧, 2=三阶贝塞尔
 * 
 * 扩展设计：添加新轨迹类型只需要：
 * 1. 在此枚举中添加新类型
 * 2. 创建继承自 PathSegment 的新类
 * 3. 在 NavigationExecutor 中添加对应的处理逻辑
 */
enum class PathSegmentType {
    kStraight = 0,        ///< 直线
    kCircularArc = 1,     ///< 圆弧（简单圆弧）
    kCubicBezier = 2,     ///< 三阶贝塞尔曲线
    kDubins = 3,          ///< Dubins曲线（仅前进的最短路径）
    kReedsShepp = 4,      ///< Reeds-Shepp曲线（支持倒车）
    kClothoid = 5,        ///< 回旋曲线（曲率线性变化）
    kSpline = 6           ///< 样条曲线（预留）
};

//==============================================================================
// 运动约束参数
//==============================================================================

/**
 * @brief 运动约束参数
 * 
 * 对应原代码中的 Line 结构体中的速度/加速度参数
 * 支持差速和四转四驱底盘
 */
struct MotionConstraints {
    // 线速度约束（x方向，车体前向）
    double max_velocity{0.5};         ///< 最大线速度 (m/s) - 对应 Line::V
    double max_acceleration{0.2};     ///< 最大加速度 (m/s^2) - 对应 Line::acc
    double max_deceleration{0.2};     ///< 最大减速度 (m/s^2) - 对应 Line::dec
    double max_jerk{0.5};             ///< 最大加加速度 (m/s^3) - 对应 Line::jerk

    // 横向速度约束（y方向，仅四转四驱）
    double max_lateral_velocity{0.3};      ///< 最大横向速度 (m/s)
    double max_lateral_acceleration{0.2};  ///< 最大横向加速度 (m/s^2)
    double max_lateral_deceleration{0.2};  ///< 最大横向减速度 (m/s^2)

    // 角速度约束
    double max_angular_velocity{0.8};      ///< 最大角速度 (rad/s) - 对应 Line::W
    double max_angular_acceleration{0.8};  ///< 最大角加速度 (rad/s^2) - 对应 Line::acc_angle
    double max_angular_deceleration{0.8};  ///< 最大角减速度 (rad/s^2) - 对应 Line::dec_angle

    // 到达判定
    double reach_distance{0.01};      ///< 位置到达阈值 (m) - 对应 Line::reachDist
    double reach_angle{0.01};         ///< 角度到达阈值 (rad) - 对应 Line::reachAngle

    // 停障相关
    double obstacle_stop_distance{0.5};  ///< 停障区距离 (m)
    double obs_expansion{0.1};            ///< 障碍物宽度 (m)
    double obs_stop_deceleration{0.5};   ///< 停障减速度 (m/s^2) obsStopDec
    double obs_emergency_stop_distance{0.3}; ///< 急停区半径 (m)
    double obs_emergency_stop_deceleration{1.0}; ///< 急停减速度 (m/s^2) 主要用于避障急停区的急停

    //非避障急停
    double emer_immediately_stop_deceleration{10.0}; ///< 非避障急停减速度 (m/s^2)
    // 行驶方向
    bool is_forward{true};            ///< true=前进, false=后退 - 对应 Line::positive

    // 底盘类型
    ChassisType chassis_type{ChassisType::kDifferential}; ///< 底盘类型

    MotionConstraints() = default;
    static MotionConstraints& getInstance() {
        static MotionConstraints instance;
        return instance;
    }
};

//==============================================================================
// 控制器参数
//==============================================================================

/**
 * @brief LQR控制器参数（直线路径）
 * 
 * 对应原代码中的 common.h 里的LQR相关参数
 */
struct LQRParams {
    double q1{10.0};    ///< x误差权重
    double q2{10.0};    ///< y误差权重
    double q3{10.0};    ///< 角度误差权重
    double r1{1.0};     ///< 线速度控制权重
    double r2{1.0};     ///< 角速度控制权重
    double dt{0.04};    ///< 控制周期 (s) - 对应 Ts
    int max_iterations{10};  ///< Riccati迭代次数

    LQRParams() = default;
};

/**
 * @brief 贝塞尔路径LQR参数
 * 
 * 贝塞尔曲线跟踪需要不同的权重配置
 */
struct BezierLQRParams {
    double q1{0.5};     ///< x误差权重
    double q2{0.5};     ///< y误差权重
    double q3{0.005};   ///< 角度误差权重
    double r1{1.0};     ///< 线速度控制权重
    double r2{1.0};     ///< 角速度控制权重
    double dt{0.04};    ///< 控制周期

    BezierLQRParams() = default;
};

//==============================================================================
// S曲线规划参数
//==============================================================================

/**
 * @brief S曲线规划参数
 * 
 * 对应原代码中 S_para 结构体
 */
struct SCurveParams {
    double v_max{0.0};           ///< 最大速度 - 对应 S_para::Vmax
    double v_acc_end{0.0};       ///< 加速段结束时的速度 - 对应 S_para::Vacc_end
    double v_dec_start{0.0};     ///< 减速段开始时的速度 - 对应 S_para::Vdec_start
    double s_deceleration{0.0};  ///< 开始减速的剩余距离 - 对应 S_para::Sd

    // 时间参数
    double t1{0.0};  ///< 加加速段时间
    double t2{0.0};  ///< 匀加速段时间
    double t3{0.0};  ///< 减加速段时间
    double t4{0.0};  ///< 匀速段时间
    double t5{0.0};  ///< 加减速段时间
    double t6{0.0};  ///< 匀减速段时间
    double t7{0.0};  ///< 减减速段时间

    SCurveParams() = default;
};

//==============================================================================
// 导航控制输出
//==============================================================================

/**
 * @brief 恢复策略枚举
 * 
 * 当急停/障碍物解除后的恢复策略
 */
enum class RecoveryStrategy {
    kContinue,          ///< 继续原路径
    kReplan,            ///< 重新规划当前段
    kReturnToPath,      ///< 返回最近路径点
    kAbort              ///< 中止导航
};

/**
 * @brief 停止原因枚举
 */
enum class StopReason {
    kNone,              ///< 无停止
    kObstacle,          ///< 障碍物停止
    kEmergencyStop,     ///< 急停按钮
    kManualPause,       ///< 手动暂停//可恢复导航任务
    kManualStop,        ///< 手动停止 //要清空导航任务
    kError              ///< 错误停止
};

/**
 * @brief 位置偏移检测结果
 */
struct PositionDeviation {
    double distance{0.0};           ///< 位置偏移距离 (m)
    double angle{0.0};              ///< 角度偏移 (rad)
    bool needs_replan{false};       ///< 是否需要重规划
    RecoveryStrategy strategy{RecoveryStrategy::kContinue}; ///< 建议的恢复策略
    
    PositionDeviation() = default;
};

/**
 * @brief 导航输出结果
 * 
 * 对应原代码中 cubicCurve 函数的返回值
 * 输出车体速度 (vx, vy, ω)：
 * - 差速底盘：vy = 0
 * - 四转四驱：vy 可能非零（斜行、横移等）
 */
struct NavigationOutput {
    Velocity velocity;              ///< 速度指令 (vx, vy, ω)
    Velocity reference_velocity;    ///< 参考速度（规划值）
    NavigationStatus status;        ///< 导航状态
    ChassisType chassis_type;       ///< 底盘类型

    // 调试信息
    double lateral_error{0.0};      ///< 横向误差
    double heading_error{0.0};      ///< 航向误差
    double remaining_distance{0.0}; ///< 剩余距离
    int current_segment{0};         ///< 当前段索引

    // 里程计反馈使用状态
    bool using_odom_feedback{false}; ///< 是否使用了里程计反馈
    
    // 恢复相关
    bool is_recovering{false};       ///< 是否正在恢复中
    RecoveryStrategy recovery_strategy{RecoveryStrategy::kContinue}; ///< 恢复策略

    NavigationOutput()
        : status(NavigationStatus::kIdle),
          chassis_type(ChassisType::kDifferential) {}
};

//==============================================================================
// 路径点信息（用于路径热更新）
//==============================================================================

/**
 * @brief 路径点信息
 * 
 * 对应原代码中 Line 结构体的路径点部分
 */
struct WaypointInfo {
    Pose2D pose;                    ///< 位姿
    //? 约束和段类型原本v原本由道路属性决定，难道是为了热更新方便，放在这里
    MotionConstraints constraints;  ///< 运动约束 
    PathSegmentType segment_type{PathSegmentType::kStraight}; ///< 段类型
    
    // 贝塞尔曲线控制点（仅当segment_type为kCubicBezier时有效）
    std::vector<Pose2D> control_points;
    
    WaypointInfo() = default;
    WaypointInfo(const Pose2D& p, const MotionConstraints& c, 
                 PathSegmentType type = PathSegmentType::kStraight)
        : pose(p), constraints(c), segment_type(type) {}
};

/**
 * @brief 导航避障、取消、急停、暂停、停止状态信息
 */
struct InfoNavStatus{
    enum class obstacleArea{
        NORMAL_REGION = 0,
        DEC_STOP_REGION = 1,
        EMER_STOP_REGION =2
    };
    obstacleArea obstacle_detected_{obstacleArea::NORMAL_REGION};//检测到避障，要减速停下，障碍物移除可以自动恢复导航
    bool cancel_requested_{false}; //导航时取消任务，小车在前方最近站点停止，然后清空导航任务 //应当是一次性请求
    bool emergency_immediately_stop_{false};//紧急停止，快速停下，默认减速度为10.0，急停取消可以自动恢复导航 //应当是一次性请求
    bool taskPaused_{false}; // 导航时暂停任务，依据taskResumed_来恢复导航任务 //应当是一次性请求
    bool taskStopped_{false};//最大减速度减速停止并清空导航任务 //应当是一次性请求
    bool taskResumed_{false};// 恢复导航 //应当是一次性请求
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_CORE_TYPES_HPP_
