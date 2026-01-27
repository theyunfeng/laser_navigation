/**
 * @file navigation_executor.cpp
 * @brief 导航执行器实现
 * 
 * 对应原代码 correct.cpp 的重构
 * 支持差速和四转四驱底盘
 * 支持可选的里程计速度反馈
 */

#include "laser_navigation_refactored/navigation_executor.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include "laser_navigation_refactored/utils/bezier_control_point_generator.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace laser_navigation {

NavigationExecutor::NavigationExecutor() {
    // 设置状态机回调
    state_machine_.setStateChangeCallback(
        [this](StateMachineState old_state, StateMachineState new_state) {
            std::stringstream ss;
            ss << "State transition: " 
               << NavigationStateMachine::getStateName(old_state)
               << " -> " 
               << NavigationStateMachine::getStateName(new_state);
            log(ss.str());
        });
}

void NavigationExecutor::log(const std::string& message) {
    if (log_callback_) {
        log_callback_(message);
    }
}

void NavigationExecutor::setChassisType(ChassisType type) {
    chassis_type_ = type;
    straight_controller_.setChassisType(type);
    bezier_controller_.setChassisType(type);
    
    std::string type_name = (type == ChassisType::kDifferential) ? 
        "Differential" : "Swerve4WIS4WID";
    log("Chassis type set to: " + type_name);
}

int NavigationExecutor::initialize(const NavigationConfig& config,
                                   const LQRParams& lqr_params,
                                   const BezierLQRParams& bezier_lqr_params) {
    std::unique_lock<std::shared_mutex> path_lock(path_mutex_);
    // 验证输入
    if (config.waypoints.size() < 2) {
        log("Error: At least 2 waypoints required");
        return -1;
    }

    // 保存配置
    waypoints_.clear();
    waypoints_ = config.waypoints;
    adjust_start_angle_ = config.adjust_start_angle;
    adjust_end_angle_ = config.adjust_end_angle;
    control_period_ = lqr_params.dt;
    chassis_type_ = config.chassis_type;

    // 构建路径段
    buildPathSegments(config);

    // 计算停止点
    computeStopPoints();

    // 初始化控制器
    straight_controller_.setParams(lqr_params);
    straight_controller_.setChassisType(chassis_type_);
    bezier_controller_.setParams(bezier_lqr_params);
    bezier_controller_.setChassisType(chassis_type_);

    // 初始化规划器（第一段）
    if (!path_segments_.empty()) {
        const auto& first_segment = path_segments_[0];
        if (first_segment->getType() == PathSegmentType::kStraight) {
            linear_planner_.initializeLinear(
                first_segment->getTotalLength(), 0.0,
                first_segment->getConstraints());
        }
    }

    // 重置状态
    current_segment_index_ = 0;
    current_waypoint_index_ = 0;
    is_cancelled_ = false;
    is_rotating_ = false;
    last_planned_velocity_.reset();
    last_control_velocity_.reset();
    odom_feedback_.invalidate();
    use_odom_feedback_.store(false);

    // 更新目标航向
    updateTargetHeading();

    // 设置状态机
    state_machine_.reset();
    state_machine_.setNeedStartRotation(adjust_start_angle_);
    state_machine_.setNeedEndRotation(adjust_end_angle_);

    is_initialized_ = true;
    state_machine_.processEvent(StateMachineEvent::kStart);

    std::stringstream ss;
    ss << "Navigation initialized with " << path_segments_.size() << " segments"
       << ", chassis: " << (chassis_type_ == ChassisType::kDifferential ? 
           "Differential" : "Swerve4WIS4WID");
    log(ss.str());

    return 0;
}

void NavigationExecutor::buildPathSegments(const NavigationConfig& config) {
    path_segments_.clear();

    // 如果启用自动生成控制点，先生成缺失的控制点
    std::vector<std::vector<Pose2D>> control_points = config.control_points;
    
    if (config.auto_generate_control_points) {//TODO 这里可以考虑加入是否是bezier的判断
        BezierControlPointGenerator generator;
        generator.setExtensionFactor(config.control_point_extension_factor);
        
        control_points = generator.generateAllControlPoints(
            config.waypoints,
            config.segment_types,
            config.constraints,
            config.control_points);
        
        // 记录自动生成日志
        for (size_t i = 0; i < config.segment_types.size(); ++i) {
            if (config.segment_types[i] == PathSegmentType::kCubicBezier) {
                bool was_auto_generated = 
                    (i >= config.control_points.size() || 
                     config.control_points[i].size() < 2);
                if (was_auto_generated && i < control_points.size() && 
                    control_points[i].size() >= 2) {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(3)
                       << "Auto-generated control points for segment " << i
                       << ": P1=(" << control_points[i][0].x << ", " 
                       << control_points[i][0].y << ")"
                       << ", P2=(" << control_points[i][1].x << ", " 
                       << control_points[i][1].y << ")";
                    log(ss.str());
                }
            }
        }
    }

    for (size_t i = 0; i < config.waypoints.size() - 1; ++i) {
        const auto& start = config.waypoints[i];
        const auto& end = config.waypoints[i + 1];
        
        // 获取约束（如果没有指定，使用默认值）
        const auto& constraints = (i < config.constraints.size()) ? 
            config.constraints[i] : MotionConstraints();
        
        // 获取段类型
        const auto segment_type = (i < config.segment_types.size()) ? 
            config.segment_types[i] : PathSegmentType::kStraight;

        PathSegmentPtr segment;
        
        if (segment_type == PathSegmentType::kCubicBezier && 
            i < control_points.size() && 
            control_points[i].size() >= 2) {
            // 创建贝塞尔曲线段
            segment = std::make_shared<BezierSegment>(
                start, end,
                control_points[i][0],
                control_points[i][1],
                constraints);
        } else if (segment_type == PathSegmentType::kCubicBezier) {
            // 贝塞尔段但没有控制点（自动生成也失败了）
            // 回退到直线段
            log("Warning: Bezier segment " + std::to_string(i) + 
                " has no control points, falling back to straight line");
            segment = std::make_shared<StraightSegment>(start, end, constraints);
        } else if (segment_type == PathSegmentType::kCircularArc) {
            // 创建圆弧段
            std::unique_ptr<ArcSegment> arc_segment;
            
            if (i < control_points.size() && !control_points[i].empty()) {
                const auto& param = control_points[i][0];
                
                if (param.yaw >= 0) {
                    // yaw >= 0 表示这是控制点（中间点），使用三点确定圆弧
                    arc_segment = ArcSegment::createFromThreePoints(start, param, end);
                    if (arc_segment) {
                        log("Created circular arc from 3 points for segment " + std::to_string(i));
                    }
                } else {
                    // yaw < 0 表示这是曲率参数
                    double curvature = param.x;
                    arc_segment = std::unique_ptr<ArcSegment>(
                        new ArcSegment(start, end, curvature));
                    log("Created circular arc with curvature " + 
                        std::to_string(curvature) + " for segment " + std::to_string(i));
                }
            }
            
            if (!arc_segment) {
                // 默认曲率
                double default_curvature = 0.2;
                arc_segment = std::unique_ptr<ArcSegment>(
                    new ArcSegment(start, end, default_curvature));
                log("Created circular arc with default curvature for segment " + std::to_string(i));
            }
            
            arc_segment->setConstraints(constraints);
            segment = std::shared_ptr<ArcSegment>(arc_segment.release());
            
        } else if (segment_type == PathSegmentType::kDubins ||
                   segment_type == PathSegmentType::kReedsShepp) {
            // 创建Dubins/Reeds-Shepp段
            std::unique_ptr<ArcSegment> arc_segment;
            
            if (i < control_points.size() && !control_points[i].empty()) {
                const auto& param = control_points[i][0];
                
                if (param.yaw >= 0) {
                    // yaw >= 0 表示这是控制点（中间点），使用三点确定圆弧
                    arc_segment = ArcSegment::createFromThreePoints(start, param, end);
                    if (arc_segment) {
                        std::string type_name = (segment_type == PathSegmentType::kDubins) ? 
                            "Dubins" : "Reeds-Shepp";
                        log("Created " + type_name + " path from 3 points for segment " + std::to_string(i));
                    }
                } else {
                    // yaw < 0 表示这是曲率参数
                    double curvature = param.x;
                    if (segment_type == PathSegmentType::kDubins) {
                        arc_segment = ArcSegment::createDubins(start, end, curvature);
                        if (arc_segment) {
                            log("Created Dubins path with curvature " + 
                                std::to_string(curvature) + " for segment " + std::to_string(i));
                        }
                    } else {
                        arc_segment = ArcSegment::createReedsShepp(start, end, curvature);
                        if (arc_segment) {
                            log("Created Reeds-Shepp path with curvature " + 
                                std::to_string(curvature) + " for segment " + std::to_string(i));
                        }
                    }
                }
            }
            
            // 没有参数或创建失败，使用默认曲率
            if (!arc_segment) {
                double default_curvature = 0.2;
                if (segment_type == PathSegmentType::kDubins) {
                    arc_segment = ArcSegment::createDubins(start, end, default_curvature);
                    log("Created Dubins path with default curvature for segment " + std::to_string(i));
                } else {
                    arc_segment = ArcSegment::createReedsShepp(start, end, default_curvature);
                    log("Created Reeds-Shepp path with default curvature for segment " + std::to_string(i));
                }
            }
            
            if (arc_segment) {
                arc_segment->setConstraints(constraints);
                segment = std::shared_ptr<ArcSegment>(arc_segment.release());
            } else {
                // 圆弧创建失败，回退到直线
                log("Warning: Arc segment " + std::to_string(i) + 
                    " creation failed, falling back to straight line");
                segment = std::make_shared<StraightSegment>(start, end, constraints);
            }
        } else {
            // 创建直线段
            segment = std::make_shared<StraightSegment>(start, end, constraints);
        }

        path_segments_.push_back(segment);
    }
}

void NavigationExecutor::computeStopPoints() {
    // 计算哪些点需要停止
    // 对应原代码中判断路径拼接的逻辑 (StraightConnectBezier, BezierConnectStraight)
    stop_points_.clear();
    is_stop_point_.clear();

    if (path_segments_.empty()) return;

    // 第一个点总是停止点
    stop_points_.push_back(path_segments_[0]->getStartPose());
    is_stop_point_.push_back(true);

    for (size_t i = 0; i < path_segments_.size() - 1; ++i) {
        const auto& current_seg = path_segments_[i];
        const auto& next_seg = path_segments_[i + 1];

        // 判断是否可以平滑过渡
        bool need_stop = !canSmoothTransition(current_seg, next_seg);

        is_stop_point_.push_back(need_stop);
        if (need_stop) {
            stop_points_.push_back(current_seg->getEndPose());
        }
    }

    // 最后一个点总是停止点
    stop_points_.push_back(path_segments_.back()->getEndPose());
    is_stop_point_.push_back(true);
    
    std::stringstream ss;
    ss << "Computed " << stop_points_.size() << " stop points from " 
       << path_segments_.size() << " segments";
    log(ss.str());
}

bool NavigationExecutor::canSmoothTransition(const PathSegmentPtr& current_seg,
                                              const PathSegmentPtr& next_seg) const {
    // 方向必须相同（前进/后退）
    bool same_direction = 
        current_seg->getConstraints().is_forward == 
        next_seg->getConstraints().is_forward;
    
    if (!same_direction) {
        return false;  // 方向改变必须停止
    }
    
    // 根据段类型计算衔接处的航向变化
    double heading_diff = 0.0;
    
    if (current_seg->getType() == PathSegmentType::kStraight &&
        next_seg->getType() == PathSegmentType::kStraight) {
        // 直线-直线衔接：比较两段航向
        heading_diff = math::MathUtils::absoluteAngleDiff(
            current_seg->getEndHeading(), next_seg->getStartHeading());
    } 
    else if (current_seg->getType() == PathSegmentType::kStraight &&
             next_seg->getType() == PathSegmentType::kCubicBezier) {
        // 直线-贝塞尔衔接：比较直线航向和贝塞尔起点切线方向
        auto* bezier_seg = dynamic_cast<BezierSegment*>(next_seg.get());
        if (bezier_seg) {
            // 获取贝塞尔曲线起点的切线方向
            double bezier_start_heading = bezier_seg->getStartHeading();
            heading_diff = math::MathUtils::absoluteAngleDiff(
                current_seg->getEndHeading(), bezier_start_heading);
        } else {
            heading_diff = M_PI;  // 转换失败，需要停止
        }
    }
    else if (current_seg->getType() == PathSegmentType::kCubicBezier &&
             next_seg->getType() == PathSegmentType::kStraight) {
        // 贝塞尔-直线衔接：比较贝塞尔终点切线方向和直线航向
        auto* bezier_seg = dynamic_cast<BezierSegment*>(current_seg.get());
        if (bezier_seg) {
            double bezier_end_heading = bezier_seg->getEndHeading();
            heading_diff = math::MathUtils::absoluteAngleDiff(
                bezier_end_heading, next_seg->getStartHeading());
        } else {
            heading_diff = M_PI;
        }
    }
    else if (current_seg->getType() == PathSegmentType::kCubicBezier &&
             next_seg->getType() == PathSegmentType::kCubicBezier) {
        // 贝塞尔-贝塞尔衔接
        heading_diff = math::MathUtils::absoluteAngleDiff(
            current_seg->getEndHeading(), next_seg->getStartHeading());
    }
    
    // 航向变化小于阈值才能平滑过渡
    return heading_diff < transition_angle_threshold_;
}

int NavigationExecutor::update(const std::vector<Pose2D>& additional_waypoints,
                               const std::vector<MotionConstraints>& additional_constraints,
                               const std::vector<PathSegmentType>& additional_types,
                               const std::vector<std::vector<Pose2D>>& additional_control_points) {
    // 热更新路径
    // 对应原代码 Correct::updateLine
    
    std::unique_lock<std::shared_mutex> path_lock(path_mutex_);

    if (additional_waypoints.empty()) {
        return 0;
    }
    
    // 记录热更新前的最后一段，用于判断衔接
    PathSegmentPtr last_segment_before_update = 
        path_segments_.empty() ? nullptr : path_segments_.back();
    size_t segments_before_update = path_segments_.size();

    // 追加路径点
    for (const auto& wp : additional_waypoints) {
        waypoints_.push_back(wp);
    }

    // 构建新的段
    for (size_t i = 0; i < additional_waypoints.size(); ++i) {
        if (i == 0 && !path_segments_.empty()) {
            // 连接现有路径的最后一点和新路径的第一点
            const auto& start = path_segments_.back()->getEndPose();
            const auto& end = additional_waypoints[i];
            const auto& constraints = (i < additional_constraints.size()) ?
                additional_constraints[i] : MotionConstraints();
            const auto segment_type = (i < additional_types.size()) ?
                additional_types[i] : PathSegmentType::kStraight;

            PathSegmentPtr segment;
            if (segment_type == PathSegmentType::kCubicBezier &&
                i < additional_control_points.size() &&
                additional_control_points[i].size() >= 2) {
                segment = std::make_shared<BezierSegment>(
                    start, end,
                    additional_control_points[i][0],
                    additional_control_points[i][1],
                    constraints);
            } else {
                segment = std::make_shared<StraightSegment>(start, end, constraints);
            }
            path_segments_.push_back(segment);
        }

        if (i + 1 < additional_waypoints.size()) {
            const auto& start = additional_waypoints[i];
            const auto& end = additional_waypoints[i + 1];
            const auto& constraints = (i + 1 < additional_constraints.size()) ?
                additional_constraints[i + 1] : MotionConstraints();
            const auto segment_type = (i + 1 < additional_types.size()) ?
                additional_types[i + 1] : PathSegmentType::kStraight;

            PathSegmentPtr segment;
            if (segment_type == PathSegmentType::kCubicBezier &&
                i + 1 < additional_control_points.size() &&
                additional_control_points[i + 1].size() >= 2) {
                segment = std::make_shared<BezierSegment>(
                    start, end,
                    additional_control_points[i + 1][0],
                    additional_control_points[i + 1][1],
                    constraints);
            } else {
                segment = std::make_shared<StraightSegment>(start, end, constraints);
            }
            path_segments_.push_back(segment);
        }
    }

    // 重新计算停止点
    computeStopPoints();
    
    // 检查新旧路径衔接点是否可以平滑过渡
    // 如果可以平滑过渡，需要移除旧终点的停止标记，实现速度连续
    if (last_segment_before_update && segments_before_update < path_segments_.size()) {
        // 获取衔接处的两段
        PathSegmentPtr connection_segment = path_segments_[segments_before_update];
        
        // 检查旧路径最后一段和连接段是否可以平滑过渡
        if (canSmoothTransition(last_segment_before_update, connection_segment)) {
            // 可以平滑过渡，更新 is_stop_point_ 标记
            // 旧终点（即新段的起点）不再是停止点
            if (segments_before_update < is_stop_point_.size()) {
                is_stop_point_[segments_before_update] = false;
                
                // 从 stop_points_ 中移除这个点
                Pose2D removed_point = last_segment_before_update->getEndPose();
                auto it = std::find_if(stop_points_.begin(), stop_points_.end(),
                    [&removed_point](const Pose2D& p) {
                        return std::abs(p.x - removed_point.x) < 0.001 &&
                               std::abs(p.y - removed_point.y) < 0.001;
                    });
                if (it != stop_points_.end()) {
                    stop_points_.erase(it);
                }
                
                log("Hot update: smooth transition at junction point (no stop required)");
            }
        } else {
            log("Hot update: stop required at junction point due to heading change");
        }
    }

    std::stringstream ss;
    ss << "Path updated with " << additional_waypoints.size() << " new waypoints, "
       << "total segments: " << path_segments_.size() 
       << ", stop points: " << stop_points_.size();
    log(ss.str());
    
    return 0;
}

//==============================================================================
// 里程计反馈接口实现
//==============================================================================

void NavigationExecutor::updateOdometryFeedback(const Pose2D& pose,
                                                  const Velocity& velocity,
                                                  double timestamp) {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_feedback_.update(pose, velocity, timestamp);
}

void NavigationExecutor::invalidateOdometryFeedback() {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_feedback_.invalidate();
}

bool NavigationExecutor::isOdometryFeedbackValid(double current_time) const {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    return !odom_feedback_.isExpired(current_time);
}

void NavigationExecutor::setOdometryTimeout(double timeout) {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_feedback_.timeout = timeout;
}

Optional<OdometryFeedback> NavigationExecutor::getValidOdometryFeedback(double current_time) const {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    if (odom_feedback_.is_valid && !odom_feedback_.isExpired(current_time)) {
        return odom_feedback_;
    }
    return nullopt;
}

//==============================================================================
// 停止点/过渡点配置接口实现
//==============================================================================

void NavigationExecutor::setTransitionAngleThreshold(double threshold) {
    transition_angle_threshold_ = threshold;
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "Transition angle threshold set to: " << threshold 
       << " rad (" << (threshold * 180.0 / M_PI) << " deg)";
    log(ss.str());
    
    // 如果已经初始化，重新计算停止点
    if (is_initialized_ && !path_segments_.empty()) {
        computeStopPoints();
    }
}

bool NavigationExecutor::isStopPoint(size_t waypoint_index) const {
    if (waypoint_index >= is_stop_point_.size()) {
        return true;  // 索引越界默认为停止点
    }
    return is_stop_point_[waypoint_index];
}

//==============================================================================
// 急停和恢复接口实现
//==============================================================================

void NavigationExecutor::emergencyStop() {
    // 记录停止时的状态
    is_stopped_ = true;
    stop_reason_ = StopReason::kEmergencyStop;
    
    // 保存当前位姿（会在下一次execute调用时更新）
    // stop_pose_ 在 executeWithOdometry 中会被设置
    
    // 保存当前速度用于后续平滑恢复
    stop_velocity_ = std::sqrt(
        last_control_velocity_.linear_x * last_control_velocity_.linear_x +
        last_control_velocity_.linear_y * last_control_velocity_.linear_y);
    
    // 处理状态机事件
    state_machine_.processEvent(StateMachineEvent::kEmergencyStop);
    
    log("Emergency stop triggered");
}

RecoveryStrategy NavigationExecutor::resumeFromStop(const Pose2D& current_pose) {
    if (!is_stopped_) {
        return RecoveryStrategy::kContinue;
    }
    
    // 检测位置偏移
    PositionDeviation deviation = checkPositionDeviation(current_pose);
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "Position deviation: distance=" << deviation.distance 
       << "m, angle=" << (deviation.angle * 180.0 / M_PI) << "deg";
    log(ss.str());
    
    // 根据偏移决定恢复策略
    RecoveryStrategy strategy = deviation.strategy;
    
    if (strategy == RecoveryStrategy::kAbort) {
        log("Deviation too large, aborting navigation");
        state_machine_.processEvent(StateMachineEvent::kError);
    } else if (strategy == RecoveryStrategy::kReplan) {
        log("Replanning current segment due to position deviation");
        if (replanCurrentSegment(current_pose) == 0) {
            log("Replan successful");
        } else {
            log("Replan failed, continuing with original path");
            strategy = RecoveryStrategy::kContinue;
        }
    } else if (strategy == RecoveryStrategy::kReturnToPath) {
        log("Returning to nearest path point");
        // TODO: 实现返回最近路径点的逻辑
        strategy = RecoveryStrategy::kReplan;  // 暂时用重规划代替
        replanCurrentSegment(current_pose);
    }
    
    // 重置停止状态
    is_stopped_ = false;
    is_recovering_ = true;
    recovery_velocity_ = 0.0;  // 从零开始平滑恢复
    current_recovery_strategy_ = strategy;
    stop_reason_ = StopReason::kNone;
    
    // 恢复状态机
    state_machine_.processEvent(StateMachineEvent::kResume);
    
    return strategy;
}

void NavigationExecutor::setDeviationThresholds(double replan_distance, double replan_angle,
                                                  double abort_distance) {
    replan_distance_threshold_ = replan_distance;
    replan_angle_threshold_ = replan_angle;
    abort_distance_threshold_ = abort_distance;
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "Deviation thresholds set: replan_dist=" << replan_distance 
       << "m, replan_angle=" << (replan_angle * 180.0 / M_PI) 
       << "deg, abort_dist=" << abort_distance << "m";
    log(ss.str());
}

void NavigationExecutor::setRealtimeDeviationCheck(bool enable, double threshold) {
    enable_realtime_deviation_check_ = enable;
    realtime_deviation_threshold_ = threshold;
    
    std::stringstream ss;
    ss << "Realtime deviation check: " << (enable ? "enabled" : "disabled")
       << ", threshold=" << threshold << "m";
    log(ss.str());
}

PositionDeviation NavigationExecutor::checkPositionDeviation(const Pose2D& current_pose) const {
    PositionDeviation deviation;
    
    // 计算位置偏移
    double dx = current_pose.x - stop_pose_.x;
    double dy = current_pose.y - stop_pose_.y;
    deviation.distance = std::sqrt(dx * dx + dy * dy);
    
    // 计算角度偏移
    deviation.angle = std::abs(math::MathUtils::normalizeAngle(
        current_pose.yaw - stop_pose_.yaw));
    
    // 判断是否需要重规划
    bool distance_exceeded = deviation.distance > replan_distance_threshold_;
    bool angle_exceeded = deviation.angle > replan_angle_threshold_;
    bool abort_exceeded = deviation.distance > abort_distance_threshold_;
    
    deviation.needs_replan = distance_exceeded || angle_exceeded;
    
    // 确定恢复策略
    if (abort_exceeded) {
        deviation.strategy = RecoveryStrategy::kAbort;
    } else if (distance_exceeded && angle_exceeded) {
        deviation.strategy = RecoveryStrategy::kReplan;
    } else if (distance_exceeded) {
        deviation.strategy = RecoveryStrategy::kReturnToPath;
    } else if (angle_exceeded) {
        deviation.strategy = RecoveryStrategy::kReplan;  // 角度偏差大时重规划
    } else {
        deviation.strategy = RecoveryStrategy::kContinue;
    }
    
    return deviation;
}

int NavigationExecutor::replanCurrentSegment(const Pose2D& current_pose) {
    if (current_segment_index_ >= static_cast<int>(path_segments_.size())) {
        log("Cannot replan: no current segment");
        return -1;
    }

    auto& segment = path_segments_[current_segment_index_];
    const auto& constraints = segment->getConstraints();
    const auto& end_pose = segment->getEndPose();
    
    // 根据段类型重新创建段
    if (segment->getType() == PathSegmentType::kStraight) {
        // 从当前位置到原终点创建新的直线段
        auto new_segment = std::make_shared<StraightSegment>(
            current_pose, end_pose, constraints);
        path_segments_[current_segment_index_] = new_segment;
        
        // 重新初始化规划器和控制器
        linear_planner_.initializeLinear(
            new_segment->getTotalLength(), 0.0, constraints);
        
        auto* straight = dynamic_cast<StraightSegment*>(new_segment.get());
        if (straight) {
            double k, b;
            bool has_slope = straight->getLineParameters(k, b);
            target_heading_ = new_segment->getStartHeading();
            straight_controller_.initialize(
                target_heading_, k, b,
                !has_slope, end_pose.x,
                chassis_type_);
        }
    } else if (segment->getType() == PathSegmentType::kCubicBezier) {
        // 贝塞尔曲线需要重新计算控制点
        // 简化处理：转换为直线段
        auto new_segment = std::make_shared<StraightSegment>(
            current_pose, end_pose, constraints);
        path_segments_[current_segment_index_] = new_segment;
        
        linear_planner_.initializeLinear(
            new_segment->getTotalLength(), 0.0, constraints);
        
        auto* straight = dynamic_cast<StraightSegment*>(new_segment.get());
        if (straight) {
            double k, b;
            bool has_slope = straight->getLineParameters(k, b);
            target_heading_ = new_segment->getStartHeading();
            straight_controller_.initialize(
                target_heading_, k, b,
                !has_slope, end_pose.x,
                chassis_type_);
        }
        
        log("Bezier segment converted to straight segment for replanning");
    }
    
    std::stringstream ss;
    ss << "Segment " << current_segment_index_ << " replanned from current position";
    log(ss.str());
    
    return 0;
}

Velocity NavigationExecutor::adjustVelocityForChassis(const Velocity& velocity) const {
    Velocity adjusted = velocity;
    
    // 差速底盘不支持横向速度
    if (chassis_type_ == ChassisType::kDifferential) {
        adjusted.linear_y = 0.0;
    }
    
    return adjusted;
}

Velocity NavigationExecutor::smoothVelocityTransition(const Velocity& target_velocity,
                                                        const Velocity& current_velocity,
                                                        double max_acceleration,
                                                        double dt) const {
    Velocity smoothed;
    
    // 计算允许的最大速度变化
    double max_delta = max_acceleration * dt;
    
    // 平滑 vx
    double delta_vx = target_velocity.linear_x - current_velocity.linear_x;
    if (std::abs(delta_vx) > max_delta) {
        smoothed.linear_x = current_velocity.linear_x + 
            math::MathUtils::sign(delta_vx) * max_delta;
    } else {
        smoothed.linear_x = target_velocity.linear_x;
    }
    
    // 平滑 vy（仅四转四驱）
    if (chassis_type_ == ChassisType::k4WIS4WID) {
        double delta_vy = target_velocity.linear_y - current_velocity.linear_y;
        if (std::abs(delta_vy) > max_delta) {
            smoothed.linear_y = current_velocity.linear_y + 
                math::MathUtils::sign(delta_vy) * max_delta;
        } else {
            smoothed.linear_y = target_velocity.linear_y;
        }
    } else {
        smoothed.linear_y = 0.0;
    }
    
    // 平滑 ω
    double max_angular_delta = max_acceleration * dt;  // 简化，使用相同加速度
    double delta_w = target_velocity.angular - current_velocity.angular;
    if (std::abs(delta_w) > max_angular_delta) {
        smoothed.angular = current_velocity.angular + 
            math::MathUtils::sign(delta_w) * max_angular_delta;
    } else {
        smoothed.angular = target_velocity.angular;
    }
    
    return smoothed;
}

//==============================================================================
// 执行控制
//==============================================================================

NavigationOutput NavigationExecutor::execute(const Pose2D& current_pose,
                                              bool obstacle_stop,
                                              double obstacle_deceleration,
                                              bool cancel) {
    // 不使用里程计反馈的版本
    use_odom_feedback_.store(false);
    return executeWithOdometry(current_pose, obstacle_stop, obstacle_deceleration, 
                               cancel, 0.0);
}

NavigationOutput NavigationExecutor::executeWithOdometry(const Pose2D& current_pose,
                                                           bool obstacle_stop,
                                                           double obstacle_deceleration,
                                                           bool cancel,
                                                           double current_time) {
    std::unique_lock<std::shared_mutex> path_lock(path_mutex_);
    NavigationOutput output;
    output.status = NavigationStatus::kIdle;
    output.chassis_type = chassis_type_;
    output.is_recovering = is_recovering_;
    output.recovery_strategy = current_recovery_strategy_;

    if (!is_initialized_) {
        return output;
    }

    // 检查里程计反馈有效性
    auto odom = getValidOdometryFeedback(current_time);
    use_odom_feedback_.store(odom.has_value());
    output.using_odom_feedback = use_odom_feedback_.load();

    // 处理取消
    if (cancel && !is_cancelled_) {
        is_cancelled_ = true;
        state_machine_.processEvent(StateMachineEvent::kCancel);
    }

    // 处理障碍物状态变化
    if (obstacle_stop && !last_obstacle_stop_) {
        // 记录停止时的位姿和速度
        stop_pose_ = current_pose;
        stop_velocity_ = std::sqrt(
            last_control_velocity_.linear_x * last_control_velocity_.linear_x +
            last_control_velocity_.linear_y * last_control_velocity_.linear_y);
        is_stopped_ = true;//TODO 这里可能需要区分急停和障碍物停 
        stop_reason_ = StopReason::kObstacle;
    
        state_machine_.processEvent(StateMachineEvent::kObstacleDetected);

    } else if (!obstacle_stop && last_obstacle_stop_) {
        // 障碍物清除，检查位置偏移并决定恢复策略
        //TODO 障碍物清楚要更新事件
        RecoveryStrategy strategy = resumeFromStop(current_pose);
        output.recovery_strategy = strategy;
        output.is_recovering = true;
    }
    last_obstacle_stop_ = obstacle_stop;

    // 根据状态执行
    switch (state_machine_.getCurrentState()) {//TODO 这里可以考虑传入output的引用而非副本
        case StateMachineState::kStartRotation:
            output = handleStartRotation(current_pose, obstacle_stop, obstacle_deceleration);
            break;

        case StateMachineState::kFollowingPath:
            output = handlePathFollowing(current_pose, obstacle_stop, obstacle_deceleration, cancel);
            break;

        case StateMachineState::kEndRotation:
            output = handleEndRotation(current_pose, obstacle_stop, obstacle_deceleration);
            break;

        case StateMachineState::kDecelerating:
            output.velocity = decelerateStop(obstacle_deceleration, obstacle_deceleration);
            output.status = NavigationStatus::kObstaclePaused;// TODO 这里要注意，cancel / 急停/障碍物停时减速到0如何告诉更新状态？
            break;

        case StateMachineState::kPaused:
            output.velocity = Velocity(0, 0, 0);
            output.status = NavigationStatus::kObstaclePaused;//TODO 有可能是急停，根据stop_reason_区分
            break;

        case StateMachineState::kCompleted:
            output.velocity = Velocity(0, 0, 0);
            output.status = NavigationStatus::kGoalReached;
            break;

        case StateMachineState::kError:
            output.velocity = Velocity(0, 0, 0);
            output.status = NavigationStatus::kError;
            break;

        default:
            break;
    }

    // 如果有里程计反馈，使用它来平滑速度过渡
    if (use_odom_feedback_.load() && odom.has_value()) {
        double max_acc = 0.5;  // 默认最大加速度
        if (!path_segments_.empty() && current_segment_index_ < static_cast<int>(path_segments_.size())) {
            max_acc = path_segments_[current_segment_index_]->getConstraints().max_acceleration;
        }
        output.velocity = smoothVelocityTransition(output.velocity, odom->velocity, 
                                                    max_acc, control_period_);
    }

    // 恢复过程中的速度平滑加速
    if (is_recovering_) {
        double target_speed = std::sqrt(
            output.velocity.linear_x * output.velocity.linear_x +
            output.velocity.linear_y * output.velocity.linear_y);
        
        // 平滑加速恢复
        if (recovery_velocity_ < target_speed) {
            recovery_velocity_ += recovery_acceleration_ * control_period_;
            recovery_velocity_ = std::min(recovery_velocity_, target_speed);
            
            // 按比例缩放速度
            if (target_speed > 1e-6) {
                double scale = recovery_velocity_ / target_speed;
                output.velocity.linear_x *= scale;
                output.velocity.linear_y *= scale;
            }
        } else {
            // 恢复完成
            is_recovering_ = false;
            current_recovery_strategy_ = RecoveryStrategy::kContinue;
            log("Velocity recovery completed");
        }
        
        output.is_recovering = is_recovering_;
    }

    // 根据底盘类型调整输出
    output.velocity = adjustVelocityForChassis(output.velocity);
    output.chassis_type = chassis_type_;

    // 更新状态
    last_control_velocity_ = output.velocity;
    output.current_segment = current_segment_index_;

    return output;
}

NavigationOutput NavigationExecutor::handleStartRotation(const Pose2D& current_pose,
                                                          bool obstacle_stop,
                                                          double obstacle_decel) {
    auto odom = use_odom_feedback_.load() ? Optional<OdometryFeedback>(odom_feedback_) : Optional<OdometryFeedback>();
    NavigationOutput output = executeRotation(current_pose, target_heading_, 
                                               obstacle_stop, obstacle_decel, odom);

    if (output.status == NavigationStatus::kGoalReached) {
        // 旋转完成，切换到路径跟踪
        state_machine_.processEvent(StateMachineEvent::kRotationComplete);
        is_rotating_ = false;
        
        // 初始化第一段的控制器
        if (!path_segments_.empty()) {
            const auto& segment = path_segments_[0];
            if (segment->getType() == PathSegmentType::kStraight) {
                linear_planner_.initializeLinear(
                    segment->getTotalLength(), 0.0,
                    segment->getConstraints());
                
                auto* straight = dynamic_cast<StraightSegment*>(segment.get());
                if (straight) {
                    double k, b;
                    bool has_slope = straight->getLineParameters(k, b);
                    straight_controller_.initialize(
                        target_heading_, k, b, 
                        !has_slope, segment->getEndPose().x,
                        chassis_type_);
                }
            }
        }
        
        output.status = NavigationStatus::kRunning;
    }

    return output;
}

NavigationOutput NavigationExecutor::handlePathFollowing(const Pose2D& current_pose,
                                                          bool obstacle_stop,
                                                          double obstacle_decel,
                                                          bool /*cancel*/) {
    // 检查是否已完成所有段
    if (current_segment_index_ >= static_cast<int>(path_segments_.size())) {
        state_machine_.processEvent(StateMachineEvent::kPathComplete);
        NavigationOutput output;
        output.status = NavigationStatus::kGoalReached;
        return output;
    }

    auto& segment = path_segments_[current_segment_index_];
    double remaining = segment->getRemainingDistance(current_pose);

    NavigationOutput output;

    // 实时检测路径偏离（只在非停止状态下检测）
    if (!obstacle_stop && enable_realtime_deviation_check_) {
        double lateral_deviation = segment->getLateralDeviation(current_pose);
        if (lateral_deviation > realtime_deviation_threshold_) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "Real-time deviation detected: " << lateral_deviation << "m";
            log(ss.str());
            
            // 尝试重规划
            if (replanCurrentSegment(current_pose) == 0) {
                log("Real-time replan successful");
            }
        }
    }

    // 获取可选的里程计反馈
    auto odom = use_odom_feedback_.load() ? Optional<OdometryFeedback>(odom_feedback_) : Optional<OdometryFeedback>();

    // 根据段类型执行跟踪
    if (segment->getType() == PathSegmentType::kStraight) {
        auto* straight = dynamic_cast<StraightSegment*>(segment.get());
        output = trackStraightSegment(current_pose, straight, remaining, 
                                       obstacle_stop, obstacle_decel, odom);
    } else if (segment->getType() == PathSegmentType::kCubicBezier) {
        auto* bezier = dynamic_cast<BezierSegment*>(segment.get());
        output = trackBezierSegment(current_pose, bezier, remaining, odom);
    }

    // 检查是否到达段终点
    if (segment->isReached(current_pose, segment->getConstraints().reach_distance)) {
        std::stringstream ss;
        ss << "Segment " << current_segment_index_ << " reached";
        log(ss.str());
        
        if (!advanceToNextSegment()) {
            // 所有段完成
            state_machine_.processEvent(StateMachineEvent::kPathComplete);
        } else {
            // 还有下一段
            state_machine_.processEvent(StateMachineEvent::kSegmentComplete);
            output.status = NavigationStatus::kSegmentReached;
        }
    }

    output.remaining_distance = remaining;
    return output;
}

NavigationOutput NavigationExecutor::trackStraightSegment(const Pose2D& current_pose,
                                                           StraightSegment* segment,
                                                           double remaining_distance,
                                                           bool obstacle_stop,
                                                           double obstacle_decel,
                                                           const Optional<OdometryFeedback>& odom) {
    NavigationOutput output;
    const auto& constraints = segment->getConstraints();

    // 处理倒车时的角度
    double adjusted_angle = adjustAngleForDirection(current_pose.yaw, constraints.is_forward);

    // 检查是否需要原地旋转
    double angle_error = math::MathUtils::absoluteAngleDiff(target_heading_, adjusted_angle);
    if (angle_error > constraints.reach_angle && is_rotating_) {
        return executeRotation(current_pose, target_heading_, obstacle_stop, obstacle_decel, odom);
    }
    is_rotating_ = false;

    // 速度规划
    double planned_velocity = linear_planner_.computeVelocity(remaining_distance, control_period_);
    
    Velocity ref_vel(planned_velocity, 0, 0);
    output.reference_velocity = ref_vel;

    // 障碍物停止处理
    if (obstacle_stop) {
        int direction = constraints.is_forward ? 1 : -1;
        double current_v = odom.has_value() ? odom->velocity.linear_x : last_control_velocity_.linear_x;
        output.velocity.linear_x = current_v - direction * obstacle_decel * control_period_;
        if (output.velocity.linear_x * direction <= 0) {
            output.velocity.linear_x = 0;
        }
        output.velocity.linear_y = 0;
        output.velocity.angular = 0;
        output.status = NavigationStatus::kObstaclePaused;
        return output;
    }

    // LQR跟踪控制 - 使用带反馈版本（如果有反馈）
    if (odom.has_value()) {
        output.velocity = straight_controller_.computeWithFeedback(
            current_pose, ref_vel, constraints.is_forward, odom);
    } else {
        output.velocity = straight_controller_.compute(current_pose, ref_vel, constraints.is_forward);
    }
    
    output.status = constraints.is_forward ? NavigationStatus::kRunning : NavigationStatus::kReversing;

    last_planned_velocity_ = ref_vel;
    return output;
}

NavigationOutput NavigationExecutor::trackBezierSegment(const Pose2D& current_pose,
                                                         BezierSegment* segment,
                                                         double /*remaining_distance*/,
                                                         const Optional<OdometryFeedback>& odom) {
    NavigationOutput output;
    const auto& constraints = segment->getConstraints();

    // 梯形/三角形速度规划（支持非零初始速度和目标速度的平滑过渡）
    double v_max = constraints.max_velocity;
    double acc = constraints.max_acceleration;
    double dec = constraints.max_deceleration;
    double total_length = segment->getTotalLength();
    double remaining_length = total_length - bezier_arc_length_;

    // 确定目标终点速度（如果下一段是过渡点，则不需要减速到零）
    double v_end_target = 0.0;
    if (current_segment_index_ + 1 < static_cast<int>(path_segments_.size()) &&
        current_waypoint_index_ + 1 < static_cast<int>(is_stop_point_.size())) {
        bool next_is_transition = !is_stop_point_[current_waypoint_index_ + 1];
        if (next_is_transition) {
            double next_max_vel = 
                path_segments_[current_segment_index_ + 1]->getConstraints().max_velocity;
            v_end_target = std::min(next_max_vel, v_max) * 0.8;  // 留20%余量
        }
    }

    // 根据剩余距离计算需要的减速距离（减速到目标速度而不是零）
    double v_current = std::abs(last_planned_velocity_.linear_x);
    double delta_v = v_current - v_end_target;
    double s_dec_needed = (delta_v > 0) ? 
        (v_current * v_current - v_end_target * v_end_target) / (2 * dec) : 0.0;
    
    double v_planned;
    
    if (delta_v > 0 && remaining_length <= s_dec_needed) {
        // 需要减速到目标速度
        // 使用公式 v^2 = v_end^2 + 2*a*(remaining_length)
        double v_squared = v_end_target * v_end_target + 2 * dec * remaining_length;
        v_planned = (v_squared > 0) ? std::sqrt(v_squared) : v_end_target;
        
        // 用时间递推进行平滑：v = v_current - dec * dt
        double v_from_decel = v_current - dec * control_period_;
        v_planned = std::max(v_end_target, std::min(v_planned, std::max(v_end_target, v_from_decel)));
    } else {
        // 加速或匀速阶段
        if (v_current < v_max) {
            // 加速
            v_planned = v_current + acc * control_period_;
            v_planned = std::min(v_planned, v_max);
        } else {
            // 匀速
            v_planned = v_max;
        }
        
        // 检查是否需要开始减速（前瞻）
        double v_for_check = std::max(v_planned, v_end_target);
        double s_dec_full = (v_for_check * v_for_check - v_end_target * v_end_target) / (2 * dec);
        if (remaining_length <= s_dec_full * 1.1) {  // 1.1倍余量
            // 开始减速
            v_planned = v_current - dec * control_period_;
            v_planned = std::max(v_end_target, v_planned);
        }
    }
    
    v_planned = std::max(0.0, std::min(v_max, v_planned));

    // 更新弧长和时间
    bezier_arc_length_ += v_planned * control_period_;
    bezier_time_ += control_period_;

    // 获取参考点和航向
    Eigen::Vector2d prev_point = bezier_ref_point_;
    bezier_ref_point_ = segment->getPointAtArcLength(bezier_arc_length_);

    if ((bezier_ref_point_ - prev_point).norm() > 1e-6) {
        bezier_ref_heading_ = std::atan2(
            bezier_ref_point_(1) - prev_point(1),
            bezier_ref_point_(0) - prev_point(0));
    }

    // 计算参考角速度
    double w_ref = (bezier_ref_heading_ - target_heading_) / control_period_;
    target_heading_ = bezier_ref_heading_;

    // TODO 考虑后续加入四转四驱底盘的横向速度控制
    Velocity ref_vel(v_planned, 0, w_ref);
    output.reference_velocity = ref_vel;

    // LQR跟踪控制 - 使用带反馈版本（如果有反馈）
    if (odom.has_value()) {
        output.velocity = bezier_controller_.computeWithFeedback(
            current_pose, bezier_ref_point_,
            ref_vel, bezier_ref_heading_,
            constraints.is_forward, odom);
    } else {
        output.velocity = bezier_controller_.compute(
            current_pose, bezier_ref_point_,
            ref_vel, bezier_ref_heading_,
            constraints.is_forward);
    }
    
    // 限制最大速度
    output.velocity.linear_x = math::MathUtils::clamp(
        output.velocity.linear_x, -1.1 * v_max, 1.1 * v_max);

    output.status = constraints.is_forward ? 
        NavigationStatus::kRunning : NavigationStatus::kReversing;
    
    last_planned_velocity_ = ref_vel;
    return output;
}

NavigationOutput NavigationExecutor::handleEndRotation(const Pose2D& current_pose,
                                                        bool obstacle_stop,
                                                        double obstacle_decel) {
    auto odom = use_odom_feedback_.load() ? Optional<OdometryFeedback>(odom_feedback_) : Optional<OdometryFeedback>();
    NavigationOutput output = executeRotation(current_pose, end_heading_,
                                               obstacle_stop, obstacle_decel, odom);

    if (output.status == NavigationStatus::kGoalReached) {
        state_machine_.processEvent(StateMachineEvent::kRotationComplete);
    }

    return output;
}

NavigationOutput NavigationExecutor::executeRotation(const Pose2D& current_pose,
                                                      double target_angle,
                                                      bool obstacle_stop,
                                                      double obstacle_decel,
                                                      const Optional<OdometryFeedback>& odom) {
    NavigationOutput output;
    output.status = NavigationStatus::kRotating;

    double angle_error = math::MathUtils::absoluteAngleDiff(target_angle, current_pose.yaw);
    
    // 检查是否完成
    double reach_threshold = path_segments_.empty() ? 0.01 : 
        path_segments_[current_segment_index_]->getConstraints().reach_angle;
    
    if (angle_error < reach_threshold) {
        output.velocity = Velocity(0, 0, 0);
        output.status = NavigationStatus::kGoalReached;
        return output;
    }

    // 确定旋转方向
    int direction = math::MathUtils::rotationDirection(current_pose.yaw, target_angle);

    // 障碍物停止处理
    if (obstacle_stop) {
        output.velocity.linear_x = 0;
        output.velocity.linear_y = 0;
        double current_w = odom.has_value() ? odom->velocity.angular : last_control_velocity_.angular;
        output.velocity.angular = current_w - direction * obstacle_decel * control_period_;
        if (output.velocity.angular * direction <= 0) {
            output.velocity.angular = 0;
        }
        output.status = NavigationStatus::kObstaclePaused;
        return output;
    }

    // S曲线角速度规划
    double w = rotation_planner_.computeAngularVelocity(angle_error, control_period_);
    
    // 如果有里程计反馈，使用当前实际角速度进行平滑
    if (odom.has_value() && odom->is_valid) {
        double current_w = odom->velocity.angular;
        const double smooth_factor = 0.4;
        w = current_w + smooth_factor * (direction * w - current_w);
        w = std::abs(w);  // 取绝对值，方向由 direction 决定
    }
    
    output.velocity = Velocity(0, 0, direction * w);
    output.reference_velocity = output.velocity;

    output.heading_error = angle_error;
    return output;
}

bool NavigationExecutor::advanceToNextSegment() {
    current_segment_index_++;
    current_waypoint_index_++;

    if (current_segment_index_ >= static_cast<int>(path_segments_.size())) {
        return false;
    }

    // 更新目标航向
    updateTargetHeading();

    // 检查是否是停止点（需要停下来转向）还是过渡点（可以平滑通过）
    bool is_stop = (current_waypoint_index_ < static_cast<int>(is_stop_point_.size())) &&
                   is_stop_point_[current_waypoint_index_];
    
    // 获取当前速度用于平滑过渡
    double current_velocity = std::abs(last_control_velocity_.linear_x);
    if (is_stop) {
        current_velocity = 0.0;  // 停止点从零开始
    }

    // 初始化新段的规划器和控制器
    const auto& segment = path_segments_[current_segment_index_];
    
    // 检查下一段是否需要平滑过渡（非停止点）
    bool next_is_transition = false;
    double next_segment_max_velocity = 0.0;
    if (current_segment_index_ + 1 < static_cast<int>(path_segments_.size()) &&
        current_waypoint_index_ + 1 < static_cast<int>(is_stop_point_.size())) {
        next_is_transition = !is_stop_point_[current_waypoint_index_ + 1];
        if (next_is_transition) {
            // 下一段的最大速度
            next_segment_max_velocity = 
                path_segments_[current_segment_index_ + 1]->getConstraints().max_velocity;
            // 取当前段和下一段最大速度的较小值作为过渡速度
            next_segment_max_velocity = std::min(
                next_segment_max_velocity, 
                segment->getConstraints().max_velocity);
        }
    }
    
    if (segment->getType() == PathSegmentType::kStraight) {
        // 直线段初始化
        if (is_stop) {
            linear_planner_.initializeLinear(
                segment->getTotalLength(), 0.0,
                segment->getConstraints());
        } else {
            // 过渡点：继承当前速度，实现平滑过渡
            linear_planner_.updatePath(
                segment->getTotalLength(), 
                current_velocity,
                segment->getConstraints());
            
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "Smooth transition to straight segment with initial velocity: " 
               << current_velocity << " m/s";
            log(ss.str());
        }
        
        // 设置目标终点速度：如果下一段是过渡点，则不需要减速到零
        if (next_is_transition && next_segment_max_velocity > 0.01) {
            linear_planner_.setTargetEndVelocity(next_segment_max_velocity * 0.8);  // 留20%余量
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "Set target end velocity for smooth transition: " 
               << next_segment_max_velocity * 0.8 << " m/s";
            log(ss.str());
        } else {
            linear_planner_.setTargetEndVelocity(0.0);  // 停止点需要减速到零
        }

        auto* straight = dynamic_cast<StraightSegment*>(segment.get());
        if (straight) {
            double k, b;
            bool has_slope = straight->getLineParameters(k, b);
            straight_controller_.initialize(
                target_heading_, k, b,
                !has_slope, segment->getEndPose().x,
                chassis_type_);
        }

        // 检查是否需要旋转（仅在停止点时才需要原地旋转）
        if (is_stop) {
            double current_adjusted = adjustAngleForDirection(
                target_heading_, segment->getConstraints().is_forward);
            double angle_error = math::MathUtils::absoluteAngleDiff(
                target_heading_, current_adjusted);
            
            if (angle_error > segment->getConstraints().reach_angle) {
                is_rotating_ = true;
                rotation_planner_.initializeRotation(
                    angle_error, 0.0, segment->getConstraints());
            }
        }
    } else if (segment->getType() == PathSegmentType::kCubicBezier) {
        // 贝塞尔段初始化
        bezier_arc_length_ = 0.0;
        bezier_ref_point_ = segment->getStartPose().toVector();
        bezier_ref_heading_ = segment->getStartHeading();
        
        if (is_stop) {
            // 停止点：从零开始
            bezier_time_ = 0.0;
        } else {
            // 过渡点：计算等效的起始时间，使初始速度与上一段末速度匹配
            // 使用梯形速度规划的加速段公式反推时间
            const auto& constraints = segment->getConstraints();
            double v0 = current_velocity;
            double acc = constraints.max_acceleration;
            
            // t = v / a (从加速段公式 v = a*t 反推)
            if (acc > 0.001 && v0 > 0.001) {
                bezier_time_ = v0 / acc;
                // 同时更新弧长，保持一致性
                bezier_arc_length_ = 0.5 * acc * bezier_time_ * bezier_time_;
            } else {
                bezier_time_ = 0.0;
            }
            
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "Smooth transition to bezier segment with initial velocity: " 
               << v0 << " m/s, equivalent time: " << bezier_time_ << " s";
            log(ss.str());
        }
    }

    return true;
}

void NavigationExecutor::updateTargetHeading() {
    if (current_segment_index_ < static_cast<int>(path_segments_.size())) {
        target_heading_ = path_segments_[current_segment_index_]->getStartHeading();
    }
    
    if (!path_segments_.empty()) {
        end_heading_ = path_segments_.back()->getEndHeading();
    }
}

bool NavigationExecutor::needsRotation(double current_angle, double target_angle,
                                        double threshold) const {
    return math::MathUtils::absoluteAngleDiff(current_angle, target_angle) > threshold;
}

double NavigationExecutor::adjustAngleForDirection(double angle, bool is_forward) const {
    if (!is_forward) {
        angle += M_PI;
        angle = math::MathUtils::normalizeAngle(angle);
    }
    return angle;
}

void NavigationExecutor::cancel() {
    is_cancelled_ = true;
    state_machine_.processEvent(StateMachineEvent::kCancel);
}

void NavigationExecutor::stop(double /*max_linear_decel*/, double /*max_angular_decel*/) {
    state_machine_.processEvent(StateMachineEvent::kObstacleDetected);
}

Velocity NavigationExecutor::decelerateStop(double linear_decel, double angular_decel) {
    Velocity result;

    // x方向线速度减速
    if (std::abs(last_control_velocity_.linear_x) > 0.001) {
        int sign = math::MathUtils::sign(last_control_velocity_.linear_x);
        result.linear_x = last_control_velocity_.linear_x - sign * linear_decel * control_period_;
        if (result.linear_x * sign < 0) {
            result.linear_x = 0;
        }
    }

    // y方向线速度减速（仅四转四驱）
    if (chassis_type_ == ChassisType::k4WIS4WID) {
        if (std::abs(last_control_velocity_.linear_y) > 0.001) {
            int sign = math::MathUtils::sign(last_control_velocity_.linear_y);
            result.linear_y = last_control_velocity_.linear_y - sign * linear_decel * control_period_;
            if (result.linear_y * sign < 0) {
                result.linear_y = 0;
            }
        }
    }

    // 角速度减速
    if (std::abs(last_control_velocity_.angular) > 0.001) {
        int sign = math::MathUtils::sign(last_control_velocity_.angular);
        result.angular = last_control_velocity_.angular - sign * angular_decel * control_period_;
        if (result.angular * sign < 0) {
            result.angular = 0;
        }
    }

    last_control_velocity_ = result;
    return result;
}

}  // namespace laser_navigation
