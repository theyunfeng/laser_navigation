/**
 * @file bezier_segment.cpp
 * @brief 三阶贝塞尔曲线路径段实现
 * 
 * 对应原代码 bezier.cpp 的重构
 */

#include "laser_navigation_refactored/path/bezier_segment.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include <algorithm>
#include <cmath>

namespace laser_navigation {

BezierSegment::BezierSegment(const Pose2D& start, const Pose2D& end,
                             const Pose2D& control_point1, const Pose2D& control_point2,
                             const MotionConstraints& constraints) {
    constraints_ = constraints;
    
    // 初始化控制点 [P0, P1, P2, P3]
    control_points_.resize(4);
    control_points_[0] = Eigen::Vector2d(start.x, start.y);
    control_points_[1] = Eigen::Vector2d(control_point1.x, control_point1.y);
    control_points_[2] = Eigen::Vector2d(control_point2.x, control_point2.y);
    control_points_[3] = Eigen::Vector2d(end.x, end.y);
    
    // 计算起点航向（P0->P1方向）
    Eigen::Vector2d start_tangent = control_points_[1] - control_points_[0];
    if (start_tangent.norm() > 1e-6) {
        start_heading_ = std::atan2(start_tangent(1), start_tangent(0));
    } else {
        start_heading_ = start.yaw;//! 如果没有给yaw呢？假设默认是0会不会出问题？
    }
    
    // 计算终点航向（P2->P3方向）
    Eigen::Vector2d end_tangent = control_points_[3] - control_points_[2];
    if (end_tangent.norm() > 1e-6) {
        end_heading_ = std::atan2(end_tangent(1), end_tangent(0));
    } else {
        end_heading_ = end.yaw;//! 同上
    }
    
    // 计算弧长表
    computeArcLengthTable();
    total_length_ = arc_length_table_.back();
}

void BezierSegment::computeArcLengthTable() {
    t_table_.resize(kTableResolution);
    arc_length_table_.resize(kTableResolution);
    
    // 生成参数表 [0, 1]
    for (int i = 0; i < kTableResolution; ++i) {
        t_table_[i] = static_cast<double>(i) / (kTableResolution - 1);
    }
    
    // 计算累积弧长
    // 对应原代码 Bezier::init() 中的弧长计算
    arc_length_table_[0] = 0.0;
    for (int i = 1; i < kTableResolution; ++i) {
        Eigen::Vector2d p_curr = getPointAtParameter(t_table_[i]);
        Eigen::Vector2d p_prev = getPointAtParameter(t_table_[i - 1]);
        double ds = (p_curr - p_prev).norm();
        arc_length_table_[i] = arc_length_table_[i - 1] + ds;
    }
}

Eigen::Vector2d BezierSegment::getPointAtParameter(double t) const {
    // 三阶贝塞尔曲线公式
    // B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
    // 对应原代码 Bezier::BezierPoint(double t)
    
    t = math::MathUtils::clamp(t, 0.0, 1.0);
    
    double u = 1.0 - t;
    double u2 = u * u;
    double u3 = u2 * u;
    double t2 = t * t;
    double t3 = t2 * t;
    
    Eigen::Vector2d result = 
        u3 * control_points_[0] +
        3.0 * u2 * t * control_points_[1] +
        3.0 * u * t2 * control_points_[2] +
        t3 * control_points_[3];
    
    return result;
}

Eigen::Vector2d BezierSegment::getFirstDerivative(double t) const {
    // 一阶导数: B'(t) = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2)
    t = math::MathUtils::clamp(t, 0.0, 1.0);
    
    double u = 1.0 - t;
    
    Eigen::Vector2d d01 = control_points_[1] - control_points_[0];
    Eigen::Vector2d d12 = control_points_[2] - control_points_[1];
    Eigen::Vector2d d23 = control_points_[3] - control_points_[2];
    
    return 3.0 * u * u * d01 + 6.0 * u * t * d12 + 3.0 * t * t * d23;
}

Eigen::Vector2d BezierSegment::getSecondDerivative(double t) const {
    // 二阶导数: B''(t) = 6*(1-t)*(P2-2*P1+P0) + 6*t*(P3-2*P2+P1)
    t = math::MathUtils::clamp(t, 0.0, 1.0);
    
    double u = 1.0 - t;
    
    Eigen::Vector2d dd0 = control_points_[2] - 2.0 * control_points_[1] + control_points_[0];
    Eigen::Vector2d dd1 = control_points_[3] - 2.0 * control_points_[2] + control_points_[1];
    
    return 6.0 * u * dd0 + 6.0 * t * dd1;
}

double BezierSegment::getHeadingAtParameter(double t) const {
    Eigen::Vector2d derivative = getFirstDerivative(t);
    if (derivative.norm() < 1e-9) {
        // 导数为零时返回终点航向
        return end_heading_;
    }
    return std::atan2(derivative(1), derivative(0));
}

double BezierSegment::getCurvatureAtParameter(double t) const {
    // 曲率公式: κ = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
    Eigen::Vector2d d1 = getFirstDerivative(t);
    Eigen::Vector2d d2 = getSecondDerivative(t);
    
    double cross = d1(0) * d2(1) - d1(1) * d2(0);
    double norm_cubed = std::pow(d1.squaredNorm(), 1.5);
    
    if (norm_cubed < 1e-9) {
        return 0.0;
    }
    
    return cross / norm_cubed;
}

double BezierSegment::getParameterAtArcLength(double arc_length) const {
    // 二分查找弧长对应的参数t
    // 对应原代码中的弧长查找逻辑
    
    arc_length = std::abs(arc_length);
    arc_length = math::MathUtils::clamp(arc_length, 0.0, total_length_);
    
    // 使用lower_bound查找
    auto it = std::lower_bound(arc_length_table_.begin(), arc_length_table_.end(), arc_length);
    int idx = std::distance(arc_length_table_.begin(), it);
    
    if (idx == 0) idx = 1;
    if (idx >= kTableResolution) idx = kTableResolution - 1;
    
    // 线性插值
    double s0 = arc_length_table_[idx - 1];
    double s1 = arc_length_table_[idx];
    double t0 = t_table_[idx - 1];
    double t1 = t_table_[idx];
    
    double ds = s1 - s0;
    if (ds < 1e-10) {
        return t0;
    }
    
    double t_interp = t0 + (t1 - t0) * (arc_length - s0) / ds;
    return math::MathUtils::clamp(t_interp, 0.0, 1.0);
}

Eigen::Vector2d BezierSegment::getPointAtArcLength(double arc_length) const {
    double t = getParameterAtArcLength(arc_length);
    return getPointAtParameter(t);
}

double BezierSegment::getHeadingAtArcLength(double arc_length) const {
    double t = getParameterAtArcLength(arc_length);
    return getHeadingAtParameter(t);
}

Pose2D BezierSegment::getStartPose() const {
    return Pose2D(control_points_[0](0), control_points_[0](1), start_heading_);
}

Pose2D BezierSegment::getEndPose() const {
    return Pose2D(control_points_[3](0), control_points_[3](1), end_heading_);
}

double BezierSegment::getRemainingDistance(const Pose2D& current_pose) const {
    // 简单实现：直接计算到终点的欧氏距离
    //TODO 更精确的实现可以查找最近点然后计算剩余弧长
    Eigen::Vector2d current(current_pose.x, current_pose.y);
    return (current - control_points_[3]).norm();
}

bool BezierSegment::isReached(const Pose2D& current_pose, double distance_threshold) const {
    // 两种判断方式：距离到达或已越过终点
    if (getRemainingDistance(current_pose) < distance_threshold) {
        return true;
    }
    
    // 判断是否越过终点：当前位置到终点的方向与曲线终点切线方向相差超过90度
    Eigen::Vector2d current(current_pose.x, current_pose.y);
    double theta_to_end = std::atan2(
        control_points_[3](1) - current(1),
        control_points_[3](0) - current(0));
    double angle_diff = math::MathUtils::absoluteAngleDiff(end_heading_, theta_to_end);
    
    return angle_diff >= M_PI / 2.0;
}

double BezierSegment::getLateralDeviation(const Pose2D& current_pose) const {
    // 查找曲线上最近点，计算横向偏离
    Eigen::Vector2d current(current_pose.x, current_pose.y);
    
    double min_distance = std::numeric_limits<double>::max();
    
    // 通过采样查找最近点
    const int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        Eigen::Vector2d point = getPointAtParameter(t);
        double dist = (current - point).norm();
        if (dist < min_distance) {
            min_distance = dist;
        }
    }
    
    return min_distance;
}

}  // namespace laser_navigation
