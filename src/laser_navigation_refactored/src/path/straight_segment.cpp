/**
 * @file straight_segment.cpp
 * @brief 直线路径段实现
 */

#include "laser_navigation_refactored/path/straight_segment.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include <cmath>

namespace laser_navigation {

StraightSegment::StraightSegment(const Pose2D& start, const Pose2D& end,
                                 const MotionConstraints& constraints)
    : start_pose_(start)
    , end_pose_(end)
    , is_vertical_(false) 
{
    constraints_ = constraints;
    
    // 计算直线长度
    total_length_ = start.distanceTo(end);
    
    // 计算航向角（从起点指向终点）
    heading_ = std::atan2(end.y - start.y, end.x - start.x);
    
    // 如果是倒车，航向角需要加180度
    // 注意：这里保存的是路径方向，不是车辆朝向
    
    // 计算直线参数 y = kx + b
    double dx = end.x - start.x;
    if (std::abs(dx) < 1e-6) {
        // 垂直线
        is_vertical_ = true;
        line_k_ = 0;
        line_b_ = 0;
    } else {
        is_vertical_ = false;
        line_k_ = (end.y - start.y) / dx;
        line_b_ = end.y - line_k_ * end.x;
    }
}

double StraightSegment::getRemainingDistance(const Pose2D& current_pose) const {
    return current_pose.distanceTo(end_pose_);
}

bool StraightSegment::isReached(const Pose2D& current_pose, double distance_threshold) const {
    // 两种判断方式：距离到达或已越过终点
    return getRemainingDistance(current_pose) < distance_threshold || hasPassedEnd(current_pose);
}

bool StraightSegment::getLineParameters(double& k, double& b) const {
    if (is_vertical_) {
        return false;
    }
    k = line_k_;
    b = line_b_;
    return true;
}

bool StraightSegment::hasPassedEnd(const Pose2D& current_pose) const {
    // 计算从当前位置到终点的方向
    double theta_to_end = std::atan2(end_pose_.y - current_pose.y, 
                                      end_pose_.x - current_pose.x);
    
    // 如果该方向与路径方向相差超过90度，说明已经越过终点
    // 这对应原代码中的判断逻辑
    double angle_diff = math::MathUtils::absoluteAngleDiff(heading_, theta_to_end);
    return angle_diff >= M_PI / 2.0;
}

double StraightSegment::getLateralDeviation(const Pose2D& current_pose) const {
    // 横向偏离取绝对值
    return std::abs(getPerpendicularDistance(current_pose));
}

double StraightSegment::getPerpendicularDistance(const Pose2D& point) const {
    if (is_vertical_) {
        // 垂直线，横向误差就是x坐标差
        return point.x - start_pose_.x;
    }
    
    // 点到直线 ax + by + c = 0 的距离公式
    // 直线方程: kx - y + b = 0, 即 a=k, b=-1, c=line_b_
    // 距离 = (a*x0 + b*y0 + c) / sqrt(a^2 + b^2)
    double numerator = line_k_ * point.x - point.y + line_b_;
    double denominator = std::sqrt(line_k_ * line_k_ + 1);
    
    return numerator / denominator;
}

Pose2D StraightSegment::getProjection(const Pose2D& point) const {
    Pose2D projection;
    
    if (is_vertical_) {
        // 垂直线，投影点x坐标固定
        projection.x = start_pose_.x;
        projection.y = point.y;
    } else {
        // 投影公式
        // 投影点 = (x - k*(kx - y + b)/(k^2+1), y + (kx - y + b)/(k^2+1))
        double denom = line_k_ * line_k_ + 1;
        projection.x = (point.x + line_k_ * (point.y - line_b_)) / denom;
        projection.y = (line_k_ * point.x + line_k_ * line_k_ * point.y + line_b_) / denom;
    }
    
    projection.yaw = heading_;
    return projection;
}

}  // namespace laser_navigation
