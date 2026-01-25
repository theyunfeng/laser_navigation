/**
 * @file arc_segment.cpp
 * @brief 圆弧路径段实现
 * 
 * 包含:
 * - 简单圆弧计算
 * - Dubins路径规划
 * - Reeds-Shepp路径规划
 */

#include "laser_navigation_refactored/path/arc_segment.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include <algorithm>
#include <limits>

namespace laser_navigation {

//==============================================================================
// 构造函数
//==============================================================================

ArcSegment::ArcSegment(const Eigen::Vector2d& center, double radius,
                       double start_angle, double end_angle,
                       ArcDirection direction)
    : center_(center)
    , radius_(radius)
    , start_angle_(start_angle)
    , end_angle_(end_angle)
    , curvature_(1.0 / radius)
    , direction_(direction)
    , is_compound_(false)
    , total_length_(0.0) {
    
    // 计算起终点位姿
    start_pose_.x = center_.x() + radius_ * std::cos(start_angle_);
    start_pose_.y = center_.y() + radius_ * std::sin(start_angle_);
    
    end_pose_.x = center_.x() + radius_ * std::cos(end_angle_);
    end_pose_.y = center_.y() + radius_ * std::sin(end_angle_);
    
    // 切线方向作为航向
    if (direction_ == ArcDirection::kLeft) {
        start_pose_.yaw = math::MathUtils::normalizeAngle(start_angle_ + M_PI / 2);
        end_pose_.yaw = math::MathUtils::normalizeAngle(end_angle_ + M_PI / 2);
    } else {
        start_pose_.yaw = math::MathUtils::normalizeAngle(start_angle_ - M_PI / 2);
        end_pose_.yaw = math::MathUtils::normalizeAngle(end_angle_ - M_PI / 2);
    }
    
    computeSimpleArcLength();
    generateInterpolatedPath(0.05);
}

ArcSegment::ArcSegment(const Pose2D& start, const Pose2D& end, double curvature)
    : curvature_(curvature)
    , is_compound_(false)
    , start_pose_(start)
    , end_pose_(end)
    , total_length_(0.0) {
    
    radius_ = std::abs(1.0 / curvature);
    direction_ = (curvature > 0) ? ArcDirection::kLeft : ArcDirection::kRight;
    
    // 根据起终点和曲率计算圆心
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double chord = std::hypot(dx, dy);
    
    if (chord > 2.0 * radius_) {
        // 无法形成圆弧，退化为直线
        total_length_ = chord;
        center_ = Eigen::Vector2d((start.x + end.x) / 2, (start.y + end.y) / 2);
        start_angle_ = 0;
        end_angle_ = 0;
    } else {
        // 计算圆心
        double h = std::sqrt(radius_ * radius_ - chord * chord / 4);
        double mx = (start.x + end.x) / 2;
        double my = (start.y + end.y) / 2;
        double nx = -dy / chord;
        double ny = dx / chord;
        
        if (direction_ == ArcDirection::kLeft) {
            center_ = Eigen::Vector2d(mx + h * nx, my + h * ny);
        } else {
            center_ = Eigen::Vector2d(mx - h * nx, my - h * ny);
        }
        
        start_angle_ = std::atan2(start.y - center_.y(), start.x - center_.x());
        end_angle_ = std::atan2(end.y - center_.y(), end.x - center_.x());
        
        computeSimpleArcLength();
    }
    
    generateInterpolatedPath(0.05);
}

// 三点确定圆弧构造函数
ArcSegment::ArcSegment(const Pose2D& start, const Pose2D& mid_point, const Pose2D& end)
    : is_compound_(false)
    , start_pose_(start)
    , end_pose_(end)
    , total_length_(0.0) {
    
    // 通过三点计算圆心和半径
    // 使用外接圆公式：三点确定一个圆
    double x1 = start.x, y1 = start.y;
    double x2 = mid_point.x, y2 = mid_point.y;
    double x3 = end.x, y3 = end.y;
    
    // 计算行列式 D = 2 * |x1-x3  y1-y3|
    //                   |x2-x3  y2-y3|
    double D = 2.0 * ((x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3));
    
    if (std::abs(D) < 1e-9) {
        // 三点共线，退化为直线处理
        // 设置一个很大的半径模拟直线
        radius_ = 1e6;
        curvature_ = 1e-6;
        center_ = Eigen::Vector2d((x1 + x3) / 2, (y1 + y3) / 2);
        direction_ = ArcDirection::kStraight;
        total_length_ = std::hypot(x3 - x1, y3 - y1);
        
        // 航向角为起点到终点的方向
        double heading = std::atan2(y3 - y1, x3 - x1);
        start_pose_.yaw = heading;
        end_pose_.yaw = heading;
        
        start_angle_ = 0;
        end_angle_ = 0;
    } else {
        // 计算圆心
        double A1 = x1 * x1 + y1 * y1;
        double A2 = x2 * x2 + y2 * y2;
        double A3 = x3 * x3 + y3 * y3;
        
        double cx = ((A1 - A3) * (y2 - y3) - (A2 - A3) * (y1 - y3)) / D;
        double cy = ((A2 - A3) * (x1 - x3) - (A1 - A3) * (x2 - x3)) / D;
        
        center_ = Eigen::Vector2d(cx, cy);
        radius_ = std::hypot(x1 - cx, y1 - cy);
        curvature_ = 1.0 / radius_;
        
        // 计算角度
        start_angle_ = std::atan2(y1 - cy, x1 - cx);
        double mid_angle = std::atan2(y2 - cy, x2 - cx);
        end_angle_ = std::atan2(y3 - cy, x3 - cx);
        
        // 判断转向方向：检查中间点是否在左转还是右转的弧上
        // 通过叉积判断
        double cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        direction_ = (cross > 0) ? ArcDirection::kLeft : ArcDirection::kRight;
        
        // 如果曲率为负（右转），调整符号
        if (direction_ == ArcDirection::kRight) {
            curvature_ = -curvature_;
        }
        
        // 计算切线方向作为航向角
        if (direction_ == ArcDirection::kLeft) {
            start_pose_.yaw = math::MathUtils::normalizeAngle(start_angle_ + M_PI / 2);
            end_pose_.yaw = math::MathUtils::normalizeAngle(end_angle_ + M_PI / 2);
        } else {
            start_pose_.yaw = math::MathUtils::normalizeAngle(start_angle_ - M_PI / 2);
            end_pose_.yaw = math::MathUtils::normalizeAngle(end_angle_ - M_PI / 2);
        }
        
        // 验证中间点确实在圆弧上（允许小误差）
        double mid_dist = std::hypot(x2 - cx, y2 - cy);
        if (std::abs(mid_dist - radius_) > 0.01) {
            // 中间点不在圆上，可能是输入错误，但仍然使用计算的圆弧
        }
        
        computeSimpleArcLength();
    }
    
    generateInterpolatedPath(0.05);
}

// 三点创建圆弧的工厂方法
std::unique_ptr<ArcSegment> ArcSegment::createFromThreePoints(
    const Pose2D& start, const Pose2D& mid_point, const Pose2D& end) {
    
    // 检查三点是否共线
    double x1 = start.x, y1 = start.y;
    double x2 = mid_point.x, y2 = mid_point.y;
    double x3 = end.x, y3 = end.y;
    
    double D = 2.0 * ((x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3));
    
    if (std::abs(D) < 1e-9) {
        // 三点共线，返回nullptr，调用者应该使用直线段
        return nullptr;
    }
    
    return std::unique_ptr<ArcSegment>(new ArcSegment(start, mid_point, end));
}

ArcSegment::ArcSegment(const Pose2D& start, 
                       const std::vector<ArcSubSegment>& sub_segments,
                       double curvature)
    : curvature_(curvature)
    , is_compound_(true)
    , sub_segments_(sub_segments)
    , start_pose_(start)
    , total_length_(0.0) {
    
    radius_ = std::abs(1.0 / curvature);
    direction_ = ArcDirection::kLeft;  // 复合路径不使用单一方向
    
    // 计算总长度和累计长度
    sub_segment_lengths_.push_back(0.0);
    for (const auto& seg : sub_segments_) {
        total_length_ += std::abs(seg.length);
        sub_segment_lengths_.push_back(total_length_);
    }
    
    // 计算终点位姿
    end_pose_ = computeCompoundPose(total_length_);
    
    generateInterpolatedPath(0.05);
}

//==============================================================================
// 简单圆弧计算
//==============================================================================

void ArcSegment::computeSimpleArcLength() {
    double angle_diff = end_angle_ - start_angle_;
    
    if (direction_ == ArcDirection::kLeft) {
        if (angle_diff < 0) angle_diff += 2 * M_PI;
    } else {
        if (angle_diff > 0) angle_diff -= 2 * M_PI;
    }
    
    total_length_ = std::abs(angle_diff) * radius_;
}

Pose2D ArcSegment::computeSimpleArcPose(double s) const {
    Pose2D pose;
    double angle_traveled = s / radius_;
    double current_angle;
    
    if (direction_ == ArcDirection::kLeft) {
        current_angle = start_angle_ + angle_traveled;
        pose.yaw = math::MathUtils::normalizeAngle(current_angle + M_PI / 2);
    } else {
        current_angle = start_angle_ - angle_traveled;
        pose.yaw = math::MathUtils::normalizeAngle(current_angle - M_PI / 2);
    }
    
    pose.x = center_.x() + radius_ * std::cos(current_angle);
    pose.y = center_.y() + radius_ * std::sin(current_angle);
    
    return pose;
}

//==============================================================================
// 复合路径计算
//==============================================================================

Pose2D ArcSegment::computeCompoundPose(double s) const {
    if (!is_compound_) {
        return computeSimpleArcPose(s);
    }
    
    // 找到当前所在的子段
    size_t seg_idx = 0;
    for (size_t i = 0; i < sub_segment_lengths_.size() - 1; ++i) {
        if (s <= sub_segment_lengths_[i + 1]) {
            seg_idx = i;
            break;
        }
    }
    if (seg_idx >= sub_segments_.size()) {
        seg_idx = sub_segments_.size() - 1;
    }
    
    // 计算在当前子段中的位置
    double local_s = s - sub_segment_lengths_[seg_idx];
    const auto& seg = sub_segments_[seg_idx];
    
    // 获取子段起点（通过从头递推）
    Pose2D origin = start_pose_;
    for (size_t i = 0; i < seg_idx; ++i) {
        origin = computeSubSegmentEndPose(origin, sub_segments_[i]);
    }
    
    // 在子段中插值
    return interpolateInSubSegment(origin, seg, local_s);
}

Pose2D ArcSegment::computeSubSegmentEndPose(const Pose2D& origin, 
                                             const ArcSubSegment& seg) const {
    double length = seg.is_forward ? seg.length : -seg.length;
    
    if (seg.direction == ArcDirection::kStraight) {
        // 直线段
        Pose2D end;
        end.x = origin.x + length / curvature_ * std::cos(origin.yaw);
        end.y = origin.y + length / curvature_ * std::sin(origin.yaw);
        end.yaw = origin.yaw;
        return end;
    } else {
        // 圆弧段
        double ldx = std::sin(length) / curvature_;
        double ldy = 0.0;
        
        if (seg.direction == ArcDirection::kLeft) {
            ldy = (1.0 - std::cos(length)) / curvature_;
        } else {
            ldy = (1.0 - std::cos(length)) / (-curvature_);
        }
        
        double gdx = std::cos(-origin.yaw) * ldx + std::sin(-origin.yaw) * ldy;
        double gdy = -std::sin(-origin.yaw) * ldx + std::cos(-origin.yaw) * ldy;
        
        Pose2D end;
        end.x = origin.x + gdx;
        end.y = origin.y + gdy;
        
        if (seg.direction == ArcDirection::kLeft) {
            end.yaw = origin.yaw + length;
        } else {
            end.yaw = origin.yaw - length;
        }
        
        return end;
    }
}

Pose2D ArcSegment::interpolateInSubSegment(const Pose2D& origin,
                                            const ArcSubSegment& seg,
                                            double s) const {
    double dist = seg.is_forward ? s : -s;
    
    if (seg.direction == ArcDirection::kStraight) {
        Pose2D pose;
        pose.x = origin.x + dist / curvature_ * std::cos(origin.yaw);
        pose.y = origin.y + dist / curvature_ * std::sin(origin.yaw);
        pose.yaw = origin.yaw;
        return pose;
    } else {
        double ldx = std::sin(dist) / curvature_;
        double ldy = 0.0;
        
        if (seg.direction == ArcDirection::kLeft) {
            ldy = (1.0 - std::cos(dist)) / curvature_;
        } else {
            ldy = (1.0 - std::cos(dist)) / (-curvature_);
        }
        
        double gdx = std::cos(-origin.yaw) * ldx + std::sin(-origin.yaw) * ldy;
        double gdy = -std::sin(-origin.yaw) * ldx + std::cos(-origin.yaw) * ldy;
        
        Pose2D pose;
        pose.x = origin.x + gdx;
        pose.y = origin.y + gdy;
        
        if (seg.direction == ArcDirection::kLeft) {
            pose.yaw = origin.yaw + dist;
        } else {
            pose.yaw = origin.yaw - dist;
        }
        
        return pose;
    }
}

//==============================================================================
// 生成插值路径
//==============================================================================

void ArcSegment::generateInterpolatedPath(double step_size) {
    interpolated_path_.clear();
    directions_.clear();
    
    double s = 0.0;
    while (s <= total_length_) {
        interpolated_path_.push_back(getPoseAtArcLength(s));
        directions_.push_back(getDirectionAtArcLength(s));
        s += step_size;
    }
    
    // 确保终点在列表中
    if (interpolated_path_.empty() || 
        interpolated_path_.back().distanceTo(end_pose_) > 1e-6) {
        interpolated_path_.push_back(end_pose_);
        directions_.push_back(1);
    }
}

//==============================================================================
// PathSegment 接口实现
//==============================================================================

Pose2D ArcSegment::getPoseAtArcLength(double s) const {
    s = std::max(0.0, std::min(s, total_length_));
    
    if (is_compound_) {
        return computeCompoundPose(s);
    } else {
        return computeSimpleArcPose(s);
    }
}

double ArcSegment::getArcLengthAtPose(const Pose2D& current_pose) const {
    // 在插值路径中找最近点
    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    
    for (size_t i = 0; i < interpolated_path_.size(); ++i) {
        double dist = current_pose.distanceTo(interpolated_path_[i]);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    
    // 返回对应的弧长
    double step_size = total_length_ / (interpolated_path_.size() - 1);
    return min_idx * step_size;
}

int ArcSegment::getDirectionAtArcLength(double s) const {
    if (!is_compound_) {
        return 1;  // 简单圆弧总是前进
    }
    
    // 找到当前所在的子段
    for (size_t i = 0; i < sub_segment_lengths_.size() - 1; ++i) {
        if (s <= sub_segment_lengths_[i + 1]) {
            return sub_segments_[i].is_forward ? 1 : -1;
        }
    }
    return 1;
}

double ArcSegment::getRemainingDistance(const Pose2D& current_pose) const {
    double s = getArcLengthAtPose(current_pose);
    return total_length_ - s;
}

bool ArcSegment::isReached(const Pose2D& current_pose, double distance_threshold) const {
    return current_pose.distanceTo(end_pose_) < distance_threshold;
}

double ArcSegment::getLateralDeviation(const Pose2D& current_pose) const {
    // 在插值路径中找最近点
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& pose : interpolated_path_) {
        double dist = current_pose.distanceTo(pose);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    
    return min_dist;
}

//==============================================================================
// Dubins路径工厂方法
//==============================================================================

std::unique_ptr<ArcSegment> ArcSegment::createDubins(
    const Pose2D& start, const Pose2D& goal, 
    double curvature, double step_size) {
    
    // 转换到局部坐标系
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double c = std::cos(start.yaw);
    double s = std::sin(start.yaw);
    
    double local_x = (c * dx + s * dy) * curvature;
    double local_y = (-s * dx + c * dy) * curvature;
    double local_yaw = goal.yaw - start.yaw;
    
    double d = std::hypot(local_x, local_y);
    double theta = mod2pi(std::atan2(local_y, local_x));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(local_yaw - theta);
    
    // 尝试所有Dubins路径类型，选择最短的
    double best_cost = std::numeric_limits<double>::max();
    double best_t = 0, best_p = 0, best_q = 0;
    std::vector<ArcDirection> best_types;
    
    // LSL
    double t, p, q;
    if (dubinsLSL(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kLeft, ArcDirection::kStraight, ArcDirection::kLeft};
        }
    }
    
    // RSR
    if (dubinsRSR(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kRight, ArcDirection::kStraight, ArcDirection::kRight};
        }
    }
    
    // LSR
    if (dubinsLSR(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kLeft, ArcDirection::kStraight, ArcDirection::kRight};
        }
    }
    
    // RSL
    if (dubinsRSL(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kRight, ArcDirection::kStraight, ArcDirection::kLeft};
        }
    }
    
    // RLR
    if (dubinsRLR(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kRight, ArcDirection::kLeft, ArcDirection::kRight};
        }
    }
    
    // LRL
    if (dubinsLRL(alpha, beta, d, t, p, q)) {
        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (cost < best_cost) {
            best_cost = cost;
            best_t = t; best_p = p; best_q = q;
            best_types = {ArcDirection::kLeft, ArcDirection::kRight, ArcDirection::kLeft};
        }
    }
    
    if (best_types.empty()) {
        return nullptr;  // 无法找到有效路径
    }
    
    // 构造子段
    std::vector<ArcSubSegment> sub_segments;
    std::vector<double> lengths = {best_t, best_p, best_q};
    
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(lengths[i]) > 1e-6) {
            sub_segments.emplace_back(best_types[i], lengths[i] / curvature, true);
        }
    }
    
    return std::unique_ptr<ArcSegment>(new ArcSegment(start, sub_segments, curvature));
}

//==============================================================================
// Dubins路径计算函数
//==============================================================================

double ArcSegment::mod2pi(double x) {
    double v = std::fmod(x, 2.0 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

void ArcSegment::polar(double x, double y, double& r, double& theta) {
    r = std::hypot(x, y);
    theta = std::atan2(y, x);
}

bool ArcSegment::dubinsLSL(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double p_squared = 2 + d * d - 2 * cos_ab + 2 * d * (sin_a - sin_b);
    if (p_squared < 0) return false;
    
    double tmp = std::atan2(cos_b - cos_a, d + sin_a - sin_b);
    t = mod2pi(-alpha + tmp);
    if (t < 0) t += 2 * M_PI;
    p = std::sqrt(p_squared);
    q = mod2pi(beta - tmp);
    if (q < 0) q += 2 * M_PI;
    
    return true;
}

bool ArcSegment::dubinsRSR(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double p_squared = 2 + d * d - 2 * cos_ab + 2 * d * (sin_b - sin_a);
    if (p_squared < 0) return false;
    
    double tmp = std::atan2(cos_a - cos_b, d - sin_a + sin_b);
    t = mod2pi(alpha - tmp);
    if (t < 0) t += 2 * M_PI;
    p = std::sqrt(p_squared);
    q = mod2pi(-beta + tmp);
    if (q < 0) q += 2 * M_PI;
    
    return true;
}

bool ArcSegment::dubinsLSR(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double p_squared = -2 + d * d + 2 * cos_ab + 2 * d * (sin_a + sin_b);
    if (p_squared < 0) return false;
    
    p = std::sqrt(p_squared);
    double tmp = std::atan2(-cos_a - cos_b, d + sin_a + sin_b) - std::atan2(-2.0, p);
    t = mod2pi(-alpha + tmp);
    if (t < 0) t += 2 * M_PI;
    q = mod2pi(-mod2pi(beta) + tmp);
    if (q < 0) q += 2 * M_PI;
    
    return true;
}

bool ArcSegment::dubinsRSL(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double p_squared = d * d - 2 + 2 * cos_ab - 2 * d * (sin_a + sin_b);
    if (p_squared < 0) return false;
    
    p = std::sqrt(p_squared);
    double tmp = std::atan2(cos_a + cos_b, d - sin_a - sin_b) - std::atan2(2.0, p);
    t = mod2pi(alpha - tmp);
    if (t < 0) t += 2 * M_PI;
    q = mod2pi(beta - tmp);
    if (q < 0) q += 2 * M_PI;
    
    return true;
}

bool ArcSegment::dubinsRLR(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double tmp = (6.0 - d * d + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0;
    if (std::abs(tmp) > 1.0) return false;
    
    p = mod2pi(2 * M_PI - std::acos(tmp));
    t = mod2pi(alpha - std::atan2(cos_a - cos_b, d - sin_a + sin_b) + p / 2.0);
    q = mod2pi(alpha - beta - t + p);
    
    return true;
}

bool ArcSegment::dubinsLRL(double alpha, double beta, double d,
                           double& t, double& p, double& q) {
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_ab = std::cos(alpha - beta);
    
    double tmp = (6.0 - d * d + 2.0 * cos_ab + 2.0 * d * (-sin_a + sin_b)) / 8.0;
    if (std::abs(tmp) > 1.0) return false;
    
    p = mod2pi(2 * M_PI - std::acos(tmp));
    t = mod2pi(-alpha - std::atan2(cos_a - cos_b, d + sin_a - sin_b) + p / 2.0);
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    
    return true;
}

//==============================================================================
// Reeds-Shepp路径工厂方法
//==============================================================================

std::unique_ptr<ArcSegment> ArcSegment::createReedsShepp(
    const Pose2D& start, const Pose2D& goal,
    double curvature, double step_size) {
    
    // 转换到局部坐标系
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double c = std::cos(start.yaw);
    double s = std::sin(start.yaw);
    
    double local_x = (c * dx + s * dy) * curvature;
    double local_y = (-s * dx + c * dy) * curvature;
    double local_yaw = goal.yaw - start.yaw;
    
    // 生成所有可能的RS路径
    auto paths = generateRSPaths(local_x, local_y, local_yaw, step_size * curvature);
    
    if (paths.empty()) {
        return nullptr;
    }
    
    // 选择最短路径
    auto best_it = std::min_element(paths.begin(), paths.end(),
        [](const RSPath& a, const RSPath& b) {
            return a.total_length < b.total_length;
        });
    
    const auto& best = *best_it;
    
    // 构造子段
    std::vector<ArcSubSegment> sub_segments;
    for (size_t i = 0; i < best.lengths.size(); ++i) {
        double len = best.lengths[i];
        ArcDirection dir;
        
        switch (best.ctypes[i]) {
            case 'L': dir = ArcDirection::kLeft; break;
            case 'R': dir = ArcDirection::kRight; break;
            default:  dir = ArcDirection::kStraight; break;
        }
        
        bool is_forward = (len >= 0);
        sub_segments.emplace_back(dir, std::abs(len) / curvature, is_forward);
    }
    
    return std::unique_ptr<ArcSegment>(new ArcSegment(start, sub_segments, curvature));
}

std::vector<ArcSegment::RSPath> ArcSegment::generateRSPaths(
    double x, double y, double phi, double step_size) {
    
    std::vector<RSPath> paths;
    
    // 辅助lambda：添加有效路径
    auto addPath = [&paths, step_size](bool valid, const RSPath& path) {
        if (valid && path.total_length > step_size) {
            // 检查是否已存在相同路径
            bool exists = false;
            for (const auto& p : paths) {
                if (p.ctypes == path.ctypes && 
                    std::abs(p.total_length - path.total_length) < step_size) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                paths.push_back(path);
            }
        }
    };
    
    RSPath path;
    
    // 尝试各种RS路径类型（正向）
    addPath(rsLSL(x, y, phi, path), path);
    addPath(rsLSR(x, y, phi, path), path);
    addPath(rsLXRXL(x, y, phi, path), path);
    addPath(rsLXRL(x, y, phi, path), path);
    addPath(rsLRXL(x, y, phi, path), path);
    
    // 时间反转（timeflip）：(x, y, phi) -> (-x, y, -phi)
    addPath(rsLSL(-x, y, -phi, path), path);
    addPath(rsLSR(-x, y, -phi, path), path);
    addPath(rsLXRXL(-x, y, -phi, path), path);
    addPath(rsLXRL(-x, y, -phi, path), path);
    addPath(rsLRXL(-x, y, -phi, path), path);
    
    // 反射（reflect）：(x, y, phi) -> (x, -y, -phi)，L<->R
    addPath(rsLSL(x, -y, -phi, path), path);
    addPath(rsLSR(x, -y, -phi, path), path);
    addPath(rsLXRXL(x, -y, -phi, path), path);
    addPath(rsLXRL(x, -y, -phi, path), path);
    addPath(rsLRXL(x, -y, -phi, path), path);
    
    // 时间反转 + 反射
    addPath(rsLSL(-x, -y, phi, path), path);
    addPath(rsLSR(-x, -y, phi, path), path);
    addPath(rsLXRXL(-x, -y, phi, path), path);
    addPath(rsLXRL(-x, -y, phi, path), path);
    addPath(rsLRXL(-x, -y, phi, path), path);
    
    return paths;
}

bool ArcSegment::rsLSL(double x, double y, double phi, RSPath& path) {
    double r, theta;
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), r, theta);
    
    if (theta >= 0.0 && theta <= M_PI) {
        double v = mod2pi(phi - theta);
        if (v >= 0.0 && v <= M_PI) {
            path.lengths = {theta, r, v};
            path.ctypes = {'L', 'S', 'L'};
            path.total_length = std::abs(theta) + std::abs(r) + std::abs(v);
            return true;
        }
    }
    return false;
}

bool ArcSegment::rsLSR(double x, double y, double phi, RSPath& path) {
    double r1, t1;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), r1, t1);
    double u1_sq = r1 * r1;
    
    if (u1_sq >= 4.0) {
        double u = std::sqrt(u1_sq - 4.0);
        double theta = std::atan2(2.0, u);
        double t = mod2pi(t1 + theta);
        double v = mod2pi(t - phi);
        
        if (t >= 0.0 && v >= 0.0) {
            path.lengths = {t, u, v};
            path.ctypes = {'L', 'S', 'R'};
            path.total_length = std::abs(t) + std::abs(u) + std::abs(v);
            return true;
        }
    }
    return false;
}

bool ArcSegment::rsLXRXL(double x, double y, double phi, RSPath& path) {
    double zeta = x - std::sin(phi);
    double eeta = y - 1.0 + std::cos(phi);
    double r, theta;
    polar(zeta, eeta, r, theta);
    
    if (r <= 4.0) {
        double A = std::acos(0.25 * r);
        double t = mod2pi(A + theta + M_PI / 2);
        double u = mod2pi(M_PI - 2 * A);
        double v = mod2pi(phi - t - u);
        
        path.lengths = {t, -u, v};
        path.ctypes = {'L', 'R', 'L'};
        path.total_length = std::abs(t) + std::abs(u) + std::abs(v);
        return true;
    }
    return false;
}

bool ArcSegment::rsLXRL(double x, double y, double phi, RSPath& path) {
    double zeta = x - std::sin(phi);
    double eeta = y - 1.0 + std::cos(phi);
    double r, theta;
    polar(zeta, eeta, r, theta);
    
    if (r <= 4.0) {
        double A = std::acos(0.25 * r);
        double t = mod2pi(A + theta + M_PI / 2);
        double u = mod2pi(M_PI - 2 * A);
        double v = mod2pi(-phi + t + u);
        
        path.lengths = {t, -u, -v};
        path.ctypes = {'L', 'R', 'L'};
        path.total_length = std::abs(t) + std::abs(u) + std::abs(v);
        return true;
    }
    return false;
}

bool ArcSegment::rsLRXL(double x, double y, double phi, RSPath& path) {
    double zeta = x - std::sin(phi);
    double eeta = y - 1.0 + std::cos(phi);
    double r, theta;
    polar(zeta, eeta, r, theta);
    
    if (r <= 4.0) {
        double u = std::acos(1.0 - r * r * 0.125);
        double A = std::asin(2.0 * std::sin(u) / r);
        double t = mod2pi(-A + theta + M_PI / 2);
        double v = mod2pi(t - u - phi);
        
        path.lengths = {t, u, -v};
        path.ctypes = {'L', 'R', 'L'};
        path.total_length = std::abs(t) + std::abs(u) + std::abs(v);
        return true;
    }
    return false;
}

}  // namespace laser_navigation
