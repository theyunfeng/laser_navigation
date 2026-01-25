/**
 * @file bezier_control_point_generator.hpp
 * @brief 贝塞尔曲线控制点自动生成器
 * 
 * 根据路径衔接点的航向自动生成贝塞尔曲线控制点，
 * 实现直线-贝塞尔、贝塞尔-直线、贝塞尔-贝塞尔之间的平滑过渡
 * 
 * 控制点生成策略：
 * - P1 沿起点切线方向（上一段终点航向）延伸
 * - P2 沿终点切线反方向（下一段起点航向的反方向）延伸
 * - 延伸距离根据曲线长度自适应计算
 */

#ifndef LASER_NAVIGATION_REFACTORED_UTILS_BEZIER_CONTROL_POINT_GENERATOR_HPP_
#define LASER_NAVIGATION_REFACTORED_UTILS_BEZIER_CONTROL_POINT_GENERATOR_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include "laser_navigation_refactored/core/optional.hpp"
#include <cmath>
#include <vector>

namespace laser_navigation {

/**
 * @brief 相邻段的航向信息
 * 
 * 用于生成控制点时参考上下文
 */
struct AdjacentSegmentInfo {
    Optional<double> prev_end_heading;   ///< 上一段终点航向
    Optional<double> next_start_heading; ///< 下一段起点航向
    bool prev_is_forward{true};               ///< 上一段是否前进
    bool next_is_forward{true};               ///< 下一段是否前进
};

/**
 * @brief 贝塞尔控制点生成器
 * 
 * 自动计算三阶贝塞尔曲线的两个控制点 P1 和 P2
 */
class BezierControlPointGenerator {
public:
    /**
     * @brief 控制点延伸系数
     * 
     * 控制点到端点的距离 = 曲线弦长 × 系数
     * - 较小的系数产生更紧凑的曲线
     * - 较大的系数产生更平缓的曲线
     * 
     * 默认值 0.3-0.4 通常能产生视觉上平滑的曲线
     */
    static constexpr double kDefaultExtensionFactor = 0.35;
    
    /**
     * @brief 最小控制点距离（米）
     * 
     * 防止控制点距离过小导致曲线异常
     */
    static constexpr double kMinControlPointDistance = 0.05;
    
    /**
     * @brief 最大控制点距离（米）
     * 
     * 防止控制点距离过大导致曲线过于夸张
     */
    static constexpr double kMaxControlPointDistance = 2.0;

    BezierControlPointGenerator() = default;
    ~BezierControlPointGenerator() = default;

    /**
     * @brief 设置控制点延伸系数
     * @param factor 延伸系数 (0.1 ~ 0.6)
     */
    void setExtensionFactor(double factor) {
        extension_factor_ = std::max(0.1, std::min(0.6, factor));
    }

    /**
     * @brief 获取当前延伸系数
     */
    double getExtensionFactor() const { return extension_factor_; }

    /**
     * @brief 生成贝塞尔曲线控制点
     * 
     * @param start 起点 (P0)
     * @param end 终点 (P3)
     * @param adjacent_info 相邻段信息（可选）
     * @param is_forward 当前段是否前进方向
     * @return 控制点对 [P1, P2]
     * 
     * 生成逻辑：
     * 1. 如果有上一段航向，P1 沿该航向延伸
     * 2. 如果有下一段航向，P2 沿该航向反方向延伸
     * 3. 如果没有相邻信息，使用默认的平滑曲线生成
     */
    std::vector<Pose2D> generateControlPoints(
        const Pose2D& start,
        const Pose2D& end,
        const AdjacentSegmentInfo& adjacent_info = AdjacentSegmentInfo(),
        bool is_forward = true) const;

    /**
     * @brief 根据起点和终点的航向生成控制点
     * 
     * @param start 起点（包含位置和航向）
     * @param end 终点（包含位置和航向）
     * @param is_forward 是否前进方向
     * @return 控制点对 [P1, P2]
     * 
     * 使用起点的 yaw 作为出发切线方向
     * 使用终点的 yaw 作为到达切线方向
     */
    std::vector<Pose2D> generateControlPointsFromPose(
        const Pose2D& start,
        const Pose2D& end,
        bool is_forward = true) const;

    /**
     * @brief 为路径序列自动生成所有贝塞尔段的控制点
     * 
     * @param waypoints 路径点序列
     * @param segment_types 每段的类型
     * @param constraints 每段的约束
     * @param existing_control_points 已有的控制点（可部分为空）
     * @return 完整的控制点数组
     * 
     * 对于每个贝塞尔段：
     * - 如果已有控制点，保留原有的
     * - 如果没有，根据上下文自动生成
     */
    std::vector<std::vector<Pose2D>> generateAllControlPoints(
        const std::vector<Pose2D>& waypoints,
        const std::vector<PathSegmentType>& segment_types,
        const std::vector<MotionConstraints>& constraints,
        const std::vector<std::vector<Pose2D>>& existing_control_points) const;

private:
    /**
     * @brief 计算从点沿指定方向延伸的点
     * @param origin 原点
     * @param heading 方向（弧度）
     * @param distance 延伸距离
     * @return 延伸后的点
     */
    static Pose2D extendPoint(const Pose2D& origin, double heading, double distance);

    /**
     * @brief 计算两点之间的距离
     */
    static double distance(const Pose2D& p1, const Pose2D& p2);

    /**
     * @brief 计算两点之间的航向
     */
    static double heading(const Pose2D& from, const Pose2D& to);

    /**
     * @brief 计算控制点延伸距离
     * @param chord_length 弦长（起点到终点距离）
     * @return 控制点延伸距离
     */
    double computeExtensionDistance(double chord_length) const;

    double extension_factor_{kDefaultExtensionFactor};
};

//==============================================================================
// 内联实现
//==============================================================================

inline Pose2D BezierControlPointGenerator::extendPoint(
    const Pose2D& origin, double heading, double distance) {
    return Pose2D(
        origin.x + distance * std::cos(heading),
        origin.y + distance * std::sin(heading),
        heading
    );
}

inline double BezierControlPointGenerator::distance(const Pose2D& p1, const Pose2D& p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

inline double BezierControlPointGenerator::heading(const Pose2D& from, const Pose2D& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

inline double BezierControlPointGenerator::computeExtensionDistance(double chord_length) const {
    double dist = chord_length * extension_factor_;
    return std::max(kMinControlPointDistance, std::min(kMaxControlPointDistance, dist));
}

inline std::vector<Pose2D> BezierControlPointGenerator::generateControlPointsFromPose(
    const Pose2D& start,
    const Pose2D& end,
    bool is_forward) const {
    
    AdjacentSegmentInfo info;
    info.prev_end_heading = start.yaw;
    info.next_start_heading = end.yaw;
    info.prev_is_forward = is_forward;
    info.next_is_forward = is_forward;
    
    return generateControlPoints(start, end, info, is_forward);
}

inline std::vector<Pose2D> BezierControlPointGenerator::generateControlPoints(
    const Pose2D& start,
    const Pose2D& end,
    const AdjacentSegmentInfo& adjacent_info,
    bool is_forward) const {
    
    std::vector<Pose2D> control_points(2);
    
    // 计算弦长
    double chord_length = distance(start, end);
    double ext_dist = computeExtensionDistance(chord_length);
    
    // 计算默认航向（起点到终点方向）
    double default_heading = heading(start, end);
    
    // 确定 P1 的方向（起点切线方向）
    double p1_heading;
    if (adjacent_info.prev_end_heading.has_value()) {
        // 使用上一段终点航向
        p1_heading = adjacent_info.prev_end_heading.value();
        // 如果是倒车，需要反转航向
        if (!adjacent_info.prev_is_forward) {
            p1_heading += M_PI;
        }
    } else {
        // 没有上一段，使用默认方向或起点航向
        p1_heading = (start.yaw != 0.0) ? start.yaw : default_heading;
        if (!is_forward) {
            p1_heading += M_PI;
        }
    }
    
    // 确定 P2 的方向（终点切线反方向）
    double p2_heading;
    if (adjacent_info.next_start_heading.has_value()) {
        // 使用下一段起点航向的反方向
        p2_heading = adjacent_info.next_start_heading.value() + M_PI;
        // 如果是倒车，需要反转航向
        if (!adjacent_info.next_is_forward) {
            p2_heading += M_PI;
        }
    } else {
        // 没有下一段，使用默认方向或终点航向
        p2_heading = (end.yaw != 0.0) ? (end.yaw + M_PI) : (default_heading + M_PI);
        if (!is_forward) {
            p2_heading += M_PI;
        }
    }
    
    // 规范化航向角
    while (p1_heading > M_PI) p1_heading -= 2 * M_PI;
    while (p1_heading < -M_PI) p1_heading += 2 * M_PI;
    while (p2_heading > M_PI) p2_heading -= 2 * M_PI;
    while (p2_heading < -M_PI) p2_heading += 2 * M_PI;
    
    // 生成控制点
    // P1: 从起点沿切线方向延伸
    control_points[0] = extendPoint(start, p1_heading, ext_dist);
    
    // P2: 从终点沿切线反方向延伸（即向终点方向延伸）
    control_points[1] = extendPoint(end, p2_heading, ext_dist);
    
    return control_points;
}

inline std::vector<std::vector<Pose2D>> BezierControlPointGenerator::generateAllControlPoints(
    const std::vector<Pose2D>& waypoints,
    const std::vector<PathSegmentType>& segment_types,
    const std::vector<MotionConstraints>& constraints,
    const std::vector<std::vector<Pose2D>>& existing_control_points) const {
    
    if (waypoints.size() < 2) {
        return {};
    }
    
    size_t num_segments = waypoints.size() - 1;
    std::vector<std::vector<Pose2D>> result(num_segments);
    
    for (size_t i = 0; i < num_segments; ++i) {
        // 如果不是贝塞尔段，跳过
        if (i >= segment_types.size() || segment_types[i] != PathSegmentType::kCubicBezier) {
            continue;
        }
        
        // 如果已有控制点且有效，保留
        if (i < existing_control_points.size() && 
            existing_control_points[i].size() >= 2) {
            result[i] = existing_control_points[i];
            continue;
        }
        
        // 需要自动生成控制点
        const auto& start = waypoints[i];
        const auto& end = waypoints[i + 1];
        bool is_forward = (i < constraints.size()) ? constraints[i].is_forward : true;
        
        // 构建相邻段信息
        AdjacentSegmentInfo adj_info;
        adj_info.prev_is_forward = is_forward;
        adj_info.next_is_forward = (i + 1 < constraints.size()) ? 
            constraints[i + 1].is_forward : is_forward;
        
        // 获取上一段终点航向
        if (i > 0) {
            PathSegmentType prev_type = (i - 1 < segment_types.size()) ? 
                segment_types[i - 1] : PathSegmentType::kStraight;
            
            if (prev_type == PathSegmentType::kStraight) {
                // 直线段航向：起点到终点方向
                adj_info.prev_end_heading = heading(waypoints[i - 1], waypoints[i]);
            } else if (prev_type == PathSegmentType::kCubicBezier) {
                // 贝塞尔段终点航向：使用终点位姿的 yaw 或 P2->P3 方向
                if (i - 1 < result.size() && result[i - 1].size() >= 2) {
                    adj_info.prev_end_heading = heading(result[i - 1][1], waypoints[i]);
                } else {
                    adj_info.prev_end_heading = waypoints[i].yaw;
                }
            }
            adj_info.prev_is_forward = (i - 1 < constraints.size()) ? 
                constraints[i - 1].is_forward : true;
        } else {
            // 第一段，使用起点航向
            adj_info.prev_end_heading = start.yaw;
        }
        
        // 获取下一段起点航向
        if (i + 1 < num_segments) {
            PathSegmentType next_type = (i + 1 < segment_types.size()) ? 
                segment_types[i + 1] : PathSegmentType::kStraight;
            
            if (next_type == PathSegmentType::kStraight) {
                // 直线段航向：起点到终点方向
                adj_info.next_start_heading = heading(waypoints[i + 1], waypoints[i + 2]);
            } else {
                // 贝塞尔段，使用起点位姿的 yaw
                adj_info.next_start_heading = waypoints[i + 1].yaw;
            }
        } else {
            // 最后一段，使用终点航向
            adj_info.next_start_heading = end.yaw;
        }
        
        // 生成控制点
        result[i] = generateControlPoints(start, end, adj_info, is_forward);
    }
    
    return result;
}

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_UTILS_BEZIER_CONTROL_POINT_GENERATOR_HPP_
