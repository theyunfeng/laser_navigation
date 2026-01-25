/**
 * @file straight_segment.hpp
 * @brief 直线路径段
 * 
 * 实现直线类型的路径段
 */

#ifndef LASER_NAVIGATION_REFACTORED_PATH_STRAIGHT_SEGMENT_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_STRAIGHT_SEGMENT_HPP_

#include "laser_navigation_refactored/path/path_segment.hpp"

namespace laser_navigation {

/**
 * @brief 直线路径段
 * 
 * 对应原代码中 type=0 的 Line
 */
class StraightSegment : public PathSegment {
public:
    /**
     * @brief 构造函数
     * @param start 起点
     * @param end 终点
     * @param constraints 运动约束
     */
    StraightSegment(const Pose2D& start, const Pose2D& end, 
                    const MotionConstraints& constraints);

    //==========================================================================
    // PathSegment 接口实现
    //==========================================================================

    PathSegmentType getType() const override { return PathSegmentType::kStraight; }
    double getTotalLength() const override { return total_length_; }
    Pose2D getStartPose() const override { return start_pose_; }
    Pose2D getEndPose() const override { return end_pose_; }
    double getStartHeading() const override { return heading_; }
    double getEndHeading() const override { return heading_; }
    double getRemainingDistance(const Pose2D& current_pose) const override;
    bool isReached(const Pose2D& current_pose, double distance_threshold) const override;
    double getLateralDeviation(const Pose2D& current_pose) const override;

    //==========================================================================
    // 直线段特有接口
    //==========================================================================

    /**
     * @brief 获取直线的斜率和截距
     * @param[out] k 斜率
     * @param[out] b 截距
     * @return true 如果斜率有效（非垂直线）
     * 
     * 对应原代码中计算 k, b 的逻辑
     */
    bool getLineParameters(double& k, double& b) const;

    /**
     * @brief 判断当前位置是否已越过终点
     * @param current_pose 当前位姿
     * @return true 如果已越过
     * 
     * 对应原代码中判断是否通过终点的逻辑
     */
    bool hasPassedEnd(const Pose2D& current_pose) const;

    /**
     * @brief 计算点到直线的垂直距离（横向误差）
     * @param point 查询点
     * @return 垂直距离（带符号）
     */
    double getPerpendicularDistance(const Pose2D& point) const;

    /**
     * @brief 计算点在直线上的投影点
     * @param point 查询点
     * @return 投影点
     */
    Pose2D getProjection(const Pose2D& point) const;

    /**
     * @brief 检查是否为垂直线
     */
    bool isVertical() const { return is_vertical_; }

    /**
     * @brief 获取航向角
     */
    double getHeading() const { return heading_; }

private:
    Pose2D start_pose_;     ///< 起点
    Pose2D end_pose_;       ///< 终点
    double total_length_;   ///< 总长度
    double heading_;        ///< 直线的航向角
    
    // 直线参数 y = kx + b
    bool is_vertical_;      ///< 是否为垂直线
    double line_k_;         ///< 斜率
    double line_b_;         ///< 截距
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PATH_STRAIGHT_SEGMENT_HPP_
