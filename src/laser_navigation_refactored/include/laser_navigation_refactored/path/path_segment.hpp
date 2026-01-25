/**
 * @file path_segment.hpp
 * @brief 路径段抽象基类
 * 
 * 定义路径段的通用接口，具体实现由子类完成
 */

#ifndef LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include <memory>

namespace laser_navigation {

/**
 * @brief 路径段抽象基类
 * 
 * 定义路径段的通用接口，支持多种路径类型（直线、贝塞尔曲线等）
 * 对应原代码中 Line 结构体的功能，但通过多态实现不同类型
 */
class PathSegment {
public:
    virtual ~PathSegment() = default;

    //==========================================================================
    // 基本属性查询
    //==========================================================================

    /**
     * @brief 获取路径段类型
     */
    virtual PathSegmentType getType() const = 0;

    /**
     * @brief 获取路径总长度
     */
    virtual double getTotalLength() const = 0;

    /**
     * @brief 获取起点
     */
    virtual Pose2D getStartPose() const = 0;

    /**
     * @brief 获取终点
     */
    virtual Pose2D getEndPose() const = 0;

    /**
     * @brief 获取起始航向角
     */
    virtual double getStartHeading() const = 0;

    /**
     * @brief 获取终点航向角
     */
    virtual double getEndHeading() const = 0;

    //==========================================================================
    // 距离和状态查询
    //==========================================================================

    /**
     * @brief 根据当前位置计算剩余距离
     * @param current_pose 当前位姿
     * @return 到终点的剩余距离
     */
    virtual double getRemainingDistance(const Pose2D& current_pose) const = 0;

    /**
     * @brief 检查是否到达终点
     * @param current_pose 当前位姿
     * @param distance_threshold 距离阈值
     * @return true 如果已到达
     */
    virtual bool isReached(const Pose2D& current_pose, double distance_threshold) const = 0;

    /**
     * @brief 计算当前位置到路径的横向偏离距离
     * @param current_pose 当前位姿
     * @return 横向偏离距离（绝对值）
     * 
     * 用于实时检测路径偏离，触发重规划
     */
    virtual double getLateralDeviation(const Pose2D& current_pose) const = 0;

    //==========================================================================
    // 约束参数
    //==========================================================================

    /**
     * @brief 获取运动约束
     */
    const MotionConstraints& getConstraints() const { return constraints_; }

    /**
     * @brief 设置运动约束
     */
    void setConstraints(const MotionConstraints& constraints) { constraints_ = constraints; }

    /**
     * @brief 检查是否前进方向
     */
    bool isForward() const { return constraints_.is_forward; }

protected:
    MotionConstraints constraints_;  ///< 运动约束参数
};

/// 路径段智能指针类型
using PathSegmentPtr = std::shared_ptr<PathSegment>;
using PathSegmentConstPtr = std::shared_ptr<const PathSegment>;

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_HPP_
