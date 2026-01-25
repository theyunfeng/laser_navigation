/**
 * @file bezier_segment.hpp
 * @brief 三阶贝塞尔曲线路径段
 * 
 * 实现三阶贝塞尔曲线类型的路径段
 */

#ifndef LASER_NAVIGATION_REFACTORED_PATH_BEZIER_SEGMENT_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_BEZIER_SEGMENT_HPP_

#include "laser_navigation_refactored/path/path_segment.hpp"
#include <vector>
#include <Eigen/Dense>

namespace laser_navigation {

/**
 * @brief 三阶贝塞尔曲线路径段
 * 
 * 对应原代码中 type=2 的 Line，以及 bezier.cpp 中的实现
 * 
 * 三阶贝塞尔曲线公式:
 * B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
 * 
 * 其中 t ∈ [0, 1]
 */
class BezierSegment : public PathSegment {
public:
    /**
     * @brief 构造函数
     * @param start 起点 P0
     * @param end 终点 P3
     * @param control_point1 第一控制点 P1
     * @param control_point2 第二控制点 P2
     * @param constraints 运动约束
     */
    BezierSegment(const Pose2D& start, const Pose2D& end,
                  const Pose2D& control_point1, const Pose2D& control_point2,
                  const MotionConstraints& constraints);

    //==========================================================================
    // PathSegment 接口实现
    //==========================================================================

    PathSegmentType getType() const override { return PathSegmentType::kCubicBezier; }
    double getTotalLength() const override { return total_length_; }
    Pose2D getStartPose() const override;
    Pose2D getEndPose() const override;
    double getStartHeading() const override { return start_heading_; }
    double getEndHeading() const override { return end_heading_; }
    double getRemainingDistance(const Pose2D& current_pose) const override;
    bool isReached(const Pose2D& current_pose, double distance_threshold) const override;
    double getLateralDeviation(const Pose2D& current_pose) const override;

    //==========================================================================
    // 贝塞尔曲线特有接口
    //==========================================================================

    /**
     * @brief 根据弧长获取对应的曲线点
     * @param arc_length 弧长 [0, total_length]
     * @return 曲线上的点
     * 
     * 对应原代码中 Bezier::BezierPoint 的弧长查找功能
     */
    Eigen::Vector2d getPointAtArcLength(double arc_length) const;

    /**
     * @brief 根据参数t获取曲线点
     * @param t 参数 [0, 1]
     * @return 曲线上的点
     * 
     * 对应原代码中 Bezier::BezierPoint(double t) 函数
     */
    Eigen::Vector2d getPointAtParameter(double t) const;

    /**
     * @brief 根据参数t获取曲线切线方向（航向角）
     * @param t 参数 [0, 1]
     * @return 切线航向角
     */
    double getHeadingAtParameter(double t) const;

    /**
     * @brief 根据参数t获取曲率
     * @param t 参数 [0, 1]
     * @return 曲率
     */
    double getCurvatureAtParameter(double t) const;

    /**
     * @brief 获取控制点
     * @return 控制点数组 [P0, P1, P2, P3]
     */
    const std::vector<Eigen::Vector2d>& getControlPoints() const { return control_points_; }

    /**
     * @brief 获取参数表（用于弧长-参数映射）
     */
    const std::vector<double>& getParameterTable() const { return t_table_; }

    /**
     * @brief 获取弧长表
     */
    const std::vector<double>& getArcLengthTable() const { return arc_length_table_; }

    /**
     * @brief 根据弧长获取对应的参数t
     * @param arc_length 弧长
     * @return 参数t
     */
    double getParameterAtArcLength(double arc_length) const;

    /**
     * @brief 根据弧长获取航向角
     * @param arc_length 弧长
     * @return 航向角
     */
    double getHeadingAtArcLength(double arc_length) const;

private:
    /**
     * @brief 计算弧长离散化表
     * 
     * 预计算弧长与参数t的对应关系，用于后续快速查找
     */
    void computeArcLengthTable();

    /**
     * @brief 计算一阶导数
     * @param t 参数
     * @return 一阶导数向量
     */
    Eigen::Vector2d getFirstDerivative(double t) const;

    /**
     * @brief 计算二阶导数
     * @param t 参数
     * @return 二阶导数向量
     */
    Eigen::Vector2d getSecondDerivative(double t) const;

    std::vector<Eigen::Vector2d> control_points_;  ///< 控制点 [P0, P1, P2, P3]
    double total_length_;      ///< 曲线总长度
    double start_heading_;     ///< 起点航向
    double end_heading_;       ///< 终点航向

    // 弧长-参数映射表（对应原代码中的 t_arr 和 s_arr）
    std::vector<double> t_table_;          ///< 参数表
    std::vector<double> arc_length_table_; ///< 弧长表

    static constexpr int kTableResolution = 100;  ///< 离散化分辨率，即表中采样点数量
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PATH_BEZIER_SEGMENT_HPP_
