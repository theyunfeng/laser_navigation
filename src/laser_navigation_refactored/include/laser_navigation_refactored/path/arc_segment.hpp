/**
 * @file arc_segment.hpp
 * @brief 圆弧路径段实现
 * 
 * 支持圆弧轨迹，包括：
 * - 简单圆弧（给定圆心、半径、起止角度）
 * - Dubins曲线（前进方向）
 * - Reeds-Shepp曲线（支持倒车）
 * 
 * 参考: PythonRobotics中的dubins_path_planner.py和reeds_shepp_path_planning.py
 */

#ifndef LASER_NAVIGATION_REFACTORED_PATH_ARC_SEGMENT_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_ARC_SEGMENT_HPP_

#include "laser_navigation_refactored/path/path_segment.hpp"
#include "laser_navigation_refactored/core/math_utils.hpp"
#include <vector>
#include <cmath>

namespace laser_navigation {

//==============================================================================
// 圆弧类型枚举
//==============================================================================

/**
 * @brief 圆弧转向方向
 */
enum class ArcDirection {
    kLeft,      ///< 左转 (逆时针)
    kRight,     ///< 右转 (顺时针)
    kStraight   ///< 直行段（用于复合路径）
};

/**
 * @brief 圆弧子段信息（用于Dubins/Reeds-Shepp）
 */
struct ArcSubSegment {
    ArcDirection direction;    ///< 转向方向
    double length;             ///< 弧长或直线长度
    bool is_forward;           ///< 是否前进（Reeds-Shepp需要）
    
    ArcSubSegment(ArcDirection dir = ArcDirection::kStraight, 
                  double len = 0.0, bool forward = true)
        : direction(dir), length(len), is_forward(forward) {}
};

//==============================================================================
// 圆弧路径段基类
//==============================================================================

/**
 * @brief 圆弧路径段
 * 
 * 支持简单圆弧和复合圆弧路径（如Dubins、Reeds-Shepp）
 * 
 * 使用示例:
 * @code
 * // 创建简单圆弧
 * ArcSegment arc(center, radius, start_angle, end_angle, ArcDirection::kLeft);
 * 
 * // 创建Dubins路径
 * auto dubins = ArcSegment::createDubins(start, goal, curvature);
 * 
 * // 创建Reeds-Shepp路径
 * auto rs = ArcSegment::createReedsShepp(start, goal, curvature);
 * @endcode
 */
class ArcSegment : public PathSegment {
public:
    //==========================================================================
    // 构造函数
    //==========================================================================

    /**
     * @brief 简单圆弧构造函数
     * @param center 圆心
     * @param radius 半径
     * @param start_angle 起始角度 (rad)
     * @param end_angle 终止角度 (rad)
     * @param direction 转向方向
     */
    ArcSegment(const Eigen::Vector2d& center, double radius,
               double start_angle, double end_angle,
               ArcDirection direction = ArcDirection::kLeft);

    /**
     * @brief 从起终点和曲率构造圆弧
     * @param start 起点位姿
     * @param end 终点位姿
     * @param curvature 曲率 (1/m)，正值左转，负值右转
     */
    ArcSegment(const Pose2D& start, const Pose2D& end, double curvature);

    /**
     * @brief 三点确定圆弧构造函数
     * @param start 起点位姿
     * @param mid_point 圆弧上的中间点（控制点）
     * @param end 终点位姿
     * 
     * 通过三点唯一确定一个圆弧。中间点必须在圆弧上。
     * 如果三点共线，将退化为直线处理。
     */
    ArcSegment(const Pose2D& start, const Pose2D& mid_point, const Pose2D& end);

    /**
     * @brief 复合路径构造函数（用于Dubins/Reeds-Shepp）
     * @param start 起点位姿
     * @param sub_segments 子段列表
     * @param curvature 曲率
     */
    ArcSegment(const Pose2D& start, 
               const std::vector<ArcSubSegment>& sub_segments,
               double curvature);

    virtual ~ArcSegment() = default;

    //==========================================================================
    // 工厂方法
    //==========================================================================

    /**
     * @brief 通过三点创建圆弧（推荐方式）
     * @param start 起点位姿
     * @param mid_point 圆弧上的中间点（控制点）
     * @param end 终点位姿
     * @return 圆弧段指针，如果三点共线返回nullptr
     */
    static std::unique_ptr<ArcSegment> createFromThreePoints(
        const Pose2D& start, const Pose2D& mid_point, const Pose2D& end);

    /**
     * @brief 创建Dubins路径（仅前进）
     * @param start 起点位姿
     * @param goal 终点位姿
     * @param curvature 最大曲率 (1/m)
     * @param step_size 插值步长 (m)
     * @return 圆弧段指针
     * 
     * Dubins路径是连接两个有向点的最短前进路径，
     * 由最多3段组成：圆弧-直线-圆弧 (CSC) 或 圆弧-圆弧-圆弧 (CCC)
     */
    static std::unique_ptr<ArcSegment> createDubins(
        const Pose2D& start, const Pose2D& goal, 
        double curvature, double step_size = 0.1);

    /**
     * @brief 创建Reeds-Shepp路径（支持倒车）
     * @param start 起点位姿
     * @param goal 终点位姿
     * @param curvature 最大曲率 (1/m)
     * @param step_size 插值步长 (m)
     * @return 圆弧段指针
     * 
     * Reeds-Shepp路径允许前进和后退，是汽车式机器人的最短路径
     */
    static std::unique_ptr<ArcSegment> createReedsShepp(
        const Pose2D& start, const Pose2D& goal,
        double curvature, double step_size = 0.1);

    //==========================================================================
    // PathSegment 接口实现
    //==========================================================================

    PathSegmentType getType() const override { return PathSegmentType::kCircularArc; }
    
    double getTotalLength() const override { return total_length_; }
    
    Pose2D getStartPose() const override { return start_pose_; }
    
    Pose2D getEndPose() const override { return end_pose_; }
    
    double getStartHeading() const override { return start_pose_.yaw; }
    
    double getEndHeading() const override { return end_pose_.yaw; }
    
    double getRemainingDistance(const Pose2D& current_pose) const override;
    
    bool isReached(const Pose2D& current_pose, double distance_threshold) const override;
    
    double getLateralDeviation(const Pose2D& current_pose) const override;

    //==========================================================================
    // 圆弧特有方法
    //==========================================================================

    /**
     * @brief 获取圆心（简单圆弧）
     */
    Eigen::Vector2d getCenter() const { return center_; }

    /**
     * @brief 获取半径
     */
    double getRadius() const { return radius_; }

    /**
     * @brief 获取曲率 (1/radius)
     */
    double getCurvature() const { return curvature_; }

    /**
     * @brief 获取转向方向
     */
    ArcDirection getDirection() const { return direction_; }

    /**
     * @brief 获取子段列表（复合路径）
     */
    const std::vector<ArcSubSegment>& getSubSegments() const { return sub_segments_; }

    /**
     * @brief 是否为复合路径（Dubins/Reeds-Shepp）
     */
    bool isCompound() const { return is_compound_; }

    /**
     * @brief 获取插值点列表
     */
    const std::vector<Pose2D>& getInterpolatedPath() const { return interpolated_path_; }

    /**
     * @brief 获取方向列表（1=前进，-1=后退）
     */
    const std::vector<int>& getDirections() const { return directions_; }

    /**
     * @brief 根据弧长参数获取位姿
     * @param s 弧长参数 [0, total_length]
     * @return 对应位姿
     */
    Pose2D getPoseAtArcLength(double s) const;

    /**
     * @brief 获取最近点的弧长参数
     * @param current_pose 当前位姿
     * @return 弧长参数
     */
    double getArcLengthAtPose(const Pose2D& current_pose) const;

    /**
     * @brief 获取当前行进方向（1=前进，-1=后退）
     * @param s 弧长参数
     */
    int getDirectionAtArcLength(double s) const;

private:
    // 简单圆弧参数
    Eigen::Vector2d center_;        ///< 圆心
    double radius_;                 ///< 半径
    double start_angle_;            ///< 起始角度
    double end_angle_;              ///< 终止角度
    double curvature_;              ///< 曲率 (1/radius)
    ArcDirection direction_;        ///< 转向方向

    // 复合路径参数
    bool is_compound_;                          ///< 是否为复合路径
    std::vector<ArcSubSegment> sub_segments_;   ///< 子段列表
    std::vector<double> sub_segment_lengths_;   ///< 累计长度
    
    // 通用参数
    Pose2D start_pose_;             ///< 起点位姿
    Pose2D end_pose_;               ///< 终点位姿
    double total_length_;           ///< 总长度
    
    // 预计算的插值路径（用于快速查询）
    std::vector<Pose2D> interpolated_path_;  ///< 插值点
    std::vector<int> directions_;            ///< 每个点的方向

    //==========================================================================
    // 私有辅助方法
    //==========================================================================

    /**
     * @brief 计算简单圆弧的位姿
     */
    Pose2D computeSimpleArcPose(double s) const;

    /**
     * @brief 计算复合路径的位姿
     */
    Pose2D computeCompoundPose(double s) const;

    /**
     * @brief 计算子段终点位姿
     */
    Pose2D computeSubSegmentEndPose(const Pose2D& origin, 
                                     const ArcSubSegment& seg) const;

    /**
     * @brief 在子段中插值
     */
    Pose2D interpolateInSubSegment(const Pose2D& origin,
                                    const ArcSubSegment& seg,
                                    double s) const;

    /**
     * @brief 生成插值路径
     */
    void generateInterpolatedPath(double step_size);

    /**
     * @brief 计算简单圆弧总长度
     */
    void computeSimpleArcLength();

    //==========================================================================
    // Dubins路径计算 (参考 dubins_path_planner.py)
    //==========================================================================

    static bool dubinsLSL(double alpha, double beta, double d,
                          double& t, double& p, double& q);
    static bool dubinsRSR(double alpha, double beta, double d,
                          double& t, double& p, double& q);
    static bool dubinsLSR(double alpha, double beta, double d,
                          double& t, double& p, double& q);
    static bool dubinsRSL(double alpha, double beta, double d,
                          double& t, double& p, double& q);
    static bool dubinsRLR(double alpha, double beta, double d,
                          double& t, double& p, double& q);
    static bool dubinsLRL(double alpha, double beta, double d,
                          double& t, double& p, double& q);

    //==========================================================================
    // Reeds-Shepp路径计算 (参考 reeds_shepp_path_planning.py)
    //==========================================================================

    struct RSPath {
        std::vector<double> lengths;
        std::vector<char> ctypes;  // 'L', 'R', 'S'
        double total_length;
    };

    static std::vector<RSPath> generateRSPaths(double x, double y, double phi,
                                                double step_size);
    
    static bool rsLSL(double x, double y, double phi, RSPath& path);
    static bool rsLSR(double x, double y, double phi, RSPath& path);
    static bool rsLXRXL(double x, double y, double phi, RSPath& path);
    static bool rsLXRL(double x, double y, double phi, RSPath& path);
    static bool rsLRXL(double x, double y, double phi, RSPath& path);
    // ... 其他RS路径类型

    static double mod2pi(double x);
    static void polar(double x, double y, double& r, double& theta);
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PATH_ARC_SEGMENT_HPP_
