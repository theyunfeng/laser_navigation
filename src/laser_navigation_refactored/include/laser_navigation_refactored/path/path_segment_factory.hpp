/**
 * @file path_segment_factory.hpp
 * @brief 路径段工厂类 - 简化新轨迹类型的添加
 * 
 * 使用说明：
 * 1. 在 PathSegmentType 枚举中添加新类型
 * 2. 创建继承自 PathSegment 的新类
 * 3. 在本工厂类中注册创建函数
 * 
 * 示例：添加新的 Clothoid (回旋曲线) 支持
 * @code
 * // 1. 在 types.hpp 中添加枚举
 * enum class PathSegmentType {
 *     ...
 *     kClothoid = 5,  // 回旋曲线
 * };
 * 
 * // 2. 创建 clothoid_segment.hpp/cpp
 * class ClothoidSegment : public PathSegment { ... };
 * 
 * // 3. 在工厂类中注册
 * factory.registerCreator(PathSegmentType::kClothoid, 
 *     [](const Pose2D& start, const Pose2D& end, 
 *        const MotionConstraints& constraints,
 *        const std::vector<Pose2D>& params) {
 *         double sharpness = params.empty() ? 0.1 : params[0].x;
 *         return std::make_shared<ClothoidSegment>(start, end, sharpness, constraints);
 *     });
 * @endcode
 */

#ifndef LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_FACTORY_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_FACTORY_HPP_

#include "laser_navigation_refactored/path/path_segment.hpp"
#include "laser_navigation_refactored/path/straight_segment.hpp"
#include "laser_navigation_refactored/path/bezier_segment.hpp"
#include "laser_navigation_refactored/path/arc_segment.hpp"
#include <functional>
#include <map>
#include <memory>

namespace laser_navigation {

/**
 * @brief 路径段创建函数类型
 * @param start 起点位姿
 * @param end 终点位姿
 * @param constraints 运动约束
 * @param params 额外参数（控制点、曲率等）
 * @return 路径段指针
 */
using PathSegmentCreator = std::function<std::shared_ptr<PathSegment>(
    const Pose2D& start,
    const Pose2D& end,
    const MotionConstraints& constraints,
    const std::vector<Pose2D>& params)>;

/**
 * @brief 路径段工厂类
 * 
 * 提供统一的路径段创建接口，支持动态注册新的轨迹类型
 */
class PathSegmentFactory {
public:
    /**
     * @brief 获取单例实例
     */
    static PathSegmentFactory& getInstance() {
        static PathSegmentFactory instance;
        return instance;
    }

    /**
     * @brief 注册路径段创建函数
     * @param type 路径段类型
     * @param creator 创建函数
     */
    void registerCreator(PathSegmentType type, PathSegmentCreator creator) {
        creators_[type] = creator;
    }

    /**
     * @brief 创建路径段
     * @param type 路径段类型
     * @param start 起点位姿
     * @param end 终点位姿
     * @param constraints 运动约束
     * @param params 额外参数
     * @return 路径段指针，失败返回nullptr
     */
    std::shared_ptr<PathSegment> create(
        PathSegmentType type,
        const Pose2D& start,
        const Pose2D& end,
        const MotionConstraints& constraints,
        const std::vector<Pose2D>& params = {}) {
        
        auto it = creators_.find(type);
        if (it != creators_.end()) {
            return it->second(start, end, constraints, params);
        }
        
        // 未知类型，回退到直线
        return std::make_shared<StraightSegment>(start, end, constraints);
    }

    /**
     * @brief 检查是否支持指定类型
     */
    bool isSupported(PathSegmentType type) const {
        return creators_.find(type) != creators_.end();
    }

    /**
     * @brief 获取支持的类型列表
     */
    std::vector<PathSegmentType> getSupportedTypes() const {
        std::vector<PathSegmentType> types;
        for (const auto& pair : creators_) {
            types.push_back(pair.first);
        }
        return types;
    }

private:
    PathSegmentFactory() {
        // 注册内置的路径段类型
        registerBuiltinTypes();
    }

    void registerBuiltinTypes() {
        // 直线段
        registerCreator(PathSegmentType::kStraight,
            [](const Pose2D& start, const Pose2D& end,
               const MotionConstraints& constraints,
               const std::vector<Pose2D>& /*params*/) {
                return std::make_shared<StraightSegment>(start, end, constraints);
            });

        // 三次贝塞尔曲线
        registerCreator(PathSegmentType::kCubicBezier,
            [](const Pose2D& start, const Pose2D& end,
               const MotionConstraints& constraints,
               const std::vector<Pose2D>& params) {
                if (params.size() >= 2) {
                    return std::static_pointer_cast<PathSegment>(
                        std::make_shared<BezierSegment>(
                            start, end, params[0], params[1], constraints));
                }
                // 没有控制点，回退到直线
                return std::static_pointer_cast<PathSegment>(
                    std::make_shared<StraightSegment>(start, end, constraints));
            });

        // 简单圆弧（支持三点确定或曲率确定）
        registerCreator(PathSegmentType::kCircularArc,
            [](const Pose2D& start, const Pose2D& end,
               const MotionConstraints& constraints,
               const std::vector<Pose2D>& params) {
                std::shared_ptr<ArcSegment> segment;
                
                if (!params.empty() && params[0].yaw >= 0) {
                    // yaw >= 0 表示这是控制点（中间点），使用三点确定圆弧
                    auto arc = ArcSegment::createFromThreePoints(start, params[0], end);
                    if (arc) {
                        segment = std::shared_ptr<ArcSegment>(arc.release());
                    }
                }
                
                if (!segment) {
                    // 使用曲率方式创建圆弧
                    double curvature = 0.2;  // 默认曲率
                    if (!params.empty() && params[0].yaw < 0) {
                        // yaw < 0 表示这是曲率参数
                        curvature = params[0].x;
                    }
                    segment = std::make_shared<ArcSegment>(start, end, curvature);
                }
                
                segment->setConstraints(constraints);
                return std::static_pointer_cast<PathSegment>(segment);
            });

        // Dubins曲线
        registerCreator(PathSegmentType::kDubins,
            [](const Pose2D& start, const Pose2D& end,
               const MotionConstraints& constraints,
               const std::vector<Pose2D>& params) {
                double curvature = params.empty() ? 0.2 : params[0].x;
                auto arc = ArcSegment::createDubins(start, end, curvature);
                if (arc) {
                    arc->setConstraints(constraints);
                    return std::shared_ptr<PathSegment>(arc.release());
                }
                // 失败回退到直线
                return std::static_pointer_cast<PathSegment>(
                    std::make_shared<StraightSegment>(start, end, constraints));
            });

        // Reeds-Shepp曲线
        registerCreator(PathSegmentType::kReedsShepp,
            [](const Pose2D& start, const Pose2D& end,
               const MotionConstraints& constraints,
               const std::vector<Pose2D>& params) {
                double curvature = params.empty() ? 0.2 : params[0].x;
                auto arc = ArcSegment::createReedsShepp(start, end, curvature);
                if (arc) {
                    arc->setConstraints(constraints);
                    return std::shared_ptr<PathSegment>(arc.release());
                }
                return std::static_pointer_cast<PathSegment>(
                    std::make_shared<StraightSegment>(start, end, constraints));
            });
    }

    std::map<PathSegmentType, PathSegmentCreator> creators_;//? 为什么不用unordered_map？
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_PATH_PATH_SEGMENT_FACTORY_HPP_
