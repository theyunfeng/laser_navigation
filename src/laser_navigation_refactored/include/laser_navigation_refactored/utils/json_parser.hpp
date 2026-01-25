/**
 * @file json_parser.hpp
 * @brief JSON解析工具类
 * 
 * 用于解析调度系统发来的导航任务JSON，支持：
 * - 多站点路径（起点、过渡点、终点）
 * - 每段边的运动约束
 * - 多种轨迹类型：直线、贝塞尔曲线、圆弧、Dubins、Reeds-Shepp
 * - 热更新任务
 * 
 * JSON格式示例：
 * {
 *   "taskId": "task_001",
 *   "chassisType": "Differential",   // 可选："Differential"(默认) 或 "swerve"/"4wis4wid"
 *   "nodes": [
 *     {"nodeId": "A", "x": 0.0, "y": 0.0, "theta": 0.0},
 *     {"nodeId": "B", "x": 1.0, "y": 0.0, "theta": 0.0},
 *     {"nodeId": "C", "x": 2.0, "y": 1.0, "theta": 1.57}
 *   ],
 *   "edges": [
 *     {
 *       "startNodeId": "A", "endNodeId": "B",
 *       "direction": 1, "maxLineSpeed": 0.5, "maxLineAcc": 0.3,
 *       "trajectory": {"type": "Straight"}
 *     },
 *     {
 *       "startNodeId": "B", "endNodeId": "C",
 *       "direction": 1, "maxLineSpeed": 0.4,
 *       "trajectory": {
 *         "type": "CubicBezier",
 *         "controlPoints": [{"x": 1.3, "y": 0.3}, {"x": 1.7, "y": 0.7}]
 *       }
 *     }
 *   ],
 *   "startAngleAdjust": true,
 *   "endAngleAdjust": true,
 *   "autoGenerateControlPoints": true,
 *   "controlPointExtensionFactor": 0.35,
 *   "smoothTransition": true
 * }
 * 
 * 轨迹类型(trajectory.type)说明：
 * 
 * 1. Straight/straight/Line - 直线轨迹（默认）
 *    无需额外参数
 * 
 * 2. CubicBezier/Bezier - 三次贝塞尔曲线
 *    - controlPoints: [{"x": p1x, "y": p1y}, {"x": p2x, "y": p2y}]  // 两个控制点
 *    - 如果不指定且 autoGenerateControlPoints=true，将自动生成
 * 
 * 3. Arc/arc/Circular - 圆弧轨迹
 *    两种指定方式（二选一）：
 *    - controlPoint: {"x": mx, "y": my}       // 中间点（三点确定圆弧）
 *    - controlPoints: [{"x": mx, "y": my}]    // 与laser_navigation_package兼容
 *    - curvature: 0.2                         // 曲率方式（不推荐）
 * 
 * 4. Dubins/dubins - Dubins路径（仅向前）
 *    两种指定方式（二选一）：
 *    - controlPoint: {"x": mx, "y": my}       // 中间点（三点确定圆弧）
 *    - controlPoints: [{"x": mx, "y": my}]    // 与laser_navigation_package兼容
 *    - curvature: 0.2                         // 曲率方式
 *    - 如果不指定，使用默认曲率0.2
 * 
 * 5. ReedsShepp/reeds_shepp/RS - Reeds-Shepp路径（支持倒车）
 *    两种指定方式（二选一）：
 *    - controlPoint: {"x": mx, "y": my}       // 中间点（三点确定圆弧）
 *    - controlPoints: [{"x": mx, "y": my}]    // 与laser_navigation_package兼容
 *    - curvature: 0.2                         // 曲率方式
 *    - 如果不指定，使用默认曲率0.2
 * 
 * 注意：Arc/Dubins/ReedsShepp的controlPoint/controlPoints格式与laser_navigation_package保持一致
 */

#ifndef LASER_NAVIGATION_REFACTORED_UTILS_JSON_PARSER_HPP_
#define LASER_NAVIGATION_REFACTORED_UTILS_JSON_PARSER_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include "laser_navigation_refactored/core/optional.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <functional>

namespace laser_navigation {

/**
 * @brief 解析后的导航任务数据
 */
struct NavigationTask {
    std::string task_id;                        ///< 任务ID
    std::vector<Pose2D> waypoints;              ///< 路径点（起点+过渡点+终点）
    std::vector<MotionConstraints> constraints; ///< 每段边的约束
    std::vector<PathSegmentType> segment_types; ///< 每段的类型
    std::vector<std::vector<Pose2D>> control_points; ///< 贝塞尔控制点
    bool start_angle_adjust{true};              ///< 是否调整起点角度
    bool end_angle_adjust{true};                ///< 是否调整终点角度
    ChassisType chassis_type{ChassisType::kDifferential}; ///< 底盘类型
    
    /// 贝塞尔控制点自动生成选项
    bool auto_generate_control_points{true};    ///< 是否自动生成缺失的控制点
    double control_point_extension_factor{0.35}; ///< 控制点延伸系数
    
    /// 是否有效
    bool isValid() const { return waypoints.size() >= 2; }
    
    /// 获取站点数量
    size_t getWaypointCount() const { return waypoints.size(); }
    
    /// 获取边（段）数量
    size_t getSegmentCount() const { 
        return waypoints.empty() ? 0 : waypoints.size() - 1; 
    }
    
    /// 是否有过渡点（中间点）
    bool hasTransitionPoints() const { return waypoints.size() > 2; }
    
    /// 获取过渡点数量
    size_t getTransitionPointCount() const {
        return waypoints.size() > 2 ? waypoints.size() - 2 : 0;
    }
};

/**
 * @brief JSON解析器
 * 
 * 解析调度系统发来的导航任务JSON
 */
class JsonParser {
public:
    /// 日志回调类型
    using LogCallback = std::function<void(const std::string&)>;
    
    JsonParser() = default;
    ~JsonParser() = default;
    
    /**
     * @brief 设置日志回调
     */
    void setLogCallback(LogCallback callback) { log_callback_ = callback; }
    
    /**
     * @brief 从JSON字符串解析导航任务
     * @param json_str JSON字符串
     * @return 解析结果，失败返回 nullopt
     */
    Optional<NavigationTask> parseTask(const std::string& json_str);
    
    /**
     * @brief 从JSON对象解析导航任务
     * @param json_obj JSON对象
     * @return 解析结果，失败返回 nullopt
     */
    Optional<NavigationTask> parseTask(const nlohmann::json& json_obj);
    
    /**
     * @brief 解析热更新任务（追加路径点）
     * @param json_str JSON字符串
     * @return 解析结果，失败返回 nullopt
     * 
     * 热更新JSON格式与普通任务相同，但只取新增的站点
     */
    Optional<NavigationTask> parseUpdateTask(const std::string& json_str);
    
    /**
     * @brief 设置默认运动约束
     * @param constraints 默认约束
     * 
     * 当JSON中没有指定某些参数时使用默认值
     */
    void setDefaultConstraints(const MotionConstraints& constraints) {
        default_constraints_ = constraints;
    }
    
    /**
     * @brief 获取最后一次解析错误信息
     */
    std::string getLastError() const { return last_error_; }

private:
    /**
     * @brief 解析nodes数组
     */
    bool parseNodes(const nlohmann::json& json_obj, 
                    std::vector<Pose2D>& waypoints);
    
    /**
     * @brief 解析edges数组
     */
    bool parseEdges(const nlohmann::json& json_obj,
                    const std::vector<Pose2D>& waypoints,
                    std::vector<MotionConstraints>& constraints,
                    std::vector<PathSegmentType>& segment_types,
                    std::vector<std::vector<Pose2D>>& control_points);
    
    /**
     * @brief 解析单条边的trajectory
     */
    bool parseTrajectory(const nlohmann::json& traj_obj,
                         PathSegmentType& type,
                         std::vector<Pose2D>& ctrl_points);
    
    /**
     * @brief 输出日志
     */
    void log(const std::string& message);
    
    /**
     * @brief 设置错误信息
     */
    void setError(const std::string& error);
    
    LogCallback log_callback_;
    MotionConstraints default_constraints_;
    std::string last_error_;
};

//==============================================================================
// 内联实现
//==============================================================================

inline void JsonParser::log(const std::string& message) {
    if (log_callback_) {
        log_callback_(message);
    }
}

inline void JsonParser::setError(const std::string& error) {
    last_error_ = error;
    log("Error: " + error);
}

inline Optional<NavigationTask> JsonParser::parseTask(const std::string& json_str) {
    try {
        nlohmann::json json_obj = nlohmann::json::parse(json_str);
        return parseTask(json_obj);
    } catch (const nlohmann::json::parse_error& e) {
        setError(std::string("JSON parse error: ") + e.what());
        return nullopt;
    }
}

inline Optional<NavigationTask> JsonParser::parseTask(const nlohmann::json& json_obj) {
    NavigationTask task;
    
    try {
        // 解析taskId（可选）
        if (json_obj.contains("taskId")) {
            task.task_id = json_obj["taskId"].get<std::string>();
        }
        
        // 解析nodes（必需）
        if (!parseNodes(json_obj, task.waypoints)) {
            return nullopt;
        }
        
        // 至少需要2个点
        if (task.waypoints.size() < 2) {
            setError("At least 2 nodes required");
            return nullopt;
        }
        
        // 解析edges（可选，没有则使用默认值）
        if (!parseEdges(json_obj, task.waypoints, task.constraints, 
                        task.segment_types, task.control_points)) {
            return nullopt;
        }
        
        // 解析角度调整选项
        if (json_obj.contains("startAngleAdjust")) {
            task.start_angle_adjust = json_obj["startAngleAdjust"].get<bool>();
        }
        if (json_obj.contains("endAngleAdjust")) {
            task.end_angle_adjust = json_obj["endAngleAdjust"].get<bool>();
        }
        
        // 解析控制点自动生成选项
        if (json_obj.contains("autoGenerateControlPoints")) {
            task.auto_generate_control_points = json_obj["autoGenerateControlPoints"].get<bool>();
        }
        if (json_obj.contains("controlPointExtensionFactor")) {
            task.control_point_extension_factor = json_obj["controlPointExtensionFactor"].get<double>();
        }
        // 也支持 smoothTransition 作为别名
        if (json_obj.contains("smoothTransition")) {
            task.auto_generate_control_points = json_obj["smoothTransition"].get<bool>();
        }
        
        // 解析底盘类型（默认为差速模型）
        task.chassis_type = ChassisType::kDifferential;  // 默认差速
        if (json_obj.contains("chassisType")) {
            std::string type_str = json_obj["chassisType"].get<std::string>();
            if (type_str == "swerve" || type_str == "4wis4wid" || type_str == "Swerve") {
                task.chassis_type = ChassisType::kSwerve4WIS4WID;
                log("Chassis type: Swerve (4WIS4WID)");
            } else {
                log("Chassis type: Differential (default)");
            }
        } else {
            log("Chassis type not specified, using Differential (default)");
        }
        
        log("Parsed task with " + std::to_string(task.waypoints.size()) + 
            " waypoints (" + std::to_string(task.getTransitionPointCount()) + 
            " transition points)");
        
        return task;
        
    } catch (const std::exception& e) {
        setError(std::string("Parse exception: ") + e.what());
        return nullopt;
    }
}

inline Optional<NavigationTask> JsonParser::parseUpdateTask(const std::string& json_str) {
    // 热更新与普通解析相同，调用者负责将结果传给 NavigationExecutor::update()
    return parseTask(json_str);
}

inline bool JsonParser::parseNodes(const nlohmann::json& json_obj,
                                    std::vector<Pose2D>& waypoints) {
    if (!json_obj.contains("nodes")) {
        setError("Missing 'nodes' field");
        return false;
    }
    
    const auto& nodes = json_obj["nodes"];
    if (!nodes.is_array() || nodes.empty()) {
        setError("'nodes' must be a non-empty array");
        return false;
    }
    
    waypoints.clear();
    waypoints.reserve(nodes.size());
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& node = nodes[i];
        
        // x和y是必需的
        if (!node.contains("x") || !node.contains("y")) {
            setError("Node " + std::to_string(i) + " missing x or y");
            return false;
        }
        
        Pose2D pose;
        pose.x = node["x"].get<double>();
        pose.y = node["y"].get<double>();
        
        // theta/yaw是可选的
        if (node.contains("theta")) {
            pose.yaw = node["theta"].get<double>();
        } else if (node.contains("yaw")) {
            pose.yaw = node["yaw"].get<double>();
        } else {
            pose.yaw = 0.0;
        }
        
        waypoints.push_back(pose);
    }
    
    return true;
}

inline bool JsonParser::parseEdges(const nlohmann::json& json_obj,
                                    const std::vector<Pose2D>& waypoints,
                                    std::vector<MotionConstraints>& constraints,
                                    std::vector<PathSegmentType>& segment_types,
                                    std::vector<std::vector<Pose2D>>& control_points) {
    size_t num_edges = waypoints.size() - 1;
    
    // 初始化默认值
    constraints.resize(num_edges, default_constraints_);
    segment_types.resize(num_edges, PathSegmentType::kStraight);
    control_points.resize(num_edges);
    
    // 如果没有edges字段，使用默认值
    if (!json_obj.contains("edges")) {
        log("No 'edges' field, using default constraints");
        return true;
    }
    
    const auto& edges = json_obj["edges"];
    if (!edges.is_array()) {
        setError("'edges' must be an array");
        return false;
    }
    
    // 解析每条边
    // 注意：edges可能不是按顺序的，需要通过startNodeId/endNodeId匹配
    // 简化处理：假设edges按顺序排列
    for (size_t i = 0; i < edges.size() && i < num_edges; ++i) {
        const auto& edge = edges[i];
        MotionConstraints& cons = constraints[i];
        
        // 解析方向
        if (edge.contains("direction")) {
            int dir = edge["direction"].get<int>();
            cons.is_forward = (dir == 1);
        }
        
        // 解析速度约束
        if (edge.contains("maxLineSpeed")) {
            cons.max_velocity = edge["maxLineSpeed"].get<double>();
        }
        if (edge.contains("maxLineAcc")) {
            cons.max_acceleration = edge["maxLineAcc"].get<double>();
        }
        if (edge.contains("maxLineDec")) {
            cons.max_deceleration = edge["maxLineDec"].get<double>();
        }
        if (edge.contains("maxLineJerk")) {
            cons.max_jerk = edge["maxLineJerk"].get<double>();
        }
        
        // 解析旋转约束
        if (edge.contains("maxRotSpeed")) {
            cons.max_angular_velocity = edge["maxRotSpeed"].get<double>();
        }
        if (edge.contains("maxRotAcc")) {
            cons.max_angular_acceleration = edge["maxRotAcc"].get<double>();
        }
        if (edge.contains("maxRotDec")) {
            cons.max_angular_deceleration = edge["maxRotDec"].get<double>();
        }
        
        // 解析到达精度
        if (edge.contains("reachDist")) {
            cons.reach_distance = edge["reachDist"].get<double>();
        }
        if (edge.contains("reachAngle")) {
            cons.reach_angle = edge["reachAngle"].get<double>();
        }
        
        // 解析轨迹类型
        if (edge.contains("trajectory")) {
            if (!parseTrajectory(edge["trajectory"], segment_types[i], control_points[i])) {
                return false;
            }
        }
    }
    
    return true;
}

inline bool JsonParser::parseTrajectory(const nlohmann::json& traj_obj,
                                         PathSegmentType& type,
                                         std::vector<Pose2D>& ctrl_points) {
    type = PathSegmentType::kStraight;
    ctrl_points.clear();
    
    if (!traj_obj.contains("type")) {
        return true;  // 默认直线
    }
    
    std::string type_str = traj_obj["type"].get<std::string>();
    
    if (type_str == "CubicBezier" || type_str == "cubicBezier" || 
        type_str == "Bezier" || type_str == "bezier") {
        type = PathSegmentType::kCubicBezier;
        
        // 解析控制点（可选，如果没有指定则后续自动生成）
        if (traj_obj.contains("controlPoints")) {
            const auto& points = traj_obj["controlPoints"];
            if (!points.is_array()) {
                setError("'controlPoints' must be an array");
                return false;
            }
            
            for (const auto& pt : points) {
                Pose2D ctrl;
                ctrl.x = pt["x"].get<double>();
                ctrl.y = pt["y"].get<double>();
                ctrl.yaw = 0.0;
                ctrl_points.push_back(ctrl);
            }
            
            // 三次贝塞尔需要2个控制点
            if (ctrl_points.size() != 2) {
                log("Warning: CubicBezier expects 2 control points, got " + 
                    std::to_string(ctrl_points.size()) + 
                    ". Will auto-generate if enabled.");
                ctrl_points.clear();  // 清空，让后续自动生成
            }
        } else {
            // 没有指定控制点，将在后续自动生成
            log("CubicBezier trajectory without controlPoints, will auto-generate");
        }
    } else if (type_str == "Straight" || type_str == "straight" || type_str == "Line") {
        type = PathSegmentType::kStraight;
    } else if (type_str == "Arc" || type_str == "arc" || type_str == "Circular") {
        type = PathSegmentType::kCircularArc;
        
        // 解析圆弧控制点（三点确定圆弧：起点、控制点、终点）
        if (traj_obj.contains("controlPoint")) {
            // 单个控制点（中间点）
            const auto& pt = traj_obj["controlPoint"];
            Pose2D ctrl;
            ctrl.x = pt["x"].get<double>();
            ctrl.y = pt["y"].get<double>();
            ctrl.yaw = 0.0;  // yaw = 0 表示这是中间点
            ctrl_points.push_back(ctrl);
            log("Arc trajectory with control point specified");
        } else if (traj_obj.contains("controlPoints")) {
            // 也支持controlPoints数组（只取第一个作为中间点）
            const auto& points = traj_obj["controlPoints"];
            if (points.is_array() && !points.empty()) {
                Pose2D ctrl;
                ctrl.x = points[0]["x"].get<double>();
                ctrl.y = points[0]["y"].get<double>();
                ctrl.yaw = 0.0;
                ctrl_points.push_back(ctrl);
                log("Arc trajectory with control point from array");
            }
        } else if (traj_obj.contains("curvature")) {
            // 使用曲率方式（旧方式，曲率存在x字段，yaw=-1表示曲率模式）
            Pose2D param;
            param.x = traj_obj.value("curvature", 0.2);
            param.y = 0;
            param.yaw = -1.0;  // yaw = -1 表示这是曲率参数
            ctrl_points.push_back(param);
            log("Arc trajectory with curvature specified");
        } else {
            // 没有指定控制点和曲率，将使用默认的曲率
            log("Arc trajectory without control point, will use default curvature");
        }
    } else if (type_str == "Dubins" || type_str == "dubins") {
        type = PathSegmentType::kDubins;
        
        // 优先解析控制点（三点确定圆弧方式，与Arc一致）
        if (traj_obj.contains("controlPoint")) {
            // 单个控制点（中间点）
            const auto& pt = traj_obj["controlPoint"];
            Pose2D ctrl;
            ctrl.x = pt["x"].get<double>();
            ctrl.y = pt["y"].get<double>();
            ctrl.yaw = 0.0;  // yaw = 0 表示这是中间点
            ctrl_points.push_back(ctrl);
            log("Dubins trajectory with control point specified");
        } else if (traj_obj.contains("controlPoints")) {
            // 也支持controlPoints数组（只取第一个作为中间点）
            const auto& points = traj_obj["controlPoints"];
            if (points.is_array() && !points.empty()) {
                Pose2D ctrl;
                ctrl.x = points[0]["x"].get<double>();
                ctrl.y = points[0]["y"].get<double>();
                ctrl.yaw = 0.0;  // yaw = 0 表示这是中间点
                ctrl_points.push_back(ctrl);
                log("Dubins trajectory with control point from array");
            }
        } else if (traj_obj.contains("curvature")) {
            // 使用曲率方式（曲率存在x字段，yaw=-1表示曲率模式）
            Pose2D param;
            param.x = traj_obj.value("curvature", 0.2);
            param.y = 0;
            param.yaw = -1.0;  // yaw = -1 表示这是曲率参数
            ctrl_points.push_back(param);
            log("Dubins trajectory with curvature specified");
        } else {
            // 没有指定控制点和曲率，将使用默认曲率
            log("Dubins trajectory without control point, will use default curvature");
        }
    } else if (type_str == "ReedsShepp" || type_str == "reeds_shepp" || 
               type_str == "RS" || type_str == "rs") {
        type = PathSegmentType::kReedsShepp;
        
        // 优先解析控制点（三点确定圆弧方式，与Arc一致）
        if (traj_obj.contains("controlPoint")) {
            // 单个控制点（中间点）
            const auto& pt = traj_obj["controlPoint"];
            Pose2D ctrl;
            ctrl.x = pt["x"].get<double>();
            ctrl.y = pt["y"].get<double>();
            ctrl.yaw = 0.0;  // yaw = 0 表示这是中间点
            ctrl_points.push_back(ctrl);
            log("ReedsShepp trajectory with control point specified");
        } else if (traj_obj.contains("controlPoints")) {
            // 也支持controlPoints数组（只取第一个作为中间点）
            const auto& points = traj_obj["controlPoints"];
            if (points.is_array() && !points.empty()) {
                Pose2D ctrl;
                ctrl.x = points[0]["x"].get<double>();
                ctrl.y = points[0]["y"].get<double>();
                ctrl.yaw = 0.0;  // yaw = 0 表示这是中间点
                ctrl_points.push_back(ctrl);
                log("ReedsShepp trajectory with control point from array");
            }
        } else if (traj_obj.contains("curvature")) {
            // 使用曲率方式（曲率存在x字段，yaw=-1表示曲率模式）
            Pose2D param;
            param.x = traj_obj.value("curvature", 0.2);
            param.y = 0;
            param.yaw = -1.0;  // yaw = -1 表示这是曲率参数
            ctrl_points.push_back(param);
            log("ReedsShepp trajectory with curvature specified");
        } else {
            // 没有指定控制点和曲率，将使用默认曲率
            log("ReedsShepp trajectory without control point, will use default curvature");
        }
    }
    
    return true;
}

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_UTILS_JSON_PARSER_HPP_
