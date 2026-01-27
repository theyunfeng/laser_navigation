/**
 * @file navigation_example_node.cpp
 * @brief 导航功能包示例节点
 * 
 * 演示如何使用重构后的导航模块
 * 支持差速底盘和四转四驱底盘
 * 
 * 位姿输入（必须）:
 *   - 来自激光SLAM的实时2D位姿 (x, y, theta)
 *   - 用于LQR轨迹跟踪和路径规划
 * 
 * 速度反馈（可选）:
 *   - 来自里程计的速度信息 (vx, vy, omega)
 *   - 用于改善控制平滑性
 *   - 无里程计时依然可以进行导航
 * 20260127 将原导航的大部分功能移植到了现在的代码中，还缺障碍物回调处理、导航恢复、里程计/激光位姿下纯平移或自转实现以及
 * controlloop循环函数中的各个功能实现，后续还要改进直线和贝塞尔导航精度以及更新日志输出格式等
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

#include "laser_navigation_refactored/navigation_executor.hpp"
#include "laser_navigation_refactored/utils/json_parser.hpp"


#include "common/topic.h"

using namespace std::chrono_literals;

/**
 * @brief 导航示例节点
 * 
 * 订阅:
 * - /slam_pose: 激光SLAM位姿（必须，用于轨迹跟踪）
 * - /odom: 里程计信息（可选，用于速度反馈改善控制）
 * - /goal_pose: 目标位姿
 * - /navigation_path: 路径信息 (JSON格式)
 * 
 * 发布:
 * - /cmd_vel: 速度指令 (对于四转四驱，包含 vx, vy, ω)
 * - /navigation_status: 导航状态
 */

// 用于控制外层导航进入不同的控制状态
typedef enum
{
    STATE_MACH_IDLE = 0,            // 空闲中
    STATE_MACH_RUN,                 // 导航算法中
    STATE_MACH_RUN_DEC_STOP,        // 障碍物被阻挡，开始缓停
    STATE_MACH_RUN_EMERGE_STOP,     // 障碍物被阻挡，开始减速急停
    STATE_MACH_RUN_EMS_STOP,        // 软急停触发，开始急停
    STATE_MACH_RUN_PAUSE,           // 手动下发，开始暂停
    STATE_MACH_RUN_STOP,            // 手动下发，开始停止
    STATE_MACH_OBS_PAUSE,           // 障碍物被阻挡暂停中
    STATE_MACH_EMS_PAUSE,           // 急停导致的暂停中
    STATE_MACH_PASUE,               // 暂停了
    STATE_MACH_RUN_TRANSLATE,       // 平动中
    STATE_MACH_RUN_ROLATE           // 转动中
} MachineControlState; 

class NavigationRefactoredNode : public rclcpp::Node {
public:
    NavigationRefactoredNode() : Node("navigation_example_node") {
        // 声明参数 - 配置文件路径
        this->declare_parameter("model_config_path", "/home/robot/config/model.json");
        this->declare_parameter("nav_config_path", "/home/robot/config/navigation_config.json");
        
        // 声明参数 - 话题名称
        this->declare_parameter("slam_pose_topic", "slam_pose");   // 激光SLAM位姿（必须）
        this->declare_parameter("odom_topic", "odom");             // 里程计（可选，用于速度反馈）
        
        // 声明参数 - 可被配置文件覆盖的默认值
        this->declare_parameter("control_frequency", 25.0);
        this->declare_parameter("max_velocity", 0.5);
        this->declare_parameter("max_angular_velocity", 0.8);
        this->declare_parameter("max_acceleration", 0.2);
        this->declare_parameter("max_angular_acceleration", 0.8);
        this->declare_parameter("use_odom_feedback", true);
        this->declare_parameter("odom_timeout", 0.2);
        
        // 1. 从 model.json 读取底盘类型
        std::string model_path = this->get_parameter("model_config_path").as_string();
        loadModelConfig(model_path);
        
        // 2. 从 navigation_config.json 读取导航参数（可选）
        std::string nav_config_path = this->get_parameter("nav_config_path").as_string();
        loadNavigationConfig(nav_config_path);
        
        // 3. 获取最终参数值（配置文件优先，否则使用ROS参数/默认值）
        double control_freq = this->get_parameter("control_frequency").as_double();
        control_period_ = 1.0 / control_freq;
        use_odom_feedback_ = this->get_parameter("use_odom_feedback").as_bool();
        odom_timeout_ = this->get_parameter("odom_timeout").as_double();
        
        // 4. 初始化运动参数和LQR参数
        initializeParameters();
        
        // 设置导航器底盘类型
        navigator_.setChassisType(chassis_type_);
        navigator_.setOdometryTimeout(odom_timeout_);
        
        // 获取话题名称
        std::string slam_pose_topic = this->get_parameter("slam_pose_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        
        // 创建订阅者 
        //  SLAM位姿（必须）
        m_slam_pose_sub_ = this->create_subscription<std_msgs::msg::String>(
            STATE_POS, 10,
            std::bind(&NavigationRefactoredNode::slamPoseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to SLAM pose: %s (REQUIRED)", slam_pose_topic.c_str());
        
        // 里程计速度反馈（可选）
        if (use_odom_feedback_) {
            m_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                STATE_ODOM, 10,
                std::bind(&NavigationRefactoredNode::odomCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to odometry: %s (OPTIONAL, for velocity feedback)", odom_topic.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Odometry feedback disabled, using open-loop control");
        }

        //急停订阅
        m_EmergedStop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "emergency_stop", 10,
            std::bind(&NavigationRefactoredNode::emergedStopCallback, this, std::placeholders::_1));
        
        // 创建订阅者 - 目标和路径
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&NavigationRefactoredNode::goalCallback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_path", 10,
            std::bind(&NavigationRefactoredNode::navTaskCallback, this, std::placeholders::_1));
        
        // 创建发布者
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("STATE_NAV", 10);
        codes_pub_ = this->create_publisher<std_msgs::msg::String>("TASK_CODE", 10);
        
        // 创建控制循环定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_),
            std::bind(&NavigationRefactoredNode::controlLoop, this));
        
        // 设置日志回调
        navigator_.setLogCallback([this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        });
        
        // 初始化JSON解析器
        json_parser_.setLogCallback([this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "[JsonParser] %s", msg.c_str());
        });
        json_parser_.setDefaultConstraints(default_constraints_);
        
        RCLCPP_INFO(this->get_logger(), "Navigation example node started");
        RCLCPP_INFO(this->get_logger(), "  - Chassis type: %s", 
            chassis_type_ == laser_navigation::ChassisType::k4WIS4WID ? "4WIS4WID" : "DiffDrive");
        RCLCPP_INFO(this->get_logger(), "  - SLAM pose input: REQUIRED (for LQR tracking)");
        RCLCPP_INFO(this->get_logger(), "  - Odom velocity feedback: %s", use_odom_feedback_ ? "enabled" : "disabled");
    }

private:
    void initializeParameters() {
        double max_vel = this->get_parameter("max_velocity").as_double();
        double max_ang_vel = this->get_parameter("max_angular_velocity").as_double();
        double max_acc = this->get_parameter("max_acceleration").as_double();
        double max_ang_acc = this->get_parameter("max_angular_acceleration").as_double();
        
        // 默认运动约束
        default_constraints_.max_velocity = max_vel;
        default_constraints_.max_angular_velocity = max_ang_vel;
        default_constraints_.max_acceleration = max_acc;
        default_constraints_.max_deceleration = max_acc;
        default_constraints_.max_angular_acceleration = max_ang_acc;
        default_constraints_.max_angular_deceleration = max_ang_acc;
        default_constraints_.max_jerk = 0.5;
        default_constraints_.reach_distance = 0.02;
        default_constraints_.reach_angle = 0.02;
        default_constraints_.is_forward = true;
        default_constraints_.chassis_type = chassis_type_;
        
        // 横向速度约束（仅四转四驱）
        default_constraints_.max_lateral_velocity = 0.3;
        default_constraints_.max_lateral_acceleration = 0.2;
        default_constraints_.max_lateral_deceleration = 0.2;
        
        // LQR参数 - 优先使用配置文件中的值
        lqr_params_.q1 = getConfigValue(config_lqr_params_, "q1", 10.0);
        lqr_params_.q2 = getConfigValue(config_lqr_params_, "q2", 10.0);
        lqr_params_.q3 = getConfigValue(config_lqr_params_, "q3", 10.0);
        lqr_params_.r1 = getConfigValue(config_lqr_params_, "r1", 1.0);
        lqr_params_.r2 = getConfigValue(config_lqr_params_, "r2", 1.0);
        lqr_params_.dt = control_period_;
        lqr_params_.max_iterations = 10;
        
        // 贝塞尔LQR参数 - 优先使用配置文件中的值
        bezier_lqr_params_.q1 = getConfigValue(config_bezier_lqr_params_, "q1", 0.5);
        bezier_lqr_params_.q2 = getConfigValue(config_bezier_lqr_params_, "q2", 0.5);
        bezier_lqr_params_.q3 = getConfigValue(config_bezier_lqr_params_, "q3", 0.005);
        bezier_lqr_params_.r1 = getConfigValue(config_bezier_lqr_params_, "r1", 1.0);
        bezier_lqr_params_.r2 = getConfigValue(config_bezier_lqr_params_, "r2", 1.0);
        bezier_lqr_params_.dt = control_period_;
    }
    
    /**
     * @brief 从 model.json 加载底盘类型配置
     * 
     * model.json 格式:
     * {
     *     "agvType": "4WIS4WID"  // 或 "DiffDrive"
     * }
     */
    void loadModelConfig(const std::string& path) {
        try {
            std::ifstream file(path);
            if (!file.is_open()) {
                RCLCPP_WARN(this->get_logger(), 
                    "Cannot open model config: %s, using default (DiffDrive)", path.c_str());
                chassis_type_ = laser_navigation::ChassisType::kDifferential;
                return;
            }
            
            nlohmann::json json = nlohmann::json::parse(file);
            
            if (json.contains("motorType")) {
                std::string agv_type = json["motorType"].get<std::string>();
                
                if (agv_type == "4WIS4WID") {
                    chassis_type_ = laser_navigation::ChassisType::k4WIS4WID;
                    RCLCPP_INFO(this->get_logger(), 
                        "[model.json] Chassis type: 4WIS4WID (Swerve)");
                } else if (agv_type == "DiffDrive") {
                    chassis_type_ = laser_navigation::ChassisType::kDifferential;
                    RCLCPP_INFO(this->get_logger(), 
                        "[model.json] Chassis type: DiffDrive (Differential)");
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                        "[model.json] Unknown agvType: %s, using DiffDrive", agv_type.c_str());
                    chassis_type_ = laser_navigation::ChassisType::kDifferential;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), 
                    "[model.json] No 'agvType' field, using DiffDrive");
                chassis_type_ = laser_navigation::ChassisType::kDifferential;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Error parsing model.json: %s", e.what());
            chassis_type_ = laser_navigation::ChassisType::kDifferential;
        }
    }
    
    /**
     * @brief 从 navigation_config.json 加载导航参数
     * 
     * navigation_config.json 格式:
     * {
     *     "control_frequency": 25.0,
     *     "use_odom_feedback": true,
     *     "odom_timeout": 0.2,
     *     "motion_constraints": {
     *         "max_velocity": 0.5,
     *         "max_angular_velocity": 0.8,
     *         "max_acceleration": 0.2,
     *         "max_angular_acceleration": 0.8,
     *         "max_lateral_velocity": 0.3,
     *         "reach_distance": 0.02,
     *         "reach_angle": 0.02
     *     },
     *     "lqr_params": {
     *         "q1": 10.0, "q2": 10.0, "q3": 10.0,
     *         "r1": 1.0, "r2": 1.0
     *     },
     *     "bezier_lqr_params": {
     *         "q1": 0.5, "q2": 0.5, "q3": 0.005,
     *         "r1": 1.0, "r2": 1.0
     *     }
     * }
     */
    void loadNavigationConfig(const std::string& path) {
        try {
            std::ifstream file(path);
            if (!file.is_open()) {
                RCLCPP_INFO(this->get_logger(), 
                    "Navigation config not found: %s, using ROS parameters/defaults", path.c_str());
                return;
            }
            
            nlohmann::json json = nlohmann::json::parse(file);
            RCLCPP_INFO(this->get_logger(), "[navigation_config.json] Loading parameters...");
            
            // 控制频率
            if (json.contains("control_frequency")) {
                this->set_parameter(rclcpp::Parameter("control_frequency", 
                    json["control_frequency"].get<double>()));
            }
            
            // 里程计反馈设置
            if (json.contains("use_odom_feedback")) {
                this->set_parameter(rclcpp::Parameter("use_odom_feedback", 
                    json["use_odom_feedback"].get<bool>()));
            }
            if (json.contains("odom_timeout")) {
                this->set_parameter(rclcpp::Parameter("odom_timeout", 
                    json["odom_timeout"].get<double>()));
            }
            
            // 运动约束 //TODO 参考params.json补充约束
            if (json.contains("motion_constraints")) {
                auto& mc = json["motion_constraints"];
                if (mc.contains("max_velocity"))
                    this->set_parameter(rclcpp::Parameter("max_velocity", 
                        mc["max_velocity"].get<double>()));
                if (mc.contains("max_angular_velocity"))
                    this->set_parameter(rclcpp::Parameter("max_angular_velocity", 
                        mc["max_angular_velocity"].get<double>()));
                if (mc.contains("max_acceleration"))
                    this->set_parameter(rclcpp::Parameter("max_acceleration", 
                        mc["max_acceleration"].get<double>()));
                if (mc.contains("max_angular_acceleration"))
                    this->set_parameter(rclcpp::Parameter("max_angular_acceleration", 
                        mc["max_angular_acceleration"].get<double>()));
            }
            
            // LQR参数 - 存储到成员变量，在initializeParameters中使用
            if (json.contains("lqr_params")) {
                auto& lqr = json["lqr_params"];
                if (lqr.contains("q1")) config_lqr_params_["q1"] = lqr["q1"].get<double>();
                if (lqr.contains("q2")) config_lqr_params_["q2"] = lqr["q2"].get<double>();
                if (lqr.contains("q3")) config_lqr_params_["q3"] = lqr["q3"].get<double>();
                if (lqr.contains("r1")) config_lqr_params_["r1"] = lqr["r1"].get<double>();
                if (lqr.contains("r2")) config_lqr_params_["r2"] = lqr["r2"].get<double>();
            }
            
            // 贝塞尔LQR参数
            if (json.contains("bezier_lqr_params")) {
                auto& blqr = json["bezier_lqr_params"];
                if (blqr.contains("q1")) config_bezier_lqr_params_["q1"] = blqr["q1"].get<double>();
                if (blqr.contains("q2")) config_bezier_lqr_params_["q2"] = blqr["q2"].get<double>();
                if (blqr.contains("q3")) config_bezier_lqr_params_["q3"] = blqr["q3"].get<double>();
                if (blqr.contains("r1")) config_bezier_lqr_params_["r1"] = blqr["r1"].get<double>();
                if (blqr.contains("r2")) config_bezier_lqr_params_["r2"] = blqr["r2"].get<double>();
            }
            
            RCLCPP_INFO(this->get_logger(), "[navigation_config.json] Parameters loaded successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Error parsing navigation_config.json: %s", e.what());
        }
    }
    
    /**
     * @brief SLAM位姿回调（必须）
     * 
     * 这是导航的核心输入，来自激光SLAM的实时2D位姿
     * 用于LQR轨迹跟踪、路径规划和偏离检测
     */
    void slamPoseCallback(const std_msgs::msg::String::SharedPtr msg) {
        
        std::string mode;
        int32_t sec;
        uint32_t nanosec;
        double x, y, yaw;
        bool valid;
        try
        {
            nlohmann::json json_obj = nlohmann::json::parse(msg->data);
            sec = json_obj["sec"].get<int32_t>();
            nanosec = json_obj["nanosec"].get<uint32_t>();
            current_pose_.x = json_obj["x"].get<double>();
            current_pose_.y = json_obj["y"].get<double>();
            current_pose_.yaw = json_obj["yaw"].get<double>();
            current_pose_.mode = json_obj["mode"].get<std::string>();
            current_pose_.valid = json_obj["valid"].get<bool>();
        }
        catch (...)
        {
            // 走到这里的都是字段类型不对，比如：float当成string下发
            //LOG4CPLUS_ERROR_FMT(log4cplus::Logger::getRoot(), "invalid json");
            return;
        }
        
        // 更新时间戳
        last_slam_pose_time_ = sec + nanosec * 1e-9;
        has_slam_pose_ = true;
    }
    
    /**
     * @brief 里程计回调（可选）
     * 
     * 仅提取速度信息用于改善控制平滑性
     * 即使没有里程计，导航仍然可以正常运行（开环控制）
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 仅提取速度信息用于反馈（位姿使用SLAM的）
        laser_navigation::Velocity odom_vel;
        odom_vel.linear_x = msg->twist.twist.linear.x;
        odom_vel.linear_y = msg->twist.twist.linear.y;  // 四转四驱使用
        odom_vel.angular = msg->twist.twist.angular.z;
        
        laser_navigation::Pose2D odom_pose;
        odom_pose.x = msg->pose.pose.position.x;
        odom_pose.y = msg->pose.pose.position.y;
        odom_pose.yaw = laser_navigation::math::MathUtils::quaternionToYaw(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // 更新里程计速度反馈（位姿使用SLAM的current_pose_）
        navigator_.updateOdometryFeedback(current_pose_, odom_vel, timestamp);
        
        has_odom_velocity_ = true;
    }
    /**
     * @brief 急停回调
     */
    void emergedStopCallback(const std_msgs::msg::String::SharedPtr msg) {
        bool isStop = false;
        try {
            nlohmann::json obj = nlohmann::json::parse(msg->data);
            isStop = obj["taskResult"]["emergency"];
        } catch (...) {
            return;
        }

        //TODO 根据状态状态更新
        if (m_machine_state_ == MachineControlState::STATE_MACH_IDLE) {
            return ;
        }
        if (isStop) {
        m_machine_state_ = MachineControlState::STATE_MACH_RUN_EMS_STOP;
       // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation pause by emerge");

    } else if (!isStop)  {
        if (m_machine_state_ != MachineControlState::STATE_MACH_EMS_PAUSE) {
            return;
        }

        // 延时半秒
        usleep(500 * 1000);
        if (curTask_ == "navigation")
        {
           //TODO 恢复导航的话要重新规划 ，尝试在内层的executor中恢复而不是在这里

            m_machine_state_ = MachineControlState::STATE_MACH_RUN;
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation reuse from emerge");
        }
        else if (curTask_ == "translate")
        {
            m_machine_state_ = MachineControlState::STATE_MACH_RUN_TRANSLATE;
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "translate reuse from emerge");
        }
        else if (curTask_ == "rotate")
        {
            m_machine_state_ = MachineControlState::STATE_MACH_RUN_ROLATE;
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "rotate reuse from emerge");
        }
    }

    }


    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!has_slam_pose_) {
            RCLCPP_WARN(this->get_logger(), "No SLAM pose received yet, ignoring goal");
            return;
        }
        
        // 从目标位姿提取信息
        laser_navigation::Pose2D goal;
        goal.x = msg->pose.position.x;
        goal.y = msg->pose.position.y;
        
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        goal.yaw = std::atan2(2 * (qw * qz + qx * qy), 
                              1 - 2 * (qy * qy + qz * qz));
        
        // 创建简单的直线路径
        laser_navigation::NavigationConfig config;
        config.waypoints = {current_pose_, goal};
        config.constraints = {default_constraints_};
        config.segment_types = {laser_navigation::PathSegmentType::kStraight};
        config.adjust_start_angle = true;
        config.adjust_end_angle = true;
        config.chassis_type = chassis_type_;
        
        // 初始化导航
        int result = navigator_.initialize(config, lqr_params_, bezier_lqr_params_);
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Navigation started: (%.2f, %.2f) -> (%.2f, %.2f)",
                current_pose_.x, current_pose_.y, goal.x, goal.y);
            is_navigating_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize navigation");
        }
    }
    
    void navTaskCallback(const std_msgs::msg::String::SharedPtr msg) {
        // 解析JSON格式的路径
        RCLCPP_INFO(this->get_logger(), "Received path JSON: %s", msg->data.c_str());
        
        auto task = json_parser_.parseTask(msg->data);

        if (!task) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse path JSON: %s", 
                json_parser_.getLastError().c_str());
            return;
        }

        if( !task.value().codes.empty() ){
            std_msgs::msg::String msg_tmp;
            msg_tmp.data = task.value().codes;
            codes_pub_->publish(msg_tmp);
        }

        // 保存当前任务ID（如果提供），用于后续状态回报
        m_task_id_ = task->task_id;
        m_taskUpdateId_ = task->task_update_id;

        
        //  TODO 根据任务类型选择不同的处理方式
        if( task->task_type == "START" ){
            // TODO 
            curTask_ = "navigation";
            handleTaskStart(task.value());

        }else if (task->task_type == "STOP")
        {
            handleTaskStop();
        }
        else if (task->task_type == "CANCEL")
        {
            handleTaskCancel();
        }
        else if (task->task_type == "PAUSE")
        {
            handleTaskPause();
        }
        else if (task->task_type == "RECOVERY")
        {
            handleTaskRecovery();
        }
        else if (task->task_type == "UPDATE")
        {
            handleTaskUpdate(task.value());
        }
        else if (task->task_type == "translate")
        {
            curTask_ = "translate";
            // TODO 考虑将平移 和 旋转 转移到excutor中
           // handleTranslate(jsonTask);
        }
        else if (task->task_type == "rotate")
        {
            curTask_ = "rotate";
           // handleRotate(jsonTask);
        }
        else
        {
            //LOG4CPLUS_WARN_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "invalid task type %s", taskType.c_str());
        }  
       
    }
    
    void controlLoop() {
        try
        {   using NavStatus = laser_navigation::NavigationStatus;
            NavStatus nav_status = m_navigation_status_;
            laser_navigation::NavigationOutput navOut;
            
            // 导航成功或取消导航完成后，在下一个周期将状态切换到空闲
            if (nav_status == NavStatus::kGoalReached || nav_status == NavStatus::kCancelled)
            {
                nav_status = NavStatus::kIdle;
            }

            if (m_machine_state_== MachineControlState::STATE_MACH_RUN)
            { // 开始导航
                navOut = runNavigation();
                nav_status = navOut.status;
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_DEC_STOP)
            { // 阻碍物被阻挡，开始缓停
                //nav_status = runDecStopByObs();
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_EMERGE_STOP)
            { // 阻碍物被阻挡，开始急停
                //nav_status = runEmrgencyStopByObs();
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_EMS_STOP)
            { // 软急停触发，开始急停
                //nav_status = runEmrgencyStopByEms();
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_PAUSE)
            { // 手动下发，开始暂停
                //nav_status = runDecStopByManaulPause();
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_STOP)
            { // 手动下发，开始停止
                //nav_status = runDecStopByManaulStop();
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_TRANSLATE)
            { // 开始平动
                if (rotateMode_ == "odom")
                {
                   // nav_status = runTranslate(odomPose_);
                }
                else if (rotateMode_ == "location")
                {
                   // nav_status = runTranslate(m_curPosition);
                }
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_RUN_ROLATE)
            { // 开始转动
                if (rotateMode_ == "odom")
                {
                    //nav_status = runRotate(odomPose_);
                }
                else if (rotateMode_ == "location")
                {
                   // nav_status = runRotate(m_curPosition);
                }
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_OBS_PAUSE)
            {
                nav_status = NavStatus::kObstaclePaused;
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_EMS_PAUSE)
            {
                nav_status = NavStatus::kEmergencyStopped;
            }
            else if (m_machine_state_ == MachineControlState::STATE_MACH_PASUE)
            {
                nav_status = NavStatus::kPaused;
            }

            // 状态变更时才通知
            if (m_navigation_status_ != nav_status)
            {
               // setNavStatus(nav_status);
                pubNavStatus();
            }
            if (nav_status == NavStatus::kGoalReached)
            {
                //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation finish");
            }
        }
        catch (...)
        {
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "throw execption");
        }
    }
    
    std::string getStatusString(laser_navigation::NavigationStatus status) {
        switch (status) {
            case laser_navigation::NavigationStatus::kIdle: return "idle";
            case laser_navigation::NavigationStatus::kRunning: return "running";
            case laser_navigation::NavigationStatus::kReversing: return "reversing";
            case laser_navigation::NavigationStatus::kObstaclePaused: return "obstacle_paused";
            case laser_navigation::NavigationStatus::kRotating: return "rotating";
            case laser_navigation::NavigationStatus::kGoalReached: return "goal_reached";
            case laser_navigation::NavigationStatus::kCancelled: return "cancelled";
            case laser_navigation::NavigationStatus::kSegmentReached: return "segment_reached";
            case laser_navigation::NavigationStatus::kError: return "error";
            default: return "unknown";
        }
    }
    
    /// @brief 从配置map中获取值，如果不存在则返回默认值
    double getConfigValue(const std::map<std::string, double>& config, 
                          const std::string& key, double default_val) {
        auto it = config.find(key);
        return (it != config.end()) ? it->second : default_val;
    }

    void handleTaskStart(  laser_navigation::NavigationTask& task )
    {
        
         // 新任务：初始化导航
        if (!has_slam_pose_ || !current_pose_.valid) {
            RCLCPP_WARN(this->get_logger(), "No valid SLAM pose received yet, ignoring path");
            setNavStatus( laser_navigation::NavigationStatus::kError );
            pubNavStatus();
            return;
        }
        
        // 如果JSON没有起点，使用当前位置作为起点
        //std::vector<laser_navigation::Pose2D> waypoints = task.waypoints;
        if (task.waypoints.size() >= 1) {
            // 检查第一个点是否接近当前位置
            double dist = std::hypot(task.waypoints[0].x - current_pose_.x,
                                     task.waypoints[0].y - current_pose_.y);
            if (dist > 0.5) {
                // 第一个点离当前位置较远，插入当前位置作为起点
                RCLCPP_INFO(this->get_logger(), 
                    "Inserting current pose as start point (distance to first waypoint: %.2f m)", dist);
                task.waypoints.insert(task.waypoints.begin(), current_pose_);
                
                // 同时需要为新增的第一段添加约束
                task.constraints.insert(task.constraints.begin(), default_constraints_);
                task.segment_types.insert(task.segment_types.begin(), 
                    laser_navigation::PathSegmentType::kStraight);
                task.control_points.insert(task.control_points.begin(), {});
            }
        }
        
        // 创建导航配置
        laser_navigation::NavigationConfig config;
        config.waypoints = task.waypoints;
        config.constraints = task.constraints;
        config.segment_types = task.segment_types;
        config.control_points = task.control_points;
        config.adjust_start_angle = task.start_angle_adjust;
        config.adjust_end_angle = task.end_angle_adjust;
        // 底盘类型使用节点启动时从 model.json 读取的值，不从导航任务JSON中获取
        config.chassis_type = chassis_type_;
        
        // 初始化导航
        int result = navigator_.initialize(config, lqr_params_, bezier_lqr_params_);
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Navigation started with %zu waypoints (%zu segments, %zu stop points)",
                task.waypoints.size(),
                navigator_.getSegmentCount(),
                navigator_.getStopPointCount());
            
            // 打印路径点信息
            for (size_t i = 0; i < task.waypoints.size(); ++i) {
                const auto& wp = task.waypoints[i];
                std::string point_type = "transition";
                if (i == 0) point_type = "start";
                else if (i == task.waypoints.size() - 1) point_type = "end";
                else if (navigator_.isStopPoint(i)) point_type = "stop";
                
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s: (%.3f, %.3f, %.2f°)",
                    i, point_type.c_str(), wp.x, wp.y, wp.yaw * 180.0 / M_PI);
            }
            
            is_navigating_ = true;

            m_machine_state_ = MachineControlState::STATE_MACH_RUN;

            pubNavStation( task.waypoints.begin()->node_id, (task.waypoints.begin()+1)->node_id );

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize navigation");
        }
    }

    void handleTaskStop(){
        // TODO: 小车非运动过程中不处理
        if (m_navigation_status_ == laser_navigation::NavigationStatus::kIdle )
        {
            pubNavStatus();
            return;
        }
        else if (m_navigation_status_ == laser_navigation::NavigationStatus::kError || m_navigation_status_ == laser_navigation::NavigationStatus::kPaused ||
                m_navigation_status_ == laser_navigation::NavigationStatus::kObstaclePaused || m_navigation_status_ == laser_navigation::NavigationStatus::kEmergencyStopped)
        {
            m_machine_state_= MachineControlState::STATE_MACH_IDLE;
            m_navigation_status_ = laser_navigation::NavigationStatus::kIdle;
            pubNavStatus();
            return;
        }

        if (curTask_ == "navigation")
        { //TODO 或许可以优化
            m_machine_state_ = MachineControlState::STATE_MACH_RUN_STOP;
            RCLCPP_INFO(this->get_logger(), "navigation stop manul");
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation stop manul");
        }
        else if (curTask_ == "translate" || curTask_ == "rotate")
        {
            m_machine_state_ = MachineControlState::STATE_MACH_RUN_STOP;
            RCLCPP_INFO(this->get_logger(), "translate/rotate stop manul");
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "translate/rotate stop manul");
        }
    }

    void handleTaskCancel(){
        // vda5050需要根据返回的导航状态刷新小车状态
        if (m_navigation_status_ == laser_navigation::NavigationStatus::kIdle)
        {
            pubNavStatus();
            return;
        }
        else if (m_navigation_status_ == laser_navigation::NavigationStatus::kError || m_navigation_status_ == laser_navigation::NavigationStatus::kPaused ||
                m_navigation_status_ == laser_navigation::NavigationStatus::kObstaclePaused || m_navigation_status_ == laser_navigation::NavigationStatus::kEmergencyStopped)
        {
            m_machine_state_= MachineControlState::STATE_MACH_IDLE;
            m_navigation_status_ = laser_navigation::NavigationStatus::kIdle;
            pubNavStatus();
            return;
        }

        if (curTask_ == "navigation")
        {
            m_cancel_requested_ = true;
            RCLCPP_INFO(this->get_logger(), "navigation cancel manul");
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation cancel manul");
        }
        else if (curTask_ == "translate" || curTask_ == "rotate")
        {
            RCLCPP_INFO(this->get_logger(), "translate/rotate cancel manul");
            //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "translate/rotate cancel manul");
        }
    }

     void handleTaskPause(){
         // TODO: 小车非运动过程中不处理
        if (m_machine_state_== MachineControlState::STATE_MACH_IDLE)
        {
            return;
        }
        //TODO 考虑根据小车是否导航决定在哪里使用excutor中的pause还是此接口外的pause
        m_machine_state_ = MachineControlState::STATE_MACH_RUN_PAUSE;
        
        //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation pause manul");
    }

    void handleTaskRecovery(){
        if (m_machine_state_!= MachineControlState::STATE_MACH_PASUE)//只有真停下了才考虑恢复
        {
            return;
        }
        m_machine_state_ = MachineControlState::STATE_MACH_RUN;
        // TODO 增加导航中恢复导航的标志位 不加的话目前只有从避障恢复导航

        RCLCPP_INFO(this->get_logger(), "navigation recovery manul");
        //LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation recovery manul");
    }

    void handleTaskUpdate( const laser_navigation::NavigationTask& task ){
        
        // 检查是否是热更新（导航进行中收到新路径）
        if ( m_machine_state_ == MachineControlState::STATE_MACH_IDLE)
        {
            //LOG4CPLUS_WARN_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "navigation is not in run state");
            return;
        }

        if (is_navigating_) {
            // 热更新：追加路径点
            RCLCPP_INFO(this->get_logger(), 
                "Hot update: appending %zu waypoints (%zu transition points)",
                task.waypoints.size(), task.getTransitionPointCount());
            
            int result = navigator_.update(
                task.waypoints,
                task.constraints,
                task.segment_types,
                task.control_points);
            
            if (result != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update path");
            }
            printNodesInfo("Updated", task);
            return;
        }
    }

    /**
     * @brief 执行导航控制 还需要补充一些因某些事件发生导航状态切换及导航相关处理的逻辑
     */
    laser_navigation::NavigationOutput runNavigation(){
        if (!is_navigating_) {
            return laser_navigation::NavigationOutput();
        }
        
        // 检查SLAM位姿是否有效
        if (!has_slam_pose_ || !current_pose_.valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for SLAM pose...");
            return laser_navigation::NavigationOutput();
        }
        
        // 检查SLAM位姿是否超时（自定义时间（大于1s） 这里假设超过1秒认为失效）
        double current_time = this->now().seconds();
        double slam_pose_age = current_time - last_slam_pose_time_;
        if (slam_pose_age > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "SLAM pose timeout (%.2f s), stopping navigation", slam_pose_age);
            // 发布停车指令
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
            return laser_navigation::NavigationOutput();
        }
        
        // 执行导航（使用里程计反馈版本）
        laser_navigation::NavigationOutput output;
        if (use_odom_feedback_) {
            output = navigator_.executeWithOdometry(
                current_pose_,
                obstacle_detected_,
                obstacle_deceleration_,
                m_cancel_requested_,
                current_time);
        } else {
            output = navigator_.execute(
                current_pose_,
                obstacle_detected_,
                obstacle_deceleration_,
                m_cancel_requested_);
        }
        
        // 发布速度指令
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = output.velocity.linear_x;
        cmd.linear.y = output.velocity.linear_y;  // 四转四驱使用，差速底盘此值为0
        cmd.angular.z = output.velocity.angular;
        cmd_pub_->publish(cmd);
        
        // 发布结构化 JSON 状态（兼容 README 中 nav_status 格式）
        
        // 检查是否完成
        if (output.status == laser_navigation::NavigationStatus::kGoalReached) {
            RCLCPP_INFO(this->get_logger(), "Navigation completed!");
            is_navigating_ = false;
            
            // 停止机器人
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
        } else if (output.status == laser_navigation::NavigationStatus::kError) {
            RCLCPP_ERROR(this->get_logger(), "Navigation error!");
            is_navigating_ = false;
            
            // 停止机器人
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
        }
        return output;
    }


    void setNavStatus(laser_navigation::NavigationStatus status)
    {
        m_navigation_status_ = status;
    }
    /**
     * @brief 发布导航状态
     */
    void pubNavStatus()
    {
        std::string navStatus;
        switch (m_navigation_status_)
        {
        case laser_navigation::NavigationStatus::kIdle:
            {
                navStatus = "idle";
            } break;
        case laser_navigation::NavigationStatus::kRunning:
            {
                navStatus = "move_forward";
            } break;
        case laser_navigation::NavigationStatus::kTurnLeft:
            {
                navStatus = "turn_left";
            } break;
        case laser_navigation::NavigationStatus::kTurnRight:
            {
                navStatus = "turn_right";
            } break;
        case laser_navigation::NavigationStatus::kRotating:
            {
                navStatus = "turn_around";
            } break;
        case laser_navigation::NavigationStatus::kReversing:
            {
                navStatus = "move_backward";
            } break;
        case laser_navigation::NavigationStatus::kGoalReached:
            {
                navStatus = "finished";
            } break;
        case laser_navigation::NavigationStatus::kError:
            {
                navStatus = "failed";
            } break;
        case laser_navigation::NavigationStatus::kCancelled:
            {
                navStatus = "cancel";
            } break;
        case laser_navigation::NavigationStatus::kPaused:
            {
                navStatus = "pause_by_manaul";
            } break;
        case laser_navigation::NavigationStatus::kObstaclePaused:
            {
                navStatus = "pause_by_obs";
            } break;
        case laser_navigation::NavigationStatus::kEmergencyStopped:
            {
                navStatus = "pause_by_ems";
            } break;
        case laser_navigation::NavigationStatus::kSegmentReached:
            {
                navStatus = "stage_reached";
            } break;
        default:
            break;
        }

        std::string status;
        try {
            nlohmann::json json_obj;
            json_obj["taskId"] = m_task_id_;
            json_obj["taskUpdateId"] = m_taskUpdateId_;
            json_obj["taskType"] = "nav_status";
            json_obj["taskStatus"] = "FINISHED";
            json_obj["navStatus"] = (int)m_navigation_status_;
            json_obj["taskResult"]["navStatus"] = navStatus;
            status = json_obj.dump();

        } catch (...) {
            // 走到这里的都是字段类型不对，比如：float当成string下发
            RCLCPP_ERROR(this->get_logger(), "construct nav status json failed");
            //LOG4CPLUS_ERROR_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "costruct nav status json failed");
            return;
        }

        if (!status.empty()) {
            std_msgs::msg::String status_msg;
            status_msg.data = status;
            status_pub_->publish(status_msg);
            RCLCPP_INFO(this->get_logger(), "Published nav status: %s", status.c_str());
            //communication::RosTopic::getInstance()->pubTask(STATE_NAV, status);
            //LOG4CPLUS_DEBUG_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "pub status %s", status.c_str());
        }   
    }
    /**
     * @brief 发布导航到达某个站点的状态
     */
    void pubNavStation(const std::string& current, const std::string& target)
    {
        try
        {
            std::string status;
            nlohmann::json json_obj;
            json_obj["taskId"] = m_task_id_;
            json_obj["taskUpdateId"] = m_taskUpdateId_;
            json_obj["taskType"] = "nav_station";
            json_obj["taskStatus"] = "FINISHED";
            json_obj["taskResult"]["currentPos"] = current;
            json_obj["taskResult"]["targetPos"] = target;
            status = json_obj.dump();

            std_msgs::msg::String status_msg;
            status_msg.data = status;
            status_pub_->publish(status_msg);
            RCLCPP_INFO(this->get_logger(), "Published nav station: %s", status.c_str());
            //communication::RosTopic::getInstance()->pubTask(STATE_NAV, status);
            //LOG4CPLUS_DEBUG_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "pub station %s", status.c_str());
        }
        catch (...)
        {
            // 走到这里的都是字段类型不对，比如：float当成string下发
            //LOG4CPLUS_ERROR_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "costruct nav station json failed");
            RCLCPP_ERROR(this->get_logger(), "construct nav station json failed");
            return;
        }
    }

    void printNodesInfo(const std::string& prex, const laser_navigation::NavigationTask& task)
    {
        std::stringstream ss;
        ss << prex << " nodes: ";
        for (auto &cit : task.waypoints) {
            ss << "{" << cit.x << "," << cit.y << "," << cit.yaw << "} ";
        }
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("NavPos"), "%s\n", ss.str().c_str());
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "%s", ss.str().c_str());

        std::stringstream sss;
        sss << prex << " edges: ";

        for( int i=0; i<task.constraints.size() && task.constraints.size() == task.segment_types.size(); i++ )
        {
            sss << "{" 
                << "maxjerk=" << task.constraints[i].max_jerk << ", "
                << "maxspeed=" << task.constraints[i].max_velocity << ", " << "maxacc=" << task.constraints[i].max_acceleration << ", " << "maxdec=" << task.constraints[i].max_deceleration << ", "
                << "maxRot=" << task.constraints[i].max_angular_velocity << ", " << "maxRotAcc=" << task.constraints[i].max_angular_acceleration << ", " << "maxRotDec=" << task.constraints[i].max_angular_deceleration << ", "
                << "direction=" << task.constraints[i].is_forward << ", " << "reachAngle=" << task.constraints[i].reach_angle << ", " << "reachDist=" << task.constraints[i].reach_distance << ", "
                << "type=" << (int)task.segment_types[i];
            if (task.segment_types[i] == laser_navigation::PathSegmentType::kCubicBezier)
            {
                sss << ", " << "ctrlPoints=[";
                for (auto &iter : task.control_points[i])
                {
                    sss << "(" << iter.x << "," << iter.y << ")";
                }
                sss << "]";
            }
            sss << "} ";
        }

        
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("NavPos"), "%s\n", sss.str().c_str());
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "%s", sss.str().c_str());

        std::stringstream ssss;
        ssss << prex << " params: ";
        ssss 
            << "startAngleAdjust=" << std::boolalpha << task.start_angle_adjust << ", "
            << "endAngleAdjust=" << task.end_angle_adjust
            ;
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("NavPos"), "%s\n", ssss.str().c_str());
        // LOG4CPLUS_INFO_FMT(log4cplus::Logger::getInstance("LaserNavNode"), "%s", ssss.str().c_str());
    }


private:
    // 导航器
    laser_navigation::NavigationExecutor navigator_;
    laser_navigation::JsonParser json_parser_;
    
    // 配置文件中读取的参数（临时存储）
    std::map<std::string, double> config_lqr_params_;
    std::map<std::string, double> config_bezier_lqr_params_;
    
    // 参数
    laser_navigation::LQRParams lqr_params_;
    laser_navigation::BezierLQRParams bezier_lqr_params_;
    laser_navigation::MotionConstraints default_constraints_;
    double control_period_{0.04};
    laser_navigation::ChassisType chassis_type_{laser_navigation::ChassisType::kDifferential};
    bool use_odom_feedback_{true};
    double odom_timeout_{0.2};
    
    // 任务状态
    laser_navigation::Pose2D current_pose_;       // 当前位姿（来自SLAM）
    bool has_slam_pose_{false};                   // 是否收到SLAM位姿（必须）
    bool has_odom_velocity_{false};               // 是否收到里程计速度（可选）
    double last_slam_pose_time_{0.0};             // 上次收到SLAM位姿的时间
    bool is_navigating_{false};
    bool obstacle_detected_{false};
    bool m_cancel_requested_{false};
    double obstacle_deceleration_{0.5};
    std::string m_task_id_;
    int m_taskUpdateId_{0};
    std::string curTask_;
    std::string rotateMode_{"odom"};

    // 机器状态 导航反馈状态
    MachineControlState m_machine_state_{STATE_MACH_IDLE};
    laser_navigation::NavigationStatus m_navigation_status_{laser_navigation::NavigationStatus::kIdle};

    
    // ROS接口
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_slam_pose_sub_;  // SLAM位姿（必须）
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub_;               // 里程计速度（可选）
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_EmergedStop_sub_;               // 急停订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr codes_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationRefactoredNode>());
    rclcpp::shutdown();
    return 0;
}
