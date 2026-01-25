# 6. LQR控制器

本章详细介绍LQR（线性二次型调节器）轨迹跟踪控制器的原理和实现。

## 6.1 模块概述

LQR控制器模块提供两种控制器：
- **StraightLQRController**：直线路径跟踪控制器
- **BezierLQRController**：曲线路径跟踪控制器

两者都支持差速底盘和四转四驱底盘，并可选择性地使用里程计反馈优化控制性能。

## 6.2 LQR控制原理

### 6.2.1 状态空间模型

对于差速移动机器人，其运动学模型可线性化为：

$$\dot{X} = A \cdot X + B \cdot U$$

其中：
- 状态向量 $X = [e_x, e_y, e_\theta]^T$（位置误差和航向误差）
- 控制输入 $U = [v, \omega]^T$（线速度和角速度）

### 6.2.2 离散化模型

对于数字控制系统，需要将连续模型离散化：

$$X_{k+1} = A_d \cdot X_k + B_d \cdot U_k$$

其中 $A_d = I + A \cdot dt$，$B_d = B \cdot dt$。

### 6.2.3 代价函数

LQR最小化以下代价函数：

$$J = \sum_{k=0}^{\infty} (X_k^T Q X_k + U_k^T R U_k)$$

其中：
- $Q$：状态权重矩阵（惩罚误差）
- $R$：输入权重矩阵（惩罚控制量）

### 6.2.4 Riccati方程

最优控制增益通过求解离散代数Riccati方程(DARE)获得：

$$P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A$$

控制增益：

$$K = (R + B^T P B)^{-1} B^T P A$$

最优控制律：

$$U = -K \cdot X$$

## 6.3 StraightLQRController

### 6.3.1 类定义

```cpp
class StraightLQRController {
public:
    StraightLQRController() = default;
    explicit StraightLQRController(const LQRParams& params);

    void initialize(double target_heading, double line_k, double line_b,
                    bool is_vertical, double end_x,
                    ChassisType chassis_type = ChassisType::kDifferential);

    Velocity compute(const Pose2D& current_pose, 
                     const Velocity& reference_velocity,
                     bool is_forward);

    Velocity computeWithFeedback(const Pose2D& current_pose, 
                                  const Velocity& reference_velocity,
                                  bool is_forward,
                                  const Optional<OdometryFeedback>& odom_feedback);

    void setParams(const LQRParams& params);
    void setChassisType(ChassisType type);
    void reset();

private:
    Eigen::Matrix3d solveDARE(const Eigen::Matrix3d& A, 
                               const Eigen::Matrix<double, 3, 2>& B,
                               const Eigen::Matrix3d& Q, 
                               const Eigen::Matrix2d& R);
    Eigen::Vector2d computeReferencePoint(const Pose2D& current_pose) const;
    double computeLateralVelocity(double lateral_error, double max_vy) const;

    LQRParams params_;
    ChassisType chassis_type_{ChassisType::kDifferential};
    double target_heading_{0.0};
    double line_k_{0.0}, line_b_{0.0}, end_x_{0.0};
    bool is_vertical_{false};
    bool is_initialized_{false};
    Velocity last_velocity_;
};
```

### 6.3.2 初始化

```cpp
void StraightLQRController::initialize(double target_heading, 
                                        double line_k, double line_b,
                                        bool is_vertical, double end_x,
                                        ChassisType chassis_type) {
    target_heading_ = target_heading;
    line_k_ = line_k;
    line_b_ = line_b;
    is_vertical_ = is_vertical;
    end_x_ = end_x;
    chassis_type_ = chassis_type;
    
    // 构建系统矩阵
    // 状态: [纵向误差, 横向误差, 航向误差]
    // 输入: [线速度, 角速度]
    
    is_initialized_ = true;
}
```

### 6.3.3 控制计算

```cpp
Velocity StraightLQRController::compute(const Pose2D& current_pose,
                                         const Velocity& reference_velocity,
                                         bool is_forward) {
    if (!is_initialized_) return Velocity();
    
    // 1. 计算参考点（当前位置在直线上的投影）
    Eigen::Vector2d ref_point = computeReferencePoint(current_pose);
    
    // 2. 计算误差状态
    double dx = current_pose.x - ref_point.x();
    double dy = current_pose.y - ref_point.y();
    
    // 转换到路径坐标系
    double cos_h = std::cos(target_heading_);
    double sin_h = std::sin(target_heading_);
    
    double e_lon = dx * cos_h + dy * sin_h;   // 纵向误差
    double e_lat = -dx * sin_h + dy * cos_h;  // 横向误差
    double e_yaw = normalizeAngle(current_pose.yaw - target_heading_);
    
    if (!is_forward) {
        e_yaw = normalizeAngle(e_yaw + M_PI);
    }
    
    Eigen::Vector3d X(e_lon, e_lat, e_yaw);
    
    // 3. 构建系统矩阵
    double v_ref = std::abs(reference_velocity.linear_x);
    double dt = params_.dt;
    
    Eigen::Matrix3d A;
    A << 1, 0, 0,
         0, 1, v_ref * dt,
         0, 0, 1;
    
    Eigen::Matrix<double, 3, 2> B;
    B << dt, 0,
         0, 0,
         0, dt;
    
    // 4. 构建权重矩阵
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Q(0,0) = params_.q[0];
    Q(1,1) = params_.q[1];
    Q(2,2) = params_.q[2];
    
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    R(0,0) = params_.r[0];
    R(1,1) = params_.r[1];
    
    // 5. 求解Riccati方程获得最优增益
    Eigen::Matrix3d P = solveDARE(A, B, Q, R);
    Eigen::Matrix<double, 2, 3> K = (R + B.transpose() * P * B).inverse() 
                                    * B.transpose() * P * A;
    
    // 6. 计算控制量
    Eigen::Vector2d U = -K * X;
    
    Velocity output;
    output.linear_x = reference_velocity.linear_x + U(0);
    output.angular = reference_velocity.angular + U(1);
    
    // 7. 四转四驱的横向速度补偿
    if (chassis_type_ == ChassisType::kSwerve4WIS4WID) {
        output.linear_y = computeLateralVelocity(e_lat, 0.3);
    } else {
        output.linear_y = 0.0;
    }
    
    // 8. 限幅
    output.linear_x = clamp(output.linear_x, -1.0, 1.0);
    output.angular = clamp(output.angular, -1.5, 1.5);
    
    last_velocity_ = output;
    return output;
}
```

### 6.3.4 DARE求解

```cpp
Eigen::Matrix3d StraightLQRController::solveDARE(
    const Eigen::Matrix3d& A, 
    const Eigen::Matrix<double, 3, 2>& B,
    const Eigen::Matrix3d& Q, 
    const Eigen::Matrix2d& R) {
    
    Eigen::Matrix3d P = Q;  // 初始猜测
    
    for (int i = 0; i < params_.max_iterations; ++i) {
        Eigen::Matrix3d P_prev = P;
        
        // Riccati迭代
        Eigen::Matrix2d S = R + B.transpose() * P * B;
        Eigen::Matrix<double, 2, 3> K = S.inverse() * B.transpose() * P * A;
        P = Q + A.transpose() * P * A - A.transpose() * P * B * K;
        
        // 收敛检查
        if ((P - P_prev).norm() < params_.tolerance) {
            break;
        }
    }
    
    return P;
}
```

### 6.3.5 四转四驱横向速度

```cpp
double StraightLQRController::computeLateralVelocity(double lateral_error, 
                                                      double max_vy) const {
    // 比例控制 + 限幅
    double k_lat = params_.k_lateral;  // 横向增益
    double vy = k_lat * lateral_error;
    
    return clamp(vy, -max_vy, max_vy);
}
```

## 6.4 BezierLQRController

### 6.4.1 类定义

```cpp
class BezierLQRController {
public:
    BezierLQRController() = default;
    explicit BezierLQRController(const BezierLQRParams& params);

    void setChassisType(ChassisType type);

    Velocity compute(const Pose2D& current_pose, 
                     const Eigen::Vector2d& reference_point,
                     const Velocity& reference_velocity, 
                     double reference_heading,
                     bool is_forward);

    Velocity computeWithFeedback(const Pose2D& current_pose, 
                                  const Eigen::Vector2d& reference_point,
                                  const Velocity& reference_velocity, 
                                  double reference_heading,
                                  bool is_forward,
                                  const Optional<OdometryFeedback>& odom_feedback);

    void setParams(const BezierLQRParams& params);

private:
    BezierLQRParams params_;
    ChassisType chassis_type_{ChassisType::kDifferential};
    Velocity last_velocity_;
};
```

### 6.4.2 曲线跟踪控制

曲线跟踪与直线跟踪的主要区别：
- 参考点是曲线上的移动点，而非投影点
- 参考航向随曲线切线变化
- 需要考虑曲率的影响

```cpp
Velocity BezierLQRController::compute(const Pose2D& current_pose,
                                       const Eigen::Vector2d& reference_point,
                                       const Velocity& reference_velocity,
                                       double reference_heading,
                                       bool is_forward) {
    // 1. 计算位置误差
    double dx = current_pose.x - reference_point.x();
    double dy = current_pose.y - reference_point.y();
    
    // 转换到参考坐标系
    double cos_h = std::cos(reference_heading);
    double sin_h = std::sin(reference_heading);
    
    double e_lon = dx * cos_h + dy * sin_h;   // 纵向误差
    double e_lat = -dx * sin_h + dy * cos_h;  // 横向误差
    
    // 2. 计算航向误差
    double e_yaw = normalizeAngle(current_pose.yaw - reference_heading);
    if (!is_forward) {
        e_yaw = normalizeAngle(e_yaw + M_PI);
    }
    
    // 3. LQR控制计算（简化版本）
    double v_ref = reference_velocity.linear_x;
    double w_ref = reference_velocity.angular;
    
    // 状态反馈
    double dv = -params_.q[0] * e_lon;
    double dw = -params_.q[1] * e_lat / (std::abs(v_ref) + 0.1) 
                - params_.q[2] * e_yaw;
    
    Velocity output;
    output.linear_x = v_ref + dv;
    output.angular = w_ref + dw;
    
    // 4. 四转四驱横向补偿
    if (chassis_type_ == ChassisType::kSwerve4WIS4WID) {
        output.linear_y = params_.k_lateral * e_lat;
        output.linear_y = clamp(output.linear_y, 
                                 -params_.max_lateral_velocity,
                                 params_.max_lateral_velocity);
    }
    
    // 5. 限幅
    output.linear_x = clamp(output.linear_x, -1.5, 1.5);
    output.angular = clamp(output.angular, -2.0, 2.0);
    
    last_velocity_ = output;
    return output;
}
```

## 6.5 里程计反馈集成

### 6.5.1 带反馈的控制

当有里程计反馈时，可以利用实际速度进行更精确的控制：

```cpp
Velocity StraightLQRController::computeWithFeedback(
    const Pose2D& current_pose,
    const Velocity& reference_velocity,
    bool is_forward,
    const Optional<OdometryFeedback>& odom_feedback) {
    
    // 基础LQR控制
    Velocity output = compute(current_pose, reference_velocity, is_forward);
    
    // 如果有有效的里程计反馈
    if (odom_feedback.has_value() && odom_feedback->is_valid) {
        const auto& odom = odom_feedback.value();
        
        // 速度误差前馈补偿
        double v_error = reference_velocity.linear_x - odom.velocity.linear_x;
        double w_error = reference_velocity.angular - odom.velocity.angular;
        
        // 添加前馈项
        double k_ff = 0.3;  // 前馈增益
        output.linear_x += k_ff * v_error;
        output.angular += k_ff * w_error;
        
        // 速度平滑（避免突变）
        double alpha = 0.7;  // 平滑因子
        output.linear_x = alpha * output.linear_x 
                        + (1 - alpha) * last_velocity_.linear_x;
        output.angular = alpha * output.angular 
                       + (1 - alpha) * last_velocity_.angular;
    }
    
    last_velocity_ = output;
    return output;
}
```

### 6.5.2 反馈优势

使用里程计反馈的优势：
1. **更平滑的速度变化**：基于实际速度而非假设速度
2. **更好的干扰抑制**：快速响应外部扰动
3. **减少超调**：避免因模型误差导致的过冲
4. **提高跟踪精度**：闭环反馈减小稳态误差

## 6.6 参数调优指南

### 6.6.1 Q矩阵（状态权重）

```cpp
LQRParams params;
params.q = {10.0, 10.0, 1.0};  // [纵向, 横向, 航向]
```

| 参数 | 增大效果 | 减小效果 |
|-----|---------|---------|
| q[0] (纵向) | 更快消除纵向误差 | 允许更大纵向偏差 |
| q[1] (横向) | 更紧密地跟踪路径 | 允许更大横向偏差 |
| q[2] (航向) | 更快对准航向 | 允许更大航向偏差 |

### 6.6.2 R矩阵（输入权重）

```cpp
params.r = {1.0, 1.0};  // [线速度, 角速度]
```

| 参数 | 增大效果 | 减小效果 |
|-----|---------|---------|
| r[0] | 更平滑的速度变化 | 更快的速度响应 |
| r[1] | 更平滑的转向 | 更快的转向响应 |

### 6.6.3 调参建议

1. **高速场景**：增大Q中的航向权重，减小R中的角速度权重
2. **窄通道**：增大Q中的横向权重
3. **平滑优先**：增大R中的所有权重
4. **精度优先**：增大Q中的所有权重

## 6.7 差速vs四转四驱对比

### 6.7.1 差速底盘

```cpp
// 输出: (vx, ω)
output.linear_x = v_ref + delta_v;
output.linear_y = 0;  // 始终为0
output.angular = w_ref + delta_w;
```

特点：
- 横向误差只能通过转向消除
- 无法原地横移
- 在曲线上需要更多的航向调整

### 6.7.2 四转四驱底盘

```cpp
// 输出: (vx, vy, ω)
output.linear_x = v_ref + delta_v;
output.linear_y = k_lat * lateral_error;  // 横向速度补偿
output.angular = w_ref + delta_w;
```

特点：
- 横向误差可直接通过横向速度消除
- 支持原地横移
- 曲线跟踪更平滑
- 可实现斜向行驶

### 6.7.3 控制效果对比

```
差速底盘跟踪曲线:
     实际轨迹
        ↘
    ●───●───●───●
   /    \    \   \    参考轨迹
  ●      ●    ●   ●
         频繁调整航向

四转四驱跟踪曲线:
     实际轨迹（更贴合）
        ↘
    ●═══●═══●═══●    参考轨迹
   /    |    |   \
  ●     ↓    ↓    ●
        横向补偿
```

---

[← 上一章：速度规划器](05_速度规划器.md) | [下一章：状态机模块 →](07_状态机模块.md)
