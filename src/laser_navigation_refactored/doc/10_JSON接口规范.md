# 10. JSON 接口规范

本章详细定义导航任务的 JSON 配置格式规范。

## 10.1 配置文件说明

本项目使用**两类配置文件**，分离了「启动配置」和「运行时导航任务」：

### 10.1.1 model.json - 底盘类型配置（节点启动时读取）

底盘类型**不在导航任务JSON中指定**，而是在节点启动时从 `model.json` 配置文件中读取：

```json
{
    "agvType": "4WIS4WID"
}
```

| agvType 值 | 底盘类型 | 说明 |
|-----------|---------|------|
| `4WIS4WID` | 四转四驱 | 支持全向移动，无需原地旋转 |
| `DiffDrive` | 双轮差速 | 传统差速底盘，不支持横向移动 |

### 10.1.2 navigation_config.json - 导航参数配置（节点启动时读取）

导航相关参数（LQR、运动约束等）也可以在启动时从配置文件读取：

```json
{
    "control_frequency": 25.0,
    "use_odom_feedback": true,
    "odom_timeout": 0.2,
    
    "motion_constraints": {
        "max_velocity": 0.5,
        "max_angular_velocity": 0.8,
        "max_acceleration": 0.2,
        "max_angular_acceleration": 0.8,
        "max_lateral_velocity": 0.3,
        "reach_distance": 0.02,
        "reach_angle": 0.02
    },
    
    "lqr_params": {
        "q1": 10.0, "q2": 10.0, "q3": 10.0,
        "r1": 1.0, "r2": 1.0
    },
    
    "bezier_lqr_params": {
        "q1": 0.5, "q2": 0.5, "q3": 0.005,
        "r1": 1.0, "r2": 1.0
    }
}
```

### 10.1.3 配置文件路径

启动节点时通过ROS参数指定配置文件路径：

```bash
ros2 run laser_navigation_refactored navigation_example_node \
    --ros-args \
    -p model_config_path:=/path/to/model.json \
    -p nav_config_path:=/path/to/navigation_config.json
```

---

## 10.2 导航任务 JSON（实时接收）

导航任务通过 `/navigation_path` topic 实时发送，格式如下：

```json
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "NavigationConfig",
    "type": "object",
    "properties": {
        "adjust_start_angle": {
            "type": "boolean",
            "default": true,
            "description": "启动时是否原地旋转调整到第一段航向（仅差速底盘有效）"
        },
        "adjust_end_angle": {
            "type": "boolean",
            "default": true,
            "description": "到达终点后是否原地旋转到目标航向（仅差速底盘有效）"
        },
        "auto_generate_control_points": {
            "type": "boolean",
            "default": false,
            "description": "是否自动生成贝塞尔控制点"
        },
        "waypoints": {
            "type": "array",
            "minItems": 2,
            "items": { "$ref": "#/definitions/Waypoint" },
            "description": "路径点数组，至少2个点"
        }
    },
    "required": ["waypoints"],
    "definitions": {
        "Waypoint": {
            "type": "object",
            "properties": {
                "x": { "type": "number", "description": "X坐标(米)" },
                "y": { "type": "number", "description": "Y坐标(米)" },
                "yaw": { 
                    "type": "number", 
                    "description": "航向角(弧度)，对于圆弧类型含义特殊" 
                },
                "type": {
                    "type": "string",
                    "enum": ["Straight", "CubicBezier", "CircularArc", 
                             "Dubins", "ReedsShepp"],
                    "default": "Straight"
                },
                "constraints": { "$ref": "#/definitions/Constraints" },
                "controlPoint": { "$ref": "#/definitions/Point2D" },
                "controlPoints": {
                    "type": "array",
                    "items": { "$ref": "#/definitions/Point2D" }
                }
            },
            "required": ["x", "y", "yaw"]
        },
        "Constraints": {
            "type": "object",
            "properties": {
                "max_velocity": { "type": "number", "default": 0.5 },
                "max_acceleration": { "type": "number", "default": 0.3 },
                "max_deceleration": { "type": "number", "default": 0.3 },
                "max_angular_velocity": { "type": "number", "default": 0.5 },
                "max_angular_acceleration": { "type": "number", "default": 0.3 },
                "is_forward": { "type": "boolean", "default": true },
                "reach_distance": { "type": "number", "default": 0.05 },
                "reach_angle": { "type": "number", "default": 0.05 }
            }
        },
        "Point2D": {
            "type": "object",
            "properties": {
                "x": { "type": "number" },
                "y": { "type": "number" }
            },
            "required": ["x", "y"]
        }
    }
}
```

## 10.2 各轨迹类型详解

### 10.2.1 直线段（Straight）

最简单的轨迹类型，连接两个点的直线。

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "Straight",
            "constraints": {
                "max_velocity": 0.5,
                "is_forward": true
            }
        },
        {
            "x": 2.0, "y": 0.0, "yaw": 0.0
        }
    ]
}
```

**说明**:
- `yaw` 表示车辆在该点时的期望航向
- 最后一个点的 `type` 会被忽略（没有下一段）

### 10.2.2 三次贝塞尔曲线（CubicBezier）

平滑曲线，需要两个控制点。

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "CubicBezier",
            "controlPoints": [
                {"x": 0.5, "y": 0.0},
                {"x": 1.5, "y": 1.0}
            ],
            "constraints": {
                "max_velocity": 0.3,
                "is_forward": true
            }
        },
        {
            "x": 2.0, "y": 1.0, "yaw": 1.57
        }
    ]
}
```

**说明**:
- `controlPoints[0]` (P1): 第一控制点，影响起点处曲线方向
- `controlPoints[1]` (P2): 第二控制点，影响终点处曲线方向
- 如果设置 `auto_generate_control_points: true`，可以省略 `controlPoints`

**控制点自动生成**:
```json
{
    "auto_generate_control_points": true,
    "waypoints": [
        {"x": 0.0, "y": 0.0, "yaw": 0.0, "type": "CubicBezier"},
        {"x": 2.0, "y": 1.0, "yaw": 1.57}
    ]
}
```

### 10.2.3 圆弧（CircularArc）

圆弧有两种指定方式：

#### 方式一：曲率模式（yaw < 0）

用曲率值指定圆弧的弯曲程度。

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": -0.5,
            "type": "CircularArc",
            "constraints": {
                "max_velocity": 0.3
            }
        },
        {
            "x": 2.0, "y": 1.0, "yaw": 1.57
        }
    ]
}
```

**说明**:
- `yaw` 的绝对值表示曲率 $\kappa = \frac{1}{R}$
- `yaw < 0` 表示顺时针转弯
- `yaw > 0` 在曲率模式下不会被识别（会进入三点模式）

#### 方式二：三点模式（yaw ≥ 0 + controlPoint）

用起点、中间点、终点三个点确定圆弧。

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "CircularArc",
            "controlPoint": {"x": 1.0, "y": 0.5}
        },
        {
            "x": 2.0, "y": 1.0, "yaw": 1.57
        }
    ]
}
```

**说明**:
- `controlPoint` 指定圆弧上的一个中间点
- 系统自动计算过三点的圆弧
- `yaw` 表示该点处的期望航向（用于后续段的起始）

### 10.2.4 Dubins 曲线

适用于非完整约束车辆（只能前进，有最小转弯半径）。

#### 曲率模式

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": -0.3,
            "type": "Dubins"
        },
        {
            "x": 3.0, "y": 2.0, "yaw": 1.57
        }
    ]
}
```

**说明**:
- `|yaw|` 表示最大曲率（最小转弯半径的倒数）
- 自动规划 LSL, RSR, LSR, RSL, RLR, LRL 六种路径中的最短路径

#### 三点模式

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "Dubins",
            "controlPoint": {"x": 1.5, "y": 1.0}
        },
        {
            "x": 3.0, "y": 2.0, "yaw": 1.57
        }
    ]
}
```

### 10.2.5 Reeds-Shepp 曲线

支持前进和后退的最短路径规划。

#### 曲率模式

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": -0.3,
            "type": "ReedsShepp"
        },
        {
            "x": 3.0, "y": 2.0, "yaw": 1.57
        }
    ]
}
```

#### 三点模式

```json
{
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "ReedsShepp",
            "controlPoint": {"x": 1.5, "y": 1.0}
        },
        {
            "x": 3.0, "y": 2.0, "yaw": 1.57
        }
    ]
}
```

## 10.3 运动约束详解

```json
{
    "constraints": {
        "max_velocity": 0.5,
        "max_acceleration": 0.3,
        "max_deceleration": 0.3,
        "max_angular_velocity": 0.5,
        "max_angular_acceleration": 0.3,
        "is_forward": true,
        "reach_distance": 0.05,
        "reach_angle": 0.05
    }
}
```

| 字段 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `max_velocity` | double | 0.5 | m/s | 最大线速度 |
| `max_acceleration` | double | 0.3 | m/s² | 最大加速度 |
| `max_deceleration` | double | 0.3 | m/s² | 最大减速度 |
| `max_angular_velocity` | double | 0.5 | rad/s | 最大角速度 |
| `max_angular_acceleration` | double | 0.3 | rad/s² | 最大角加速度 |
| `is_forward` | bool | true | - | true=前进，false=后退 |
| `reach_distance` | double | 0.05 | m | 到点距离阈值 |
| `reach_angle` | double | 0.05 | rad | 到点角度阈值 |

## 10.4 完整示例

### 10.5.1 多段混合路径

```json
{
    "adjust_start_angle": true,
    "adjust_end_angle": true,
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "Straight",
            "constraints": {
                "max_velocity": 0.5,
                "is_forward": true
            }
        },
        {
            "x": 2.0, "y": 0.0, "yaw": 0.0,
            "type": "CubicBezier",
            "controlPoints": [
                {"x": 2.5, "y": 0.0},
                {"x": 3.0, "y": 0.5}
            ],
            "constraints": {
                "max_velocity": 0.3,
                "is_forward": true
            }
        },
        {
            "x": 3.0, "y": 1.0, "yaw": 1.57,
            "type": "CircularArc",
            "controlPoint": {"x": 2.5, "y": 1.5}
        },
        {
            "x": 2.0, "y": 2.0, "yaw": 3.14,
            "type": "Straight",
            "constraints": {
                "max_velocity": 0.4,
                "is_forward": false
            }
        },
        {
            "x": 0.0, "y": 2.0, "yaw": 3.14
        }
    ]
}
```

### 10.5.2 四驱四转底盘任务

首先确保 `model.json` 中配置为：
```json
{"agvType": "4WIS4WID"}
```

然后发送导航任务：
```json
{
    "adjust_start_angle": false,
    "adjust_end_angle": false,
    "waypoints": [
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "type": "Straight",
            "constraints": {
                "max_velocity": 0.8,
                "is_forward": true
            }
        },
        {
            "x": 2.0, "y": 1.0, "yaw": 0.0
        }
    ]
}
```

**说明**: 四驱四转底盘可以横移，`adjust_start_angle` 和 `adjust_end_angle` 通常设为 `false`。

### 10.5.3 纯倒车任务

```json
{
    "adjust_start_angle": true,
    "adjust_end_angle": false,
    "waypoints": [
        {
            "x": 5.0, "y": 0.0, "yaw": 0.0,
            "type": "Straight",
            "constraints": {
                "max_velocity": 0.3,
                "is_forward": false
            }
        },
        {
            "x": 0.0, "y": 0.0, "yaw": 0.0
        }
    ]
}
```

## 10.6 yaw 字段含义总结

| 轨迹类型 | yaw 值 | 含义 |
|----------|--------|------|
| Straight | 任意 | 该点期望航向 |
| CubicBezier | 任意 | 该点期望航向 |
| CircularArc | ≥ 0 且有 controlPoint | 该点期望航向 |
| CircularArc | < 0 | 曲率值（绝对值） |
| Dubins | ≥ 0 且有 controlPoint | 该点期望航向 |
| Dubins | < 0 | 最大曲率（绝对值） |
| ReedsShepp | ≥ 0 且有 controlPoint | 该点期望航向 |
| ReedsShepp | < 0 | 最大曲率（绝对值） |

## 10.7 解析错误处理

```cpp
JsonParser parser;
auto config = parser.parseNavigationConfig(json_string);

if (!config.has_value()) {
    std::cerr << "Parse error: " << parser.getLastError() << std::endl;
}
```

**常见错误**:
- `"Missing or invalid 'waypoints' array"` - 缺少 waypoints 数组
- `"At least 2 waypoints required"` - 路径点少于2个
- `"JSON parse error: ..."` - JSON 语法错误

---

[← 上一章：工具类模块](09_工具类模块.md) | [下一章：使用示例 →](11_使用示例.md)
