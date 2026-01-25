# laser_navigation_refactored 扩展指南

## 项目概述

本项目是一个模块化的导航控制功能包，支持多种底盘类型（差速/四转四驱）和多种轨迹类型（直线/贝塞尔/圆弧/Dubins/Reeds-Shepp）。

## 输入输出说明

### 输入

| 输入 | 类型 | 描述 | 来源 |
|------|------|------|------|
| `NavigationConfig` | 结构体 | 导航配置 | 用户/JSON |
| ├─ `waypoints` | `vector<Pose2D>` | 路径点列表 | 路径规划 |
| ├─ `constraints` | `vector<MotionConstraints>` | 运动约束 | 用户配置 |
| ├─ `segment_types` | `vector<PathSegmentType>` | 轨迹类型 | 用户配置 |
| ├─ `control_points` | `vector<vector<Pose2D>>` | 控制点/参数 | 用户/自动生成 |
| └─ `chassis_type` | `ChassisType` | 底盘类型 | 用户配置 |
| `current_pose` | `Pose2D` | 当前位姿 `(x, y, yaw)` | 定位系统 |
| `odom_feedback` | `OdometryFeedback` | 里程计反馈（可选） | 编码器 |
| `obstacle_stop` | `bool` | 障碍物停止标志 | 感知模块 |
| `obstacle_decel` | `double` | 减速系数 | 感知模块 |

### 输出

| 输出 | 类型 | 描述 | 去向 |
|------|------|------|------|
| `NavigationOutput` | 结构体 | 导航输出 | 底盘控制 |
| ├─ `velocity.linear_x` | `double` | x方向线速度 (m/s) | 差速/四驱 |
| ├─ `velocity.linear_y` | `double` | y方向线速度 (m/s) | 仅四驱 |
| ├─ `velocity.angular` | `double` | 角速度 (rad/s) | 差速/四驱 |
| ├─ `status` | `NavigationStatus` | 导航状态 | 状态监控 |
| └─ `remaining_distance` | `double` | 剩余距离 | 显示/监控 |

## 添加新轨迹类型指南

### 步骤 1: 添加类型枚举

在 `include/laser_navigation_refactored/core/types.hpp` 中添加新类型：

```cpp
enum class PathSegmentType {
    kStraight = 0,
    kCircularArc = 1,
    kCubicBezier = 2,
    kDubins = 3,
    kReedsShepp = 4,
    kClothoid = 5,      // 新增：回旋曲线
    kMyCustomPath = 6   // 新增：自定义轨迹
};
```

### 步骤 2: 创建新的路径段类

创建头文件 `include/laser_navigation_refactored/path/my_custom_segment.hpp`：

```cpp
#ifndef LASER_NAVIGATION_REFACTORED_PATH_MY_CUSTOM_SEGMENT_HPP_
#define LASER_NAVIGATION_REFACTORED_PATH_MY_CUSTOM_SEGMENT_HPP_

#include "laser_navigation_refactored/path/path_segment.hpp"

namespace laser_navigation {

class MyCustomSegment : public PathSegment {
public:
    MyCustomSegment(const Pose2D& start, const Pose2D& end, 
                    double custom_param);
    
    // 必须实现的虚函数
    PathSegmentType getType() const override { 
        return PathSegmentType::kMyCustomPath; 
    }
    double getTotalLength() const override;
    Pose2D getStartPose() const override;
    Pose2D getEndPose() const override;
    double getStartHeading() const override;
    double getEndHeading() const override;
    double getRemainingDistance(const Pose2D& current_pose) const override;
    bool isReached(const Pose2D& current_pose, double threshold) const override;
    double getLateralDeviation(const Pose2D& current_pose) const override;
    
    // 自定义方法
    Pose2D getPoseAtParameter(double t) const;
    
private:
    Pose2D start_pose_;
    Pose2D end_pose_;
    double custom_param_;
    double total_length_;
};

}  // namespace laser_navigation

#endif
```

创建实现文件 `src/path/my_custom_segment.cpp`：

```cpp
#include "laser_navigation_refactored/path/my_custom_segment.hpp"

namespace laser_navigation {

MyCustomSegment::MyCustomSegment(const Pose2D& start, const Pose2D& end,
                                  double custom_param)
    : start_pose_(start)
    , end_pose_(end)
    , custom_param_(custom_param) {
    // 计算总长度
    total_length_ = computeLength();
}

// 实现其他虚函数...

}  // namespace laser_navigation
```

### 步骤 3: 更新 CMakeLists.txt

```cmake
set(NAV_CORE_SOURCES
  src/path/straight_segment.cpp
  src/path/bezier_segment.cpp
  src/path/arc_segment.cpp
  src/path/my_custom_segment.cpp  # 新增
  ...
)
```

### 步骤 4: 注册到工厂类

在 `path_segment_factory.hpp` 的 `registerBuiltinTypes()` 中添加：

```cpp
registerCreator(PathSegmentType::kMyCustomPath,
    [](const Pose2D& start, const Pose2D& end,
       const MotionConstraints& constraints,
       const std::vector<Pose2D>& params) {
        double custom_param = params.empty() ? 1.0 : params[0].x;
        auto segment = std::make_shared<MyCustomSegment>(start, end, custom_param);
        segment->setConstraints(constraints);
        return std::static_pointer_cast<PathSegment>(segment);
    });
```

### 步骤 5: 更新 JSON 解析器（可选）

在 `utils/json_parser.hpp` 的 `parseTrajectory()` 中添加：

```cpp
} else if (type_str == "MyCustom" || type_str == "custom") {
    type = PathSegmentType::kMyCustomPath;
    if (traj_obj.contains("customParam")) {
        Pose2D param;
        param.x = traj_obj["customParam"].get<double>();
        ctrl_points.push_back(param);
    }
}
```

### 步骤 6: 更新 NavigationExecutor（如需特殊处理）

如果新轨迹类型需要特殊的跟踪逻辑，在 `navigation_executor.cpp` 的 `handlePathFollowing()` 中添加处理分支：

```cpp
if (segment->getType() == PathSegmentType::kMyCustomPath) {
    output = trackMyCustomSegment(current_pose, 
        dynamic_cast<MyCustomSegment*>(segment), ...);
}
```

## C++ 标准兼容性

当前项目使用 **C++11**，通过自定义 `Optional<T>` 类替代 `std::optional`。

### 切换到 C++17

如需使用 C++17：

1. 修改 `CMakeLists.txt`：
   ```cmake
   set(CMAKE_CXX_STANDARD 17)
   ```

2. （可选）将 `Optional<T>` 替换为 `std::optional<T>`

### 当前 C++11 兼容实现

- `Optional<T>` 类位于 `core/optional.hpp`
- 提供与 `std::optional` 相同的接口：`has_value()`, `value()`, `value_or()`, `operator*`, `operator->`
- 使用 `nullopt` 表示空值
- 使用 `make_optional()` 创建值

## 支持的轨迹类型

| 类型 | 枚举值 | 描述 | 特点 |
|------|--------|------|------|
| 直线 | `kStraight` | 两点间直线 | 最简单，效率最高 |
| 贝塞尔 | `kCubicBezier` | 三次贝塞尔曲线 | 平滑过渡，支持自动生成控制点 |
| 圆弧 | `kCircularArc` | 简单圆弧 | 恒定曲率 |
| Dubins | `kDubins` | Dubins曲线 | 最短前进路径，CSC/CCC组合 |
| Reeds-Shepp | `kReedsShepp` | RS曲线 | 最短路径，支持倒车 |

## JSON 配置示例

```json
{
  "task_id": "nav_001",
  "nodes": [
    {"id": "A", "x": 0.0, "y": 0.0, "yaw": 0.0},
    {"id": "B", "x": 5.0, "y": 3.0, "yaw": 0.785},
    {"id": "C", "x": 8.0, "y": 0.0, "yaw": 0.0}
  ],
  "edges": [
    {
      "from": "A", "to": "B",
      "trajectory": {
        "type": "CubicBezier"
      },
      "constraints": {"maxVelocity": 0.5}
    },
    {
      "from": "B", "to": "C",
      "trajectory": {
        "type": "Dubins",
        "curvature": 0.3
      },
      "constraints": {"maxVelocity": 0.4}
    }
  ]
}
```
