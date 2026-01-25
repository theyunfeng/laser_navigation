# laser_navigation_refactored

重构后的激光导航功能包 - 模块化设计

## 目录结构

```
laser_navigation_refactored/
├── include/laser_navigation_refactored/
│   ├── core/
│   │   ├── types.hpp              # 统一类型定义
│   │   └── math_utils.hpp         # 数学工具函数
│   ├── path/
│   │   ├── path_segment.hpp       # 路径段抽象基类
│   │   ├── straight_segment.hpp   # 直线段
│   │   └── bezier_segment.hpp     # 贝塞尔曲线段
│   ├── planner/
│   │   └── s_curve_planner.hpp    # S曲线速度规划器
│   ├── controller/
│   │   └── lqr_controller.hpp     # LQR控制器
│   ├── state_machine/
│   │   └── navigation_state_machine.hpp  # 导航状态机
│   └── navigation_executor.hpp    # 导航执行器（核心类）
├── src/
│   ├── path/
│   │   ├── straight_segment.cpp
│   │   └── bezier_segment.cpp
│   ├── planner/
│   │   └── s_curve_planner.cpp
│   ├── controller/
│   │   └── lqr_controller.cpp
│   ├── state_machine/
│   │   └── navigation_state_machine.cpp
│   ├── navigation_executor.cpp
│   └── example/
│       └── navigation_example_node.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 与原代码的对应关系

| 原代码文件 | 重构后文件 | 说明 |
|-----------|-----------|------|
| `common.h` | `core/types.hpp`, `core/math_utils.hpp` | 类型定义和工具函数分离 |
| `straight.h/cpp` | `path/straight_segment.hpp/cpp`, `planner/s_curve_planner.hpp/cpp` | 直线路径和速度规划分离 |
| `bezier.h/cpp` | `path/bezier_segment.hpp/cpp` | 贝塞尔曲线独立模块 |
| `srotate.h/cpp` | `planner/s_curve_planner.hpp/cpp` | 旋转规划合并到S曲线规划器 |
| `correct.h/cpp` | `navigation_executor.hpp/cpp`, `state_machine/` | 核心导航逻辑重构 |

## 主要改进

### 1. 模块化设计
- **路径段抽象**: `PathSegment` 基类定义统一接口，`StraightSegment` 和 `BezierSegment` 实现具体路径类型
- **规划器独立**: `SCurvePlanner` 独立处理速度规划，支持直线和旋转两种模式
- **控制器独立**: `StraightLQRController` 和 `BezierLQRController` 分别处理不同路径的跟踪控制
- **状态机管理**: `NavigationStateMachine` 集中管理导航状态转换

### 2. 命名规范化
原代码中的变量命名（如 `n`, `k`, `i`, `num`）已改为有意义的名称：
- `n` → `waypoint_count_`, `total_segments_`
- `k` → `stop_point_count_`, `line_k_`
- `num` → `current_segment_index_`

### 3. 注释完善
所有公共接口都添加了Doxygen格式的注释，说明：
- 函数功能
- 参数含义
- 返回值
- 与原代码的对应关系

### 4. 代码风格
遵循 Google C++ 风格指南：
- 类名使用 PascalCase
- 函数名使用 camelCase
- 成员变量使用 snake_case_ 后缀
- 常量使用 kPascalCase

## 使用示例

```cpp
#include "laser_navigation_refactored/navigation_executor.hpp"

// 创建导航执行器
laser_navigation::NavigationExecutor navigator;

// 配置导航
laser_navigation::NavigationConfig config;
config.waypoints = {{0, 0, 0}, {1, 1, 0.785}, {2, 0, 0}};
config.constraints = {default_constraints, default_constraints};
config.segment_types = {PathSegmentType::kStraight, PathSegmentType::kStraight};
config.adjust_start_angle = true;
config.adjust_end_angle = true;

// LQR参数
laser_navigation::LQRParams lqr_params;
lqr_params.q1 = 10.0;
lqr_params.dt = 0.04;

laser_navigation::BezierLQRParams bezier_params;

// 初始化
navigator.initialize(config, lqr_params, bezier_params);

// 控制循环
while (navigator.isNavigating()) {
    Pose2D current_pose = getCurrentPose();
    auto output = navigator.execute(current_pose, false, 0.5, false);
    
    // 发布速度指令
    publishVelocity(output.velocity);
}
```

## 编译

```bash
cd /home/bydabc/Codes_zqm/4WIS_4WID_ws
colcon build --packages-select laser_navigation_refactored
```

## 运行示例节点

```bash
source install/setup.bash
ros2 run laser_navigation_refactored navigation_example_node
```

## 依赖

- ROS 2 Humble
- Eigen3
- rclcpp
- geometry_msgs
- nav_msgs
- std_msgs
