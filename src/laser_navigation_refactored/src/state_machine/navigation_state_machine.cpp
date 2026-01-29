/**
 * @file navigation_state_machine.cpp
 * @brief 导航状态机实现
 */

#include "laser_navigation_refactored/state_machine/navigation_state_machine.hpp"

namespace laser_navigation {

NavigationStateMachine::NavigationStateMachine()
    : current_state_(StateMachineState::kIdle) {}

bool NavigationStateMachine::processEvent(StateMachineEvent event) {
    StateMachineState next_state = current_state_;

    // 状态转换逻辑
    // 对应原代码中分散在 cubicCurve 函数各处的状态判断
    switch (current_state_) {
        case StateMachineState::kIdle:
            if (event == StateMachineEvent::kStart) {
                // 开始导航时，根据配置决定是否先旋转
                next_state = need_start_rotation_ ? 
                    StateMachineState::kStartRotation : StateMachineState::kFollowingPath;
            }
            break;

        case StateMachineState::kStartRotation:
            if (event == StateMachineEvent::kRotationComplete) {
                // 起始旋转完成，开始跟踪路径
                next_state = StateMachineState::kFollowingPath;
            } else if (event == StateMachineEvent::kObstacleDetected ||
                       event == StateMachineEvent::kCancel) {
                // 旋转时检测到障碍物，进入减速状态
                next_state = StateMachineState::kDecelerating;
            } else if (event == StateMachineEvent::kEmergencyStop) {
                next_state = StateMachineState::kPaused;
            } else if (event == StateMachineEvent::kError) {
                next_state = StateMachineState::kError;
            }
            break;

        case StateMachineState::kFollowingPath:
            if (event == StateMachineEvent::kSegmentComplete) {
                // 段完成，继续下一段（状态不变）
                // 具体逻辑在执行器中处理
            } else if (event == StateMachineEvent::kPathComplete) {
                // 路径完成，检查是否需要终点旋转
                next_state = need_end_rotation_ ? 
                    StateMachineState::kEndRotation : StateMachineState::kCompleted;
            } else if (event == StateMachineEvent::kObstacleDetected) {
                // 检测到障碍物，进入减速状态
                next_state = StateMachineState::kDecelerating;
            } else if (event == StateMachineEvent::kEmergencyStop) {
                next_state = StateMachineState::kPaused;
            } else if (event == StateMachineEvent::kCancel) {
                // 取消时先减速
                next_state = StateMachineState::kDecelerating;
            } else if (event == StateMachineEvent::kError) {
                next_state = StateMachineState::kError;
            }
            break;

        case StateMachineState::kEndRotation:
            if (event == StateMachineEvent::kRotationComplete) {
                // 终点旋转完成
                next_state = StateMachineState::kCompleted;
            } else if (event == StateMachineEvent::kObstacleDetected ||
                       event == StateMachineEvent::kCancel) {
                next_state = StateMachineState::kDecelerating;
            } else if (event == StateMachineEvent::kEmergencyStop) {
                next_state = StateMachineState::kPaused;
            } else if (event == StateMachineEvent::kError) {
                next_state = StateMachineState::kError;
            }
            break;

        case StateMachineState::kDecelerating:
            if (event == StateMachineEvent::kObstacleCleared) {
                // 障碍物消除，恢复路径跟踪
                next_state = StateMachineState::kFollowingPath;
            }else if ( event == StateMachineEvent::kResume ) {
                // 恢复导航
                next_state = StateMachineState::kFollowingPath;

            } else if (event == StateMachineEvent::kEmergencyStop) {
                next_state = StateMachineState::kPaused;
            } else if (event == StateMachineEvent::kCancel) {
                // 已经在减速，继续减速直到停止
                // 状态不变，等待速度降为0后外部处理
            }
            break;

        case StateMachineState::kPaused:
            if (event == StateMachineEvent::kResume) {
                // 恢复导航
                next_state = StateMachineState::kFollowingPath;
            } else if (event == StateMachineEvent::kCancel) {
                // 在暂停状态取消，直接进入空闲 ？ 万一没停下呢，除非这里表示已经完全停下
                next_state = StateMachineState::kIdle;
            } else if (event == StateMachineEvent::kError) {
                next_state = StateMachineState::kError;
            }
            break;

        case StateMachineState::kCompleted:
            if (event == StateMachineEvent::kStart) {
                // 允许从完成状态重新开始
                next_state = StateMachineState::kIdle;
            }
            break;

        case StateMachineState::kError:
            if (event == StateMachineEvent::kStart) {
                // 允许从错误状态重新开始
                next_state = StateMachineState::kIdle;
            }
            break;
    }

    // 执行状态转换
    if (next_state != current_state_) {
        transitionTo(next_state);
        return true;
    }
    return false;
}

void NavigationStateMachine::transitionTo(StateMachineState new_state) {
    StateMachineState old_state = current_state_;
    current_state_ = new_state;
    
    // 触发回调
    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
}

bool NavigationStateMachine::isValidTransition(StateMachineState /*from*/, 
                                                StateMachineState /*to*/) const {
    //TODO 可以在这里添加更严格的状态转换验证
    // 目前的逻辑已经在 processEvent 中处理
    return true;
}

std::string NavigationStateMachine::getStateName(StateMachineState state) {
    switch (state) {
        case StateMachineState::kIdle: return "Idle";
        case StateMachineState::kStartRotation: return "StartRotation";
        case StateMachineState::kFollowingPath: return "FollowingPath";
        case StateMachineState::kEndRotation: return "EndRotation";
        case StateMachineState::kDecelerating: return "Decelerating";
        case StateMachineState::kPaused: return "Paused";
        case StateMachineState::kCompleted: return "Completed";
        case StateMachineState::kError: return "Error";
        default: return "Unknown";
    }
}

std::string NavigationStateMachine::getEventName(StateMachineEvent event) {
    switch (event) {
        case StateMachineEvent::kStart: return "Start";
        case StateMachineEvent::kRotationComplete: return "RotationComplete";
        case StateMachineEvent::kSegmentComplete: return "SegmentComplete";
        case StateMachineEvent::kPathComplete: return "PathComplete";
        case StateMachineEvent::kObstacleDetected: return "ObstacleDetected";
        case StateMachineEvent::kObstacleCleared: return "ObstacleCleared";
        case StateMachineEvent::kEmergencyStop: return "EmergencyStop";
        case StateMachineEvent::kResume: return "Resume";
        case StateMachineEvent::kCancel: return "Cancel";
        case StateMachineEvent::kError: return "Error";
        default: return "Unknown";
    }
}

void NavigationStateMachine::reset() {
    current_state_ = StateMachineState::kIdle;
    need_start_rotation_ = false;
    need_end_rotation_ = false;
}

bool NavigationStateMachine::isActive() const {
    return current_state_ != StateMachineState::kIdle &&
           current_state_ != StateMachineState::kCompleted &&
           current_state_ != StateMachineState::kError;
}

}  // namespace laser_navigation
