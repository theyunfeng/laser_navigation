/**
 * @file navigation_state_machine.hpp
 * @brief 导航状态机
 * 
 * 管理导航过程中的状态转换
 */

#ifndef LASER_NAVIGATION_REFACTORED_STATE_MACHINE_NAVIGATION_STATE_MACHINE_HPP_
#define LASER_NAVIGATION_REFACTORED_STATE_MACHINE_NAVIGATION_STATE_MACHINE_HPP_

#include "laser_navigation_refactored/core/types.hpp"
#include <functional>
#include <string>

namespace laser_navigation {

/**
 * @brief 状态机事件
 */
enum class StateMachineEvent {
    kStart,              ///< 开始导航
    kRotationComplete,   ///< 旋转完成
    kSegmentComplete,    ///< 路径段完成
    kPathComplete,       ///< 整条路径完成
    kObstacleDetected,   ///< 检测到障碍物
    kObstacleCleared,    ///< 障碍物消除
    kEmergencyStop,      ///< 紧急停止
    kResume,             ///< 恢复导航
    kCancel,             ///< 取消导航
    kError               ///< 发生错误
};

/**
 * @brief 导航状态机
 * 
 * 管理导航过程中的状态转换，确保状态转换的合法性
 * 对应原代码中分散在各处的状态判断逻辑
 */
class NavigationStateMachine {
public:
    /// 状态变化回调函数类型
    using StateChangeCallback = std::function<void(StateMachineState, StateMachineState)>;

    NavigationStateMachine();

    /**
     * @brief 处理事件，触发状态转换
     * @param event 事件
     * @return 状态是否发生变化
     */
    bool processEvent(StateMachineEvent event);

    /**
     * @brief 获取当前状态
     */
    StateMachineState getCurrentState() const { return current_state_; }

    /**
     * @brief 获取状态名称（用于调试）
     */
    static std::string getStateName(StateMachineState state);

    /**
     * @brief 获取事件名称（用于调试）
     */
    static std::string getEventName(StateMachineEvent event);

    /**
     * @brief 设置状态变化回调
     * @param callback 回调函数
     */
    void setStateChangeCallback(StateChangeCallback callback) {
        state_change_callback_ = callback;
    }

    /**
     * @brief 设置是否需要起始旋转
     */
    void setNeedStartRotation(bool need) { need_start_rotation_ = need; }

    /**
     * @brief 设置是否需要终点旋转
     */
    void setNeedEndRotation(bool need) { need_end_rotation_ = need; }

    /**
     * @brief 检查是否需要起始旋转
     */
    bool needStartRotation() const { return need_start_rotation_; }

    /**
     * @brief 检查是否需要终点旋转
     */
    bool needEndRotation() const { return need_end_rotation_; }

    /**
     * @brief 重置状态机
     */
    void reset();

    /**
     * @brief 检查是否处于活动状态（正在导航）
     */
    bool isActive() const;

    /**
     * @brief 检查是否已完成
     */
    bool isCompleted() const { return current_state_ == StateMachineState::kCompleted; }

    /**
     * @brief 检查是否有错误
     */
    bool hasError() const { return current_state_ == StateMachineState::kError; }

private:
    /**
     * @brief 执行状态转换
     * @param new_state 新状态
     */
    void transitionTo(StateMachineState new_state);

    /**
     * @brief 验证状态转换是否合法
     * @param from 源状态
     * @param to 目标状态
     * @return 是否合法
     */
    bool isValidTransition(StateMachineState from, StateMachineState to) const;

    StateMachineState current_state_;           ///< 当前状态
    StateChangeCallback state_change_callback_; ///< 状态变化回调
    
    bool need_start_rotation_{false};  ///< 是否需要起始旋转
    bool need_end_rotation_{false};    ///< 是否需要终点旋转
};

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_STATE_MACHINE_NAVIGATION_STATE_MACHINE_HPP_
