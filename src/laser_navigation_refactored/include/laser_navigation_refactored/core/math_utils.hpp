/**
 * @file math_utils.hpp
 * @brief 数学工具函数集合
 * 
 * 提供角度处理、符号函数等常用数学工具
 */

#ifndef LASER_NAVIGATION_REFACTORED_CORE_MATH_UTILS_HPP_
#define LASER_NAVIGATION_REFACTORED_CORE_MATH_UTILS_HPP_

#include <cmath>
#include <algorithm>

namespace laser_navigation {
namespace math {

/**
 * @brief 数学工具函数集合
 */
class MathUtils {
public:
    /**
     * @brief 角度归一化到 [-π, π]
     * @param angle 输入角度 (rad)
     * @return 归一化后的角度
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief 计算两个角度的最短角度差（带符号）
     * @param to_angle 目标角度
     * @param from_angle 起始角度
     * @return 从 from_angle 到 to_angle 的最短路径角度差
     */
    static double angleDifference(double to_angle, double from_angle) {
        return normalizeAngle(to_angle - from_angle);
    }

    /**
     * @brief 计算两个角度的绝对差值
     * @param angle1 角度1
     * @param angle2 角度2
     * @return 绝对角度差 [0, π]
     */
    static double absoluteAngleDiff(double angle1, double angle2) {
        double diff = std::abs(normalizeAngle(angle1) - normalizeAngle(angle2));
        if (diff > M_PI) {
            diff = 2.0 * M_PI - diff;
        }
        return diff;
    }

    /**
     * @brief 判断旋转方向
     * @param current_angle 当前角度
     * @param target_angle 目标角度
     * @return 1=逆时针（正向）, -1=顺时针（反向）
     */
    static int rotationDirection(double current_angle, double target_angle) {
        double diff = normalizeAngle(target_angle - current_angle);
        return (diff >= 0) ? 1 : -1;
    }

    /**
     * @brief 符号函数
     * @param val 输入值
     * @return 1（正）, -1（负）, 0（零或NaN）
     */
    template<typename T>
    static int sign(T val) {
        if (std::isnan(static_cast<double>(val))) return 0;
        if (val > T(0)) return 1;
        if (val < T(0)) return -1;
        return 0;
    }

    /**
     * @brief 判断浮点数是否接近零
     * @param value 输入值
     * @param epsilon 容差
     */
    static bool isNearZero(double value, double epsilon = 1e-9) {
        return std::abs(value) < epsilon;
    }

    /**
     * @brief 限制值在范围内
     * @param value 输入值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return 限制后的值
     */
    template<typename T>
    static T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

    /**
     * @brief 计算组合数 C(n, k)
     * @param n 总数
     * @param k 选取数
     * @return 组合数
     */
    static double binomial(int n, int k) {
        if (k < 0 || k > n) return 0.0;
        if (k == 0 || k == n) return 1.0;
        
        double result = 1.0;
        for (int i = 0; i < k; ++i) {
            result = result * (n - i) / (i + 1);
        }
        return result;
    }

    /**
     * @brief 计算两点间距离
     * @param x1, y1 点1坐标
     * @param x2, y2 点2坐标
     * @return 欧氏距离
     */
    static double distance(double x1, double y1, double x2, double y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    /**
     * @brief 线性插值
     * @param a 起始值
     * @param b 终止值
     * @param t 插值参数 [0, 1]
     * @return 插值结果
     */
    static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    /**
     * @brief 角度线性插值（考虑周期性）
     * @param a 起始角度
     * @param b 终止角度
     * @param t 插值参数 [0, 1]
     * @return 插值角度
     */
    static double lerpAngle(double a, double b, double t) {
        double diff = angleDifference(b, a);
        return normalizeAngle(a + t * diff);
    }

    /**
     * @brief 角度转弧度
     */
    static double degToRad(double degrees) {
        return degrees * M_PI / 180.0;
    }

    /**
     * @brief 弧度转角度
     */
    static double radToDeg(double radians) {
        return radians * 180.0 / M_PI;
    }

    /**
     * @brief 计算向量叉积的z分量（2D）
     */
    static double cross2D(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    /**
     * @brief 计算向量点积（2D）
     */
    static double dot2D(double ax, double ay, double bx, double by) {
        return ax * bx + ay * by;
    }

    /**
     * @brief 四元数转欧拉角（yaw）rad
     */
    static double quaternionToYaw(double qx, double qy, double qz, double qw) {
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

}  // namespace math
}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_CORE_MATH_UTILS_HPP_
