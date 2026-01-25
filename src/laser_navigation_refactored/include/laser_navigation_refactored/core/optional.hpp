/**
 * @file optional.hpp
 * @brief C++11 兼容的 Optional 实现
 * 
 * 提供类似 std::optional (C++17) 的功能，但兼容 C++11
 */

#ifndef LASER_NAVIGATION_REFACTORED_CORE_OPTIONAL_HPP_
#define LASER_NAVIGATION_REFACTORED_CORE_OPTIONAL_HPP_

#include <stdexcept>
#include <utility>

namespace laser_navigation {

/**
 * @brief 空值标记类型
 */
struct nullopt_t {
    explicit constexpr nullopt_t(int) {}
};

/**
 * @brief 空值常量
 */
constexpr nullopt_t nullopt{0};

/**
 * @brief C++11 兼容的 Optional 类
 * 
 * @tparam T 存储的值类型
 */
template <typename T>
class Optional {
public:
    /**
     * @brief 默认构造（无值状态）
     */
    Optional() : has_value_(false) {}
    
    /**
     * @brief 从 nullopt 构造（无值状态）
     */
    Optional(nullopt_t) : has_value_(false) {}
    
    /**
     * @brief 从值构造
     */
    Optional(const T& value) : has_value_(true) {
        new (&storage_) T(value);
    }
    
    /**
     * @brief 移动构造
     */
    Optional(T&& value) : has_value_(true) {
        new (&storage_) T(std::move(value));
    }
    
    /**
     * @brief 拷贝构造
     */
    Optional(const Optional& other) : has_value_(other.has_value_) {
        if (has_value_) {
            new (&storage_) T(other.value());
        }
    }
    
    /**
     * @brief 移动构造
     */
    Optional(Optional&& other) : has_value_(other.has_value_) {
        if (has_value_) {
            new (&storage_) T(std::move(other.value()));
            other.reset();
        }
    }
    
    /**
     * @brief 析构函数
     */
    ~Optional() {
        reset();
    }
    
    /**
     * @brief 拷贝赋值
     */
    Optional& operator=(const Optional& other) {
        if (this != &other) {
            reset();
            has_value_ = other.has_value_;
            if (has_value_) {
                new (&storage_) T(other.value());
            }
        }
        return *this;
    }
    
    /**
     * @brief 移动赋值
     */
    Optional& operator=(Optional&& other) {
        if (this != &other) {
            reset();
            has_value_ = other.has_value_;
            if (has_value_) {
                new (&storage_) T(std::move(other.value()));
                other.reset();
            }
        }
        return *this;
    }
    
    /**
     * @brief 从值赋值
     */
    Optional& operator=(const T& value) {
        reset();
        has_value_ = true;
        new (&storage_) T(value);
        return *this;
    }
    
    /**
     * @brief 从 nullopt 赋值（重置）
     */
    Optional& operator=(nullopt_t) {
        reset();
        return *this;
    }
    
    /**
     * @brief 检查是否有值
     */
    bool has_value() const { return has_value_; }
    
    /**
     * @brief 隐式转换为 bool
     */
    explicit operator bool() const { return has_value_; }
    
    /**
     * @brief 获取值引用（检查有效性）
     */
    T& value() {
        if (!has_value_) {
            throw std::runtime_error("Optional has no value");
        }
        return *reinterpret_cast<T*>(&storage_);
    }
    
    /**
     * @brief 获取值常引用（检查有效性）
     */
    const T& value() const {
        if (!has_value_) {
            throw std::runtime_error("Optional has no value");
        }
        return *reinterpret_cast<const T*>(&storage_);
    }
    
    /**
     * @brief 获取值或默认值
     */
    T value_or(const T& default_value) const {
        return has_value_ ? value() : default_value;
    }
    
    /**
     * @brief 解引用运算符
     */
    T& operator*() { return value(); }
    const T& operator*() const { return value(); }
    
    /**
     * @brief 成员访问运算符
     */
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }
    
    /**
     * @brief 重置为无值状态
     */
    void reset() {
        if (has_value_) {
            reinterpret_cast<T*>(&storage_)->~T();
            has_value_ = false;
        }
    }
    
    /**
     * @brief 原地构造值
     */
    template <typename... Args>
    void emplace(Args&&... args) {
        reset();
        new (&storage_) T(std::forward<Args>(args)...);
        has_value_ = true;
    }

private:
    typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
    bool has_value_;
};

/**
 * @brief 创建 Optional 的辅助函数
 */
template <typename T>
Optional<T> make_optional(const T& value) {
    return Optional<T>(value);
}

template <typename T>
Optional<T> make_optional(T&& value) {
    return Optional<T>(std::forward<T>(value));
}

}  // namespace laser_navigation

#endif  // LASER_NAVIGATION_REFACTORED_CORE_OPTIONAL_HPP_
