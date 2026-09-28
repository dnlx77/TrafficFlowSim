#pragma once
#include <cmath>

namespace tfs::core {
    struct Vec2 {
        float x{0.0f};
        float y{0.0f};

        constexpr Vec2() noexcept = default;
        constexpr Vec2(float x_, float y_) noexcept : x(x_), y(y_) {}

        constexpr Vec2 operator+(const Vec2& rhs) const noexcept { return {x + rhs.x, y + rhs.y}; }
        constexpr Vec2 operator-(const Vec2& rhs) const noexcept { return {x - rhs.x, y - rhs.y}; }
        constexpr Vec2 operator*(float s) const noexcept { return {x * s, y * s}; }
        constexpr Vec2 operator/(float s) const noexcept { return {x / s, y / s}; }
        constexpr Vec2& operator+=(const Vec2& rhs) noexcept { x += rhs.x; y += rhs.y; return *this; }
        constexpr Vec2& operator-=(const Vec2& rhs) noexcept { x -= rhs.x; y -= rhs.y; return *this; }

        [[nodiscard]] float dot(const Vec2& rhs) const noexcept { return x * rhs.x + y * rhs.y; }
        [[nodiscard]] float cross(const Vec2& rhs) const noexcept { return x * rhs.y - y * rhs.x; }
        [[nodiscard]] float lengthSq() const noexcept { return x * x + y * y; }
        [[nodiscard]] float length() const noexcept { return std::sqrt(lengthSq()); }
        [[nodiscard]] Vec2 normalized() const noexcept {
            const float len = length();
            if (len < 1e-8f) return {0.0f, 0.0f};
            return {x / len, y / len};
        }
        [[nodiscard]] Vec2 perpRight() const noexcept { return {y, -x}; }
        [[nodiscard]] Vec2 perpLeft() const noexcept { return {-y, x}; }
        [[nodiscard]] float angle() const noexcept { return std::atan2(y, x); }
        [[nodiscard]] float distanceTo(const Vec2& rhs) const noexcept { return (rhs - *this).length(); }
    };

    constexpr Vec2 operator*(float s, const Vec2& v) noexcept { return v * s; }
}
