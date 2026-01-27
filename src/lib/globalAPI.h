#pragma once
#include "main.h"
#include <string>
#include <cmath>

//| MARK: Predefs
struct Vec2;
struct W_Vec2;





//| MARK: Constants
constexpr double PI = 3.14159265358979323846;
constexpr float FRAME = {10}; // Frame time




//| MARK: Structs
struct Vec2 {
    float x;
    float y;

    constexpr Vec2(float _x = 0.0f, float _y = 0.0f) noexcept
        : x(_x), y(_y) {}

    // Vec2 <op> Vec2
    constexpr inline Vec2 operator+(const Vec2& a) const noexcept { return {x + a.x, y + a.y}; }
    constexpr inline Vec2 operator-(const Vec2& a) const noexcept { return {x - a.x, y - a.y}; }
    constexpr inline Vec2 operator*(const Vec2& a) const noexcept { return {x * a.x, y * a.y}; }
    constexpr inline Vec2 operator/(const Vec2& a) const noexcept { return {x / a.x, y / a.y}; }

    // Vec2 <op> float
    constexpr inline Vec2 operator*(const float& a) const noexcept { return {x * a, y * a}; }
    constexpr inline Vec2 operator/(const float& a) const noexcept { return {x / a, y / a}; }

    // Compound assignments for Vec2
    constexpr inline Vec2& operator+=(const Vec2& a) noexcept { x += a.x; y += a.y; return *this; }
    constexpr inline Vec2& operator-=(const Vec2& a) noexcept { x -= a.x; y -= a.y; return *this; }
    constexpr inline Vec2& operator*=(const Vec2& a) noexcept { x *= a.x; y *= a.y; return *this; }
    constexpr inline Vec2& operator/=(const Vec2& a) noexcept { x /= a.x; y /= a.y; return *this; }
    constexpr inline Vec2& operator*=(const float& a) noexcept { x *= a; y *= a;    return *this; }
    constexpr inline Vec2& operator/=(const float& a) noexcept { x /= a; y /= a;    return *this; }

};
// Contains leftSpeed and rightSpeed as a pair
struct SpeedPair {
    float leftSpeed;
    float rightSpeed;

    constexpr SpeedPair(float _leftSpeed = 0.0f, float _rightSpeed = 0.0f) noexcept
        : leftSpeed(_leftSpeed), rightSpeed(_rightSpeed) {}

    // Vec2 <op> Vec2
    constexpr inline SpeedPair operator+(const SpeedPair& a) const noexcept { return {leftSpeed + a.leftSpeed, rightSpeed + a.rightSpeed}; }
    constexpr inline SpeedPair operator-(const SpeedPair& a) const noexcept { return {leftSpeed - a.leftSpeed, rightSpeed - a.rightSpeed}; }
    constexpr inline SpeedPair operator*(const SpeedPair& a) const noexcept { return {leftSpeed * a.leftSpeed, rightSpeed * a.rightSpeed}; }
    constexpr inline SpeedPair operator/(const SpeedPair& a) const noexcept { return {leftSpeed / a.leftSpeed, rightSpeed / a.rightSpeed}; }

    // Vec2 <op> float
    constexpr inline SpeedPair operator*(const float& a) const noexcept { return {rightSpeed * a, leftSpeed * a}; }
    constexpr inline SpeedPair operator/(const float& a) const noexcept { return {rightSpeed / a, leftSpeed / a}; }

    // Compound assignments for Vec2
    constexpr inline SpeedPair& operator+=(const SpeedPair& a) noexcept { leftSpeed += a.leftSpeed; rightSpeed += a.rightSpeed; return *this; }
    constexpr inline SpeedPair& operator-=(const SpeedPair& a) noexcept { leftSpeed -= a.leftSpeed; rightSpeed -= a.rightSpeed; return *this; }
    constexpr inline SpeedPair& operator*=(const SpeedPair& a) noexcept { leftSpeed *= a.leftSpeed; rightSpeed *= a.rightSpeed; return *this; }
    constexpr inline SpeedPair& operator/=(const SpeedPair& a) noexcept { leftSpeed /= a.leftSpeed; rightSpeed /= a.rightSpeed; return *this; }

    constexpr inline SpeedPair& operator*=(const float& a) noexcept { leftSpeed *= a; rightSpeed *= a; return *this; }
    constexpr inline SpeedPair& operator/=(const float& a) noexcept { leftSpeed /= a; rightSpeed /= a; return *this; }
};

//| MARK:  Classes
class UnitInterval {
private:
    float value;
    static float clamp(float v) {
        if (v < 0.0f) return 0.0f;
        if (v > 1.0f) return 1.0f;
        return v;
    }
public:
    // Constructors
    UnitInterval(float v = 0.0f) : value(clamp(v)) {}
    // Implicit conversion to float
    operator float() const { return value; }
    operator double() const { return value; }
    // Assignment from float
    inline UnitInterval& operator=(float v) {
        value = clamp(v);
        return *this;
    }
    inline UnitInterval& operator=(double v) {
        value = clamp(v);
        return *this;
    }
    // Arithmetic with UnitInterval
    inline UnitInterval operator+(const UnitInterval& other) const { return UnitInterval(clamp(value + other.value)); }
    inline UnitInterval operator-(const UnitInterval& other) const { return UnitInterval(clamp(value - other.value)); }
    inline UnitInterval operator*(const UnitInterval& other) const { return UnitInterval(clamp(value * other.value)); }
    inline UnitInterval operator/(const UnitInterval& other) const { return UnitInterval(clamp(value / other.value)); }

    // Arithmetic with floats
    inline UnitInterval operator+(float f) const { return UnitInterval(clamp(value + f)); }
    inline UnitInterval operator-(float f) const { return UnitInterval(clamp(value - f)); }
    inline UnitInterval operator*(float f) const { return UnitInterval(clamp(value * f)); }
    inline UnitInterval operator/(float f) const { return UnitInterval(clamp(value / f)); }

    // Arithmetic with floats
    inline UnitInterval operator+(double f) const { return UnitInterval(clamp(value + f)); }
    inline UnitInterval operator-(double f) const { return UnitInterval(clamp(value - f)); }
    inline UnitInterval operator*(double f) const { return UnitInterval(clamp(value * f)); }
    inline UnitInterval operator/(double f) const { return UnitInterval(clamp(value / f)); }

    // Optional getter
    float get() const { return value; }
};
inline UnitInterval operator+(float f, const UnitInterval& u) { return UnitInterval(f + float(u)); }
inline UnitInterval operator-(float f, const UnitInterval& u) { return UnitInterval(f - float(u)); }
inline UnitInterval operator*(float f, const UnitInterval& u) { return UnitInterval(f * float(u)); }
inline UnitInterval operator/(float f, const UnitInterval& u) { return UnitInterval(f / float(u)); }


//| MARK: Weighted
struct W_Vec2 {
    float x;
    float y;
    float weight;

    constexpr W_Vec2(float _x = 0.0f, float _y = 0.0f, float _weight = 0.0f) noexcept
        : x(_x), y(_y), weight(_weight) {}

    // Member compound assignments (W_Vec2 <op>= Vec2 or W_Vec2)
    constexpr inline W_Vec2& operator+=(const Vec2& a) noexcept { x += a.x; y += a.y; return *this; }
    constexpr inline W_Vec2& operator-=(const Vec2& a) noexcept { x -= a.x; y -= a.y; return *this; }
    constexpr inline W_Vec2& operator*=(const Vec2& a) noexcept { x *= a.x; y *= a.y; return *this; }
    constexpr inline W_Vec2& operator/=(const Vec2& a) noexcept { x /= a.x; y /= a.y; return *this; }

    constexpr inline W_Vec2& operator+=(const W_Vec2& a) noexcept { x += a.x; y += a.y; return *this; }
    constexpr inline W_Vec2& operator-=(const W_Vec2& a) noexcept { x -= a.x; y -= a.y; return *this; }
    constexpr inline W_Vec2& operator*=(const W_Vec2& a) noexcept { x *= a.x; y *= a.y; return *this; }
    constexpr inline W_Vec2& operator/=(const W_Vec2& a) noexcept { x /= a.x; y /= a.y; return *this; }

    // Assign from Vec2
    constexpr inline W_Vec2& operator=(const Vec2& a) noexcept { x = a.x; y = a.y; return *this; }
    constexpr inline W_Vec2& operator=(const W_Vec2& a) noexcept { x = a.x; y = a.y; return *this; }
    constexpr inline operator Vec2() const noexcept { return {x, y}; }
};
constexpr inline Vec2 operator+(const Vec2& a, const W_Vec2& b) noexcept { return {a.x + b.x, a.y + b.y}; }
constexpr inline Vec2 operator-(const Vec2& a, const W_Vec2& b) noexcept { return {a.x - b.x, a.y - b.y}; }
constexpr inline Vec2 operator*(const Vec2& a, const W_Vec2& b) noexcept { return {a.x * b.x, a.y * b.y}; }
constexpr inline Vec2 operator/(const Vec2& a, const W_Vec2& b) noexcept { return {a.x / b.x, a.y / b.y}; }

constexpr inline Vec2 operator+(const W_Vec2& a, const Vec2& b) noexcept { return {a.x + b.x, a.y + b.y}; }
constexpr inline Vec2 operator-(const W_Vec2& a, const Vec2& b) noexcept { return {a.x - b.x, a.y - b.y}; }
constexpr inline Vec2 operator*(const W_Vec2& a, const Vec2& b) noexcept { return {a.x * b.x, a.y * b.y}; }
constexpr inline Vec2 operator/(const W_Vec2& a, const Vec2& b) noexcept { return {a.x / b.x, a.y / b.y}; }

// Symmetric non-member float operators
constexpr inline Vec2 operator*(const float& a, const Vec2& b) noexcept { return {b.x * a, b.y * a}; }
constexpr inline Vec2 operator/(const float& a, const Vec2& b) noexcept { return {b.x / a, b.y / a}; }
// constexpr inline Vec2 operator=(Vec2& a, const W_Vec2& b) noexcept { a.x = b.x; a.y = b.y; return a; }


struct W_int {
    int val;
    float weight;
    constexpr W_int(int _val=0, float _weight=0) noexcept
        : val(_val), weight(_weight) {};

    constexpr inline W_int operator*(const int& a) const noexcept { return { val * a, weight }; }
    constexpr inline W_int operator+(const int& a) const noexcept { return { val + a, weight }; }
    constexpr inline W_int operator-(const int& a) const noexcept { return { val - a, weight }; }
    constexpr inline W_int operator/(const int& a) const noexcept { return { val / a, weight }; }

    // Member operators (W_float <op> W_float)
    constexpr inline W_int operator*(const W_int& a) const noexcept { return { val * a.val, weight }; }
    constexpr inline W_int operator+(const W_int& a) const noexcept { return { val + a.val, weight }; }
    constexpr inline W_int operator-(const W_int& a) const noexcept { return { val - a.val, weight }; }
    constexpr inline W_int operator/(const W_int& a) const noexcept { return { val / a.val, weight }; }

    // Compound assignment (ignore weight)
    constexpr inline W_int& operator*=(const int& a) noexcept { val *= a; return *this; }
    constexpr inline W_int& operator+=(const int& a) noexcept { val += a; return *this; }
    constexpr inline W_int& operator-=(const int& a) noexcept { val -= a; return *this; }
    constexpr inline W_int& operator/=(const int& a) noexcept { val /= a; return *this; }

    constexpr inline W_int& operator*=(const W_int& a) noexcept { val *= a.val; return *this; }
    constexpr inline W_int& operator+=(const W_int& a) noexcept { val += a.val; return *this; }
    constexpr inline W_int& operator-=(const W_int& a) noexcept { val -= a.val; return *this; }
    constexpr inline W_int& operator/=(const W_int& a) noexcept { val /= a.val; return *this; }

    // Assignment (ignore weight)
    constexpr inline W_int& operator=(const float& a) noexcept { val = a; return *this; }
    constexpr inline W_int& operator=(const W_int& a) noexcept { val = a.val; return *this; }
};

struct W_float {
    float val;
    float weight;

    constexpr W_float(float _val = 0.0f, float _weight = 0.0f) noexcept
        : val(_val), weight(_weight) {}

    // Member operators (W_float <op> float)
    constexpr inline W_float operator*(const float& a) const noexcept { return { val * a, weight }; }
    constexpr inline W_float operator+(const float& a) const noexcept { return { val + a, weight }; }
    constexpr inline W_float operator-(const float& a) const noexcept { return { val - a, weight }; }
    constexpr inline W_float operator/(const float& a) const noexcept { return { val / a, weight }; }

    // Member operators (W_float <op> W_float)
    constexpr inline W_float operator*(const W_float& a) const noexcept { return { val * a.val, weight }; }
    constexpr inline W_float operator+(const W_float& a) const noexcept { return { val + a.val, weight }; }
    constexpr inline W_float operator-(const W_float& a) const noexcept { return { val - a.val, weight }; }
    constexpr inline W_float operator/(const W_float& a) const noexcept { return { val / a.val, weight }; }

    // Compound assignment (ignore weight)
    constexpr inline W_float& operator*=(const float& a) noexcept { val *= a; return *this; }
    constexpr inline W_float& operator+=(const float& a) noexcept { val += a; return *this; }
    constexpr inline W_float& operator-=(const float& a) noexcept { val -= a; return *this; }
    constexpr inline W_float& operator/=(const float& a) noexcept { val /= a; return *this; }

    constexpr inline W_float& operator*=(const W_float& a) noexcept { val *= a.val; return *this; }
    constexpr inline W_float& operator+=(const W_float& a) noexcept { val += a.val; return *this; }
    constexpr inline W_float& operator-=(const W_float& a) noexcept { val -= a.val; return *this; }
    constexpr inline W_float& operator/=(const W_float& a) noexcept { val /= a.val; return *this; }

    // Assignment (ignore weight)
    constexpr inline W_float& operator=(const float& a) noexcept { val = a; return *this; }
    constexpr inline W_float& operator=(const W_float& a) noexcept { val = a.val; return *this; }
};

// Non-member symmetric overloads (float <op> W_float)
constexpr inline W_float operator*(float a, const W_float& b) noexcept { return { a * b.val, b.weight }; }
constexpr inline W_float operator+(float a, const W_float& b) noexcept { return { a + b.val, b.weight }; }
constexpr inline W_float operator-(float a, const W_float& b) noexcept { return { a - b.val, b.weight }; }
constexpr inline W_float operator/(float a, const W_float& b) noexcept { return { a / b.val, b.weight }; }



//| MARK: Utilities

// Returns degrees -> radians
inline double deg2rad(float degrees) {
	return degrees * (PI / 180);
}

// Returns radians -> degrees
inline double rad2deg(double radians) {
	return radians * (180 / PI);
}

// Truncates a value at cutoff decimal places. (Default = 2)
inline double truncate(double num, int cutoff = 2) {
	return std::floor(num * std::pow(10, cutoff)) / std::pow(10, cutoff);
}

// Returns the sign of the input
inline int sign(float input) {
	return (input >= 0) ? 1 : -1;
}

// Maps a value between a range
inline double map_value(double input, double input_start, double input_end, double output_start, double output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

// Returns the degrees between two positions
inline float degreesTill(const Vec2& from, const Vec2& to) {
	return rad2deg(std::atan2(to.y - from.y, to.x - from.x));
}

// Returns the radians between two positions
inline float radiansTill(const Vec2& from, const Vec2& to) {
	return std::atan2(to.y - from.y, to.x - from.x);
}

// Returns in the same length unit that is stored in each Vec2 (mismatch leads to mathematically incorrect results)
inline double distanceBetween(const Vec2& pos1, const Vec2& pos2) {
    return std::sqrt((std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2)));
}

// Returns dot product of two vectors
inline float dot(const Vec2& vec1, const Vec2& vec2) {
    return (vec1.x * vec2.x) + (vec1.y * vec2.y);
}


//| MARK: QOL Funcs 
template <typename T>
void printOnScreen(const T& input, int row = 0) {
    std::string printtext;
    if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char*>) {
        // Handle string and C-style string types
        printtext = input;
    } else {
        // Handle other types using stringstream
        std::stringstream ss;
        ss << input;
        printtext = ss.str();
    }
    pros::lcd::set_text(row, printtext);
}