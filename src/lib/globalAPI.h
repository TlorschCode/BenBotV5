#pragma once
#include "main.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;
constexpr float GEAR_RATIO = {0.5f}; // Wheel-motor gear ratio
constexpr float WHEEL_CIRCUMFERENCE = {12.56f}; //  inches
constexpr float FRAME = {10}; // Frame time

template <typename T>
void println(const T& input, int row = 1) {
    string printtext;
    if constexpr (is_same_v<T, string> || is_same_v<T, const char*>) {
        // Handle string and C-style string types
        printtext = input;
    } else {
        // Handle other types using stringstream
        stringstream ss;
        ss << input;
        printtext = ss.str();
    }
    pros::lcd::set_text(row, printtext);
}

struct Vector2 {
	float x;
	float y;
	Vector2(float _x, float _y) : x(_x), y(_y) {}
	Vector2() : x(0), y(0) {}

    // Example: Overload the addition operator
    inline Vector2 operator+(const Vector2 &other) const {
        return Vector2(x + other.x, y + other.y);
    }
	inline Vector2 operator-(const Vector2 &other) const {
        return Vector2(x - other.x, y - other.y);
    }
	inline Vector2 operator/(const Vector2 &other) const {
        return Vector2(x / other.x, y / other.y);
    }
	inline Vector2 operator*(const Vector2 &other) const {
        return Vector2(x * other.x, y * other.y);
    }
};

// MARK: Utilities
inline double toRadians(float degrees) {
	return degrees * (PI / 180);
}

inline double toDegrees(double radians) {
	return radians * (180 / PI);
}

inline double truncate(double num, int cutoff = 2) {
	return std::floor(num * std::pow(10, cutoff)) / std::pow(10, cutoff);
}

inline int sign(float &input) {
	return (input >= 0) ? 1 : -1;
}

inline double map_value(double &input, double &input_start, double &input_end, double &output_start, double &output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

inline float degreesTill(Vector2 &from, Vector2 &to) {
	return toDegrees(std::atan2(to.y - from.y, to.x - from.x));
}

inline float radiansTill(Vector2 &from, Vector2 &to) {
	return std::atan2(to.y - from.y, to.x - from.x);
}