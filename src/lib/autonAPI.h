/*###|   autonAPI.h   |###*/

#pragma once
#include "common.h"
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <vector>
#include <variant>
#include <array>

namespace robotAPI { class Robot; }

struct Point : Vec2 {
    bool visited = false;

    Point(float _x, float _y) : Vec2{_x, _y} {}
    Point(const Vec2& v) : Vec2{v} {}

    inline bool operator==(const Point& other) const noexcept { return (static_cast<const Vec2&>(other) == static_cast<const Vec2&>(*this) && (other.visited == visited)); }
    inline Point& operator=(const Vec2& other) noexcept {
        x = other.x;
        y = other.y;
        return *this;
    }
};

enum PID : uint8_t {
    PID_P = 1 << 0,
    PID_I = 1 << 1,
    PID_D = 1 << 2
};
inline constexpr PID operator|(PID a, PID b) noexcept { return static_cast<PID>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b)); }
inline constexpr PID operator&(PID a, PID b) noexcept { return static_cast<PID>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b)); }
inline constexpr PID operator^(PID a, PID b) noexcept { return static_cast<PID>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b)); }
inline constexpr PID operator~(PID a) noexcept { return static_cast<PID>(~static_cast<uint8_t>(a)); }
inline constexpr PID operator<<(PID a, int b) noexcept { return static_cast<PID>(static_cast<uint8_t>(a) << b); }
inline constexpr PID operator>>(PID a, int b) noexcept { return static_cast<PID>(static_cast<uint8_t>(a) >> b); }
inline constexpr PID& operator|=(PID& a, PID b) noexcept { return a = a | b; }
inline constexpr PID& operator&=(PID& a, PID b) noexcept { return a = a & b; }
inline constexpr PID& operator^=(PID& a, PID b) noexcept { return a = a ^ b; }
inline constexpr PID& operator<<=(PID& a, int b) noexcept { return a = a << b; }
inline constexpr PID& operator>>=(PID& a, int b) noexcept { return a = a >> b; }

namespace autonAPI {


class AutonController {
    private:
        robotAPI::Robot* robot = nullptr;
        float prev_allWheelRot = {0};
        NODISCARD std::variant<Vec2, std::nullptr_t> getIntersect(const Vec2 &target, const Vec2 lastValidLookaheadPoint);
        W_float pPos{};
        W_float iPos{};
        W_float dPos{};

        W_float pRot{};
        W_float iRot{};
        W_float dRot{};
    public:
        inline void setPIDposVal(PID selection, W_float val) {
            if (selection & PID_P) pPos = val;
            if (selection & PID_I) iPos = val;
            if (selection & PID_D) dPos = val;
        }
        inline void setPIDrotVal(PID selection, W_float val) {
            if (selection & PID_P) pRot = val;
            if (selection & PID_I) iRot = val;
            if (selection & PID_D) dRot = val;
        }
        inline void setPIDposVal(int selection, W_float val) {
            if (selection & PID_P) pPos = val;
            if (selection & PID_I) iPos = val;
            if (selection & PID_D) dPos = val;
        }
        inline void setPIDrotVal(int selection, W_float val) {
            if (selection & PID_P) pRot = val;
            if (selection & PID_I) iRot = val;
            if (selection & PID_D) dRot = val;
        }
        NODISCARD inline W_float getPIDposVal(PID selection) {
            if (selection & PID_P) return pPos;
            if (selection & PID_I) return iPos;
            if (selection & PID_D) return dPos;
        }
        NODISCARD inline W_float getPIDrotVal(PID selection) {
            if (selection & PID_P) return pRot;
            if (selection & PID_I) return iRot;
            if (selection & PID_D) return dRot;
        }
        NODISCARD inline W_float getPIDposVal(int selection) {
            if (selection & PID_P) return pPos;
            if (selection & PID_I) return iPos;
            if (selection & PID_D) return dPos;
        }
        NODISCARD inline W_float getPIDrotVal(int selection) {
            if (selection & PID_P) return pRot;
            if (selection & PID_I) return iRot;
            if (selection & PID_D) return dRot;
        }
        float checkRadius = {10.0f};
        AutonController() = default;

	    pros::Mutex mtx;

        std::vector<Point> driveAlongPath(std::vector<Point> path, pros::Mutex& _mtx, PID PIDvalsToIgnore);
        float updateHeadingAndOdom();
        NODISCARD SpeedPair getPID_speedTo(Vec2& _target, PID PIDvalsToIgnore);
        // Gets the intersect of the robot's check circle and the target/next target
        NODISCARD Vec2 getPurePursuitLoc(const Vec2& target, const Vec2& nextTarget, bool resetLastLookahead);
        inline void _bindRobot(robotAPI::Robot* r) { robot = r; }
    };
}
