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

namespace autonAPI {
    class AutonController {
    private:
        robotAPI::Robot* robot = nullptr;
        W_float pPos = {0, 0.60f}; // vvv magic numbers found through testing vvv
        W_float iPos = {0, 0.05f};
        W_float dPos = {0, 0.42f};

        W_float pRot = {0, 0.10f};
        W_float iRot = {0, 0.01f};
        W_float dRot = {0, 0.095f};
        float prev_allWheelRot = {0};
        NODISCARD std::variant<Vec2, std::nullptr_t> getIntersect(const Vec2 &target, const Vec2 lastValidLookaheadPoint);
    public:
        float checkRadius = {10.0f};
        AutonController() = default;

        std::vector<Point> driveAlongPath(std::vector<Point> path);
        float updateHeadingAndOdom();
        NODISCARD SpeedPair getPID_speedTo(Vec2& _target);
        // Gets the intersect of the robot's check circle and the target/next target
        NODISCARD Vec2 getPurePursuitLoc(const Vec2& target, const Vec2& nextTarget, bool resetLastLookahead);
        inline void _bindRobot(robotAPI::Robot* r) { robot = r; }
    };
}
