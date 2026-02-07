#pragma once
#include "common.h"
#include <array>

namespace robotAPI { class Robot; }

struct Point {
    Vec2 pos = {0, 0};
    bool visited = false;

    Point(float _x, float _y) : pos{_x, _y} {}
    Point(const Vec2& v) : pos(v) {}

    inline Point& operator=(const Vec2& other) {
        pos = other;
        return *this;
    }
};

namespace autonAPI {
    class AutonController {
    private:
        robotAPI::Robot* robot = nullptr;
        W_float pPos = {0, 0.60f}; // vvv magic numbers found through testing vvv
        W_float iPos = {0, 0.005f};
        W_float dPos = {0, 0.42f};

        W_float pRot = {0, 0.10f};
        W_float iRot = {0, 0.005f};
        W_float dRot = {0, 0.085f};
        float prev_allWheelRot = 0;
    public:
        AutonController() = default;

        SpeedPair getPID_speedTo(Vec2& _target);
        float updateHeadingAndOdom();
        Vec2 getPurePursuitLoc(const float& checkRadius, const Vec2& target);
        inline void _bindRobot(robotAPI::Robot* r) { robot = r; }
    };
}
