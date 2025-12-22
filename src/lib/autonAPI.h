#pragma once
#include "globalAPI.h"

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
    class PID_Controller {
    private:
        robotAPI::Robot* robot = nullptr;
        W_Vec2 pPos = {0, 0, 6.0f};
        W_Vec2 iPos = {0, 0, 0.05f};
        W_Vec2 dPos = {0, 0, 4.2f};
        Vec2 prev_pPos = {0, 0};

        W_float pRot = {0, 1.0f};
        W_float iRot = {0, 0.05f};
        W_float dRot = {0, 0.85f};
        float prev_pRot = 0;
        float prev_rot = 0;
        float prev_allWheelRot = 0;

    public:
        float leftSpeed = 0;
        float rightSpeed = 0;

        PID_Controller() = default;

        void updatePID(Vec2& _target);
        float updateOdom();
        Vec2 getPurePursuitLoc(const float& checkRadius, const Vec2& target, const Vec2& prevTarget);
        inline void bindRobot(robotAPI::Robot* r) { robot = r; }
    };
}
