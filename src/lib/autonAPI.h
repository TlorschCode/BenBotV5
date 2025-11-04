#pragma once
#include "globalAPI.h"
namespace robotAPI {
    class Robot;
} // namespace robotAPI

namespace autonAPI {
struct Point {
	Vec2 pos = {0, 0};
	Point(float _x, float _y) {
		pos.x = _x;
		pos.y = _y;
	}
	Point(const Vec2 &v) : pos(v) {};
	bool visited = false;
	inline Point& operator=(const Vec2 &other) {
		pos = other;
		return *this;
	}
};

class PID_Controller {
	private:
		robotAPI::Robot *robot = nullptr;
		W_Vec2 pPos = {0, 0, 6.0f};
		W_Vec2 iPos = {0, 0, 0.05f};
		W_Vec2 dPos = {0, 0, 4.2f};
		Vec2 prev_pPos = {0, 0};

		W_float pRot = {0, 1.0f};
		W_float iRot = {0, 0.05f};
		W_float dRot = {0, 0.85f};
		float prev_pRot = 0;

		float prev_rot = 0;
		float original_x; // FIXME: orignal_x, _y, and _rot are incompletely implemented.
		float original_y;
	public:
		float linearSpeed;
		float rotSpeed;
		PID_Controller() {};
		void update(Vec2 &_target);
        Vec2 getPurePursuitLoc(float &checkRadius, Vec2 &target, Vec2 &prevTarget);
};
} // namespace autonAPI;

#include "robotAPI.h"
namespace autonAPI {
inline void PID_Controller::update(Vec2 &_target) {
    // robotAPI::Robot &dereffedRobot = *robot;
	// float PID_pos;
	// float PID_rot;
    // const float targetRot = degreesTill(dereffedRobot.pos, _target);
    // const bool doRotCalcs = (targetRot - dereffedRobot.heading.getDegrees()) > 10; // flag if heading is close enough to target // FIXME: Not set correctly
    // const double rot_radians = dereffedRobot.heading.getRadians();

    // pPos = {(_target.x - dereffedRobot.pos.x) * pPos.weight, (_target.y - dereffedRobot.pos.y) * pPos.weight};
    // iPos += Vec2{(iPos.x + pPos.x) * sin(rot_radians), (iPos.y + pPos.y) * cos(rot_radians)};
    // dPos = (prev_pPos - pPos) * dPos.weight;
	// prev_pPos = pPos;

    // const Vec2 PID_pos = {(pPos.x + (iPos.x * iPos.weight) + (dPos.x * dPos.weight)) * sin(rot_radians), (pPos.y + (iPos.y * iPos.weight) + (dPos.y * dPos.weight)) * cos(rot_radians)};
    // //| rotation
	// if (doRotCalcs) {
    // 	PID_rot = ((pRot + (iRot * iRot.weight) + (dRot * dRot.weight))).val;
	// } else {
	// 	PID_rot = 0;
	// }
	// pRot = (targetRot - pRot) * pRot.weight;
	// iRot += pRot * iRot.weight;
	// dRot = (prev_pRot - dereffedRobot.heading.getDegrees()) * dRot.weight;
	// prev_pRot = pRot.val;
	// // FIXME: set linear and rot speed
}

inline Vec2 PID_Controller::getPurePursuitLoc(float &checkRadius, Vec2 &target, Vec2 &prevTarget) {
    robotAPI::Robot &dereffedRobot = *robot;
	Vec2 minTarget = Vec2(std::min(target.x, prevTarget.x), std::min(target.y, prevTarget.y));
	Vec2 maxTarget = Vec2(std::max(target.x, prevTarget.x), std::max(target.y, prevTarget.y));

	// FIXME: CHECK MATH vvv
	float dotProduct = std::pow(target.x - prevTarget.x, 2) + std::pow(target.y - prevTarget.y, 2);
	float twoceCircleOffset = 2 * (((prevTarget.x - dereffedRobot.pos.x) * (target.x - prevTarget.x)) + ((prevTarget.y - dereffedRobot.pos.y) * (target.y - prevTarget.y)));
	float prevTarCurPosDist = (std::pow(prevTarget.x - dereffedRobot.pos.x, 2) + std::pow(prevTarget.y - dereffedRobot.pos.y, 2)) - std::pow(checkRadius, 2);
	float discriminant = std::pow(twoceCircleOffset, 2) - (4 * dotProduct * prevTarCurPosDist);
	float intersectRatio1 = (-twoceCircleOffset + sqrt(discriminant)) / (2 * dotProduct);
	float intersectRatio2 = (-twoceCircleOffset - sqrt(discriminant)) / (2 * dotProduct);
	// FIXME: CHECK MATH ^^^

	Vec2 intercept1 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio1, prevTarget.y + (target.y - prevTarget.y) * intersectRatio1};
	Vec2 intercept2 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio2, prevTarget.y + (target.y - prevTarget.y) * intersectRatio2};

	bool within_x = (minTarget.x <= intercept1.x <= maxTarget.x) || (minTarget.x <= intercept2.y <= maxTarget.x);
	bool within_y = (minTarget.y <= intercept1.y <= maxTarget.y) || (minTarget.y <= intercept2.y <= maxTarget.y);
	if (discriminant >= 0) {
		if (within_x && within_y) {
			if (abs(intercept2.x - target.x) + abs(intercept2.y - target.y) < abs(intercept1.x - target.x) + abs(intercept1.y - target.y)) {
				return intercept2;
			} else {
				return prevTarget;
			}
		}
	}
}
} // namespace autonAPI;