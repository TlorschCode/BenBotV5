#pragma once
#include "globalAPI.h"
namespace robotAPI {
    class Robot;
} // namespace robotAPI

namespace autonAPI {
struct Point {
	Vector2 pos = {0, 0};
	Point(float _x, float _y) {
		pos.x = _x;
		pos.y = _y;
	}
	Point(const Vector2 &v) : pos(v) {};
	bool visited = false;
	inline Point& operator=(const Vector2 &other) {
		pos = other;
		return *this;
	}
};
constexpr float pPosWeight = {6.0f};
constexpr float iPosWeight = {0.05f};
constexpr float dPosWeight = {4.2f};
constexpr float pRotWeight = {1.0f};
constexpr float iRotWeight = {0.05f};
constexpr float dRotWeight = {0.85f};
class PID_Controller {
	private:
		robotAPI::Robot *robot = nullptr;
		Vector2 pPos = {0, 0};
		Vector2 iPos = {0, 0};
		Vector2 dPos = {0, 0};
		float pRot = 0;
		float iRot = 0;
		float dRot = 0;
		float PID_rot = 0;
		float prev_rot = 0;
	public:
		float linearSpeed;
		float rotSpeed;
		PID_Controller() {};
		void update(Vector2 &_target);
        Vector2 getPurePursuitLoc(float &checkRadius, Vector2 &target, Vector2 &prevTarget);
			
};
} // namespace autonAPI;

#include "robotAPI.h"
namespace autonAPI {
inline void PID_Controller::update(Vector2 &_target) {
    robotAPI::Robot &dereffedRobot = *robot;
    float original_x = 0;
    float original_y = 0;
    float original_rot = 0;
    bool auton_rot = true; // flag if heading is close enough to target
    double rot_radians = dereffedRobot.heading.getRadians();
    pPos = {(_target.x - dereffedRobot.pos.x) * pPosWeight, (_target.y - dereffedRobot.pos.y) * pPosWeight};
    iPos = {(iPos.x + pPos.x) * cos(rot_radians), (iPos.y + pPos.y) * cos(rot_radians)};
    dPos = {original_x - (dereffedRobot.pos.x * dPosWeight), original_y - ((*robot).pos.y * dPosWeight)};
    Vector2 PID = {(pPos.x + (iPos.x * iPosWeight) + (dPos.x * dPosWeight)) * sin(rot_radians), (pPos.y + (iPos.y * iPosWeight) + (dPos.y * dPosWeight)) * cos(rot_radians)};
    //| rotation -
    float target_rotation = degreesTill(dereffedRobot.pos, _target);
    pRot = (target_rotation - pRot) * pRotWeight;
    iRot += pRot;
    // FIXME:
    dRot = original_rot - dereffedRobot.heading.getDegrees();
    PID_rot = ((pRot + (iRot * iRotWeight) + (dRot * dRotWeight)) * auton_rot);
}

inline Vector2 PID_Controller::getPurePursuitLoc(float &checkRadius, Vector2 &target, Vector2 &prevTarget) {
    robotAPI::Robot &dereffedRobot = *robot;
	Vector2 minTarget = Vector2(std::min(target.x, prevTarget.x), std::min(target.y, prevTarget.y));
	Vector2 maxTarget = Vector2(std::max(target.x, prevTarget.x), std::max(target.y, prevTarget.y));

	// FIXME: CHECK MATH vvv
	float dotProduct = std::pow(target.x - prevTarget.x, 2) + std::pow(target.y - prevTarget.y, 2);
	float twoceCircleOffset = 2 * (((prevTarget.x - dereffedRobot.pos.x) * (target.x - prevTarget.x)) + ((prevTarget.y - dereffedRobot.pos.y) * (target.y - prevTarget.y)));
	float prevTarCurPosDist = (std::pow(prevTarget.x - dereffedRobot.pos.x, 2) + std::pow(prevTarget.y - dereffedRobot.pos.y, 2)) - std::pow(checkRadius, 2);
	float discriminant = std::pow(twoceCircleOffset, 2) - (4 * dotProduct * prevTarCurPosDist);
	float intersectRatio1 = (-twoceCircleOffset + sqrt(discriminant)) / (2 * dotProduct);
	float intersectRatio2 = (-twoceCircleOffset - sqrt(discriminant)) / (2 * dotProduct);
	// FIXME: CHECK MATH ^^^

	Vector2 intercept1 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio1, prevTarget.y + (target.y - prevTarget.y) * intersectRatio1};
	Vector2 intercept2 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio2, prevTarget.y + (target.y - prevTarget.y) * intersectRatio2};

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