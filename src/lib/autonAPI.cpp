#include "autonAPI.h"
#include "robotAPI.h"
#include <cmath>
#include <algorithm>


namespace autonAPI {

void PID_Controller::updatePID(Vec2 &_target) {
    robotAPI::Robot &dereffedRobot = *robot;
    float PID_rot;
    const float targetRot = degreesTill(dereffedRobot.pos, _target);
    const bool doRotCalcs = (targetRot - dereffedRobot.heading.getDegrees()) > 10;
    const double rot_radians = dereffedRobot.heading.getRadians();

    pPos = Vec2{(_target.x - dereffedRobot.pos.x) * pPos.weight, (_target.y - dereffedRobot.pos.y) * pPos.weight};
    iPos += Vec2{(iPos.x + pPos.x) * sin(rot_radians), (iPos.y + pPos.y) * cos(rot_radians)};
    dPos = (prev_pPos - pPos) * dPos.weight;
    prev_pPos = pPos;

    const Vec2 PID_pos = {
        (pPos.x + (iPos.x * iPos.weight) + (dPos.x * dPos.weight)) * sin(rot_radians),
        (pPos.y + (iPos.y * iPos.weight) + (dPos.y * dPos.weight)) * cos(rot_radians)
    };

    if (doRotCalcs) {
        pRot = (targetRot - pRot) * pRot.weight;
        iRot += pRot * iRot.weight;
        dRot = (prev_pRot - dereffedRobot.heading.getDegrees()) * dRot.weight;
        PID_rot = (pRot + (iRot * iRot.weight) + (dRot * dRot.weight)).val;
    } else {
        PID_rot = 0;
    }
    prev_pRot = pRot.val;

    leftSpeed = PID_pos.x + PID_pos.y;
    rightSpeed = PID_pos.x - PID_pos.y;
}

Vec2 PID_Controller::getPurePursuitLoc(const float &checkRadius, const Vec2 &target, const Vec2 &prevTarget) {
    robotAPI::Robot &dereffedRobot = *robot;
    Vec2 minTarget = {std::min(target.x, prevTarget.x), std::min(target.y, prevTarget.y)};
    Vec2 maxTarget = {std::max(target.x, prevTarget.x), std::max(target.y, prevTarget.y)};

    float dotProduct = std::pow(target.x - prevTarget.x, 2) + std::pow(target.y - prevTarget.y, 2);
    float twoceCircleOffset = 2 * (((prevTarget.x - dereffedRobot.pos.x) * (target.x - prevTarget.x)) + ((prevTarget.y - dereffedRobot.pos.y) * (target.y - prevTarget.y)));
    float prevTarCurPosDist = (std::pow(prevTarget.x - dereffedRobot.pos.x, 2) + std::pow(prevTarget.y - dereffedRobot.pos.y, 2)) - std::pow(checkRadius, 2);
    float discriminant = std::pow(twoceCircleOffset, 2) - (4 * dotProduct * prevTarCurPosDist);
    

    if (discriminant >= 0) {
        const float intersectRatio1 = (-twoceCircleOffset + sqrt(discriminant)) / (2 * dotProduct);
        const float intersectRatio2 = (-twoceCircleOffset - sqrt(discriminant)) / (2 * dotProduct);

        const Vec2 intercept1 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio1, prevTarget.y + (target.y - prevTarget.y) * intersectRatio1};
        const Vec2 intercept2 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio2, prevTarget.y + (target.y - prevTarget.y) * intersectRatio2};

        const bool within_x = (minTarget.x <= intercept1.x && intercept1.x <= maxTarget.x) || (minTarget.x <= intercept2.x && intercept2.x <= maxTarget.x);
        const bool within_y = (minTarget.y <= intercept1.y && intercept1.y <= maxTarget.y) || (minTarget.y <= intercept2.y && intercept2.y <= maxTarget.y);
        if (within_x && within_y && std::abs(intercept2.x - target.x) + std::abs(intercept2.y - target.y) <
            std::abs(intercept1.x - target.x) + std::abs(intercept1.y - target.y)) {
            return intercept2;
        } else {
            return prevTarget;
        }
    }
    return target; // Fallback
}

float PID_Controller::updateOdom() {
    robotAPI::Robot &dereffedRobot = *robot;
    uint32_t now = pros::millis();

    dereffedRobot.heading.setDegrees(truncate(dereffedRobot.inertial.get_rotation()));

    float leftMotorsPos = (dereffedRobot.drivetrain.topLeft.rawMotor.get_raw_position(&now) +
                           dereffedRobot.drivetrain.middleLeft.rawMotor.get_raw_position(&now) + 
                           dereffedRobot.drivetrain.bottomLeft.rawMotor.get_raw_position(&now)
                        ) / 3;
    float rightMotorsPos = (dereffedRobot.drivetrain.topRight.rawMotor.get_raw_position(&now) +
                            dereffedRobot.drivetrain.middleRight.rawMotor.get_raw_position(&now) + 
                            dereffedRobot.drivetrain.bottomRight.rawMotor.get_raw_position(&now)
                        ) / 3;

    float averageWheelRot = (leftMotorsPos + rightMotorsPos) / 2;
    float wheelRotDelta = averageWheelRot - prev_allWheelRot;

    prev_allWheelRot = averageWheelRot;
    dereffedRobot.pos.x += ((wheelRotDelta / 360.0f) * GEAR_RATIO * WHEEL_CIRCUMFERENCE) * sin(dereffedRobot.heading.getRadians());
    dereffedRobot.pos.y += ((wheelRotDelta / 360.0f) * GEAR_RATIO * WHEEL_CIRCUMFERENCE) * cos(dereffedRobot.heading.getRadians());
    return (wheelRotDelta / 360.0f);
}

} // namespace autonAPI
