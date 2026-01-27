#include "autonAPI.h"
#include "robotAPI.h"
#include <cmath>
#include <algorithm>
#include <vector>




namespace autonAPI {

// Returns the left and right speed a robot's drivetrain should adopt in order to go towards a point using PID.
// (Uses dot product to control rotation PID)
SpeedPair PID_Controller::getSpeedFromPID_to(Vec2& target) {
    //| Statics
    static float prevPosError = 0;
    static float prevRotError = 0;
    static float prev_pPos = 0;
    static float prev_pRot = 0;
    

    //| Setup
    const float targetRot = degreesTill(robot->pos, target);
    const Vec2 targRotVec = deg2vec(targetRot); // The direction as a vec, using sin and cos to convert it to x and y (unit circle style)
    const Vec2 headingVec = deg2vec(robot->heading_deg);

    // TODO: Set up dot product and cross product math to avoid turning the wrong way if the values are like: targ = 30 & cur=270
    //| Rotation
    const float curRotError = rad2deg(std::atan2(crossProd(headingVec, target), dotProd(robot->pos, target))); // from -pi to pi
    // PID calc
    pRot = (targetRot - pRot) * pRot.weight;
    iRot += pRot * iRot.weight;
    dRot = dRot.weight * (curRotError - prevRotError);
    prev_pRot = pRot.val;
    const float PID_rot = pRot.val;//(pRot + (iRot * iRot.weight) + (dRot * dRot.weight)).val;


    //| Position
    const float curPosError = distanceBetween(robot->pos, target); 
    // PID calc
    pPos = curPosError * pPos.weight;
    iPos += pPos;
    dPos = dPos.weight * (curPosError - prevPosError);
    prev_pPos = pPos.val;
    const float PID_pos = pPos.val;//(pPos + (iPos * iPos.weight) + (dPos * dPos.weight)).val;

    //| Cleanup
    prevRotError = curRotError;
    prevPosError = curPosError;

    //| Result
    if (curRotError < 0) {}
    return SpeedPair{
        PID_pos - PID_rot,
        PID_pos + PID_rot
    };
}


Vec2 PID_Controller::getPurePursuitLoc(const float &checkRadius, const Vec2 &target, const Vec2 &prevTarget) {
    Vec2 minTarget = {std::min(target.x, prevTarget.x), std::min(target.y, prevTarget.y)};
    Vec2 maxTarget = {std::max(target.x, prevTarget.x), std::max(target.y, prevTarget.y)};

    float dotProduct = std::pow(target.x - prevTarget.x, 2) + std::pow(target.y - prevTarget.y, 2);
    float twoceCircleOffset = 2 * (((prevTarget.x - robot->pos.x) * (target.x - prevTarget.x)) + ((prevTarget.y - robot->pos.y) * (target.y - prevTarget.y)));
    float prevTarCurPosDist = (std::pow(prevTarget.x - robot->pos.x, 2) + std::pow(prevTarget.y - robot->pos.y, 2)) - std::pow(checkRadius, 2);
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


float PID_Controller::updateHeadingAndOdom() {
    uint32_t now = pros::millis();

    robot->heading_deg = truncate(robot->inertial.get_rotation()); // Cutoff at 2 decimal places because inertial sensor is innacurate

    float leftMotorsPos = robot->drivetrain.getLeftMotorsPos();
    float rightMotorsPos = robot->drivetrain.getRightMotorsPos();

    float averageWheelRot = (leftMotorsPos + rightMotorsPos) / 2;
    float wheelRotDelta = averageWheelRot - prev_allWheelRot;
    prev_allWheelRot = averageWheelRot;

    robot->pos.x += ((wheelRotDelta / 360.0f) * robot->drivetrain.GEAR_RATIO * robot->drivetrain.WHEEL_CIRCUMFERENCE) * sin(deg2rad(robot->heading_deg));
    robot->pos.y += ((wheelRotDelta / 360.0f) * robot->drivetrain.GEAR_RATIO * robot->drivetrain.WHEEL_CIRCUMFERENCE) * cos(deg2rad(robot->heading_deg));
    return (wheelRotDelta / 360.0f); // Return this for debugging purposes
}

} // namespace autonAPI
