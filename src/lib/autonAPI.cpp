#include "autonAPI.h"
#include "robotAPI.h"
#include <cmath>
#include <algorithm>
#include <vector>




namespace autonAPI {

// Returns the left and right speed a robot's drivetrain should adopt in order to go towards a point.
// - The left speed is the 'x' attribute and the right speed is the 'y' attribute of the returned Vec2
Vec2 PID_Controller::getSpeedPID_to(Vec2 &target) {
    static float prevPosError = 0.0f;
    static float prevRotError = 0.0f;
    static float prev_pPos = 0.0f;
    
    const float targetRot = degreesTill(robot->pos, target);
    const double robot_rotRadians = toRadians(robot->heading);

    const float curPosError = distanceBetween(robot->pos, target);
    const float curRotError = targetRot - robot->heading;

    float PID_rotScale;

    // Get PID for position
    pPos = pPos.weight * curPosError;
    iPos += pPos;
    dPos = dPos.weight * (curPosError - prevPosError);
    prev_pPos = pPos.val;

    // Construct the modified position (originally the target location)
    const float PID_pos = (pPos + (iPos * iPos.weight) + (dPos * dPos.weight)).val;

    // Calculate rotation PID if we aren't facing towards the target (with margin of 5 degrees)
    if ((curRotError) > 5) {
        // Calculate PID
        pRot = (targetRot - pRot) * pRot.weight;
        iRot += pRot * iRot.weight;
        dRot = dRot.weight * (curRotError - prevRotError);
        prev_pRot = pRot.val;
        PID_rotScale = (pRot + (iRot * iRot.weight) + (dRot * dRot.weight)).val; // A value to scale rotation weighting
        // Convert the direction we need to be facing into a position
    } else {
        PID_rotScale = 1;
    }
    
    prevRotError = curRotError;
    prevPosError = curPosError;
    // TODO: PID calculations are wonked
    return dRot.val;
    // return Vec2{
    //     finalSpeeds.x + finalSpeeds.y,
    //     finalSpeeds.x - finalSpeeds.y
    // };
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


float PID_Controller::updateOdom() {
    uint32_t now = pros::millis();

    robot->heading = truncate(robot->inertial.get_rotation()); // Cutoff at 2 decimal places because inertial sensor is innacurate

    float leftMotorsPos = robot->drivetrain.getLeftMotorsPos();
    float rightMotorsPos = robot->drivetrain.getRightMotorsPos();

    float averageWheelRot = (leftMotorsPos + rightMotorsPos) / 2;
    float wheelRotDelta = averageWheelRot - prev_allWheelRot;
    prev_allWheelRot = averageWheelRot;

    robot->pos.x += ((wheelRotDelta / 360.0f) * robot->drivetrain.GEAR_RATIO * robot->drivetrain.WHEEL_CIRCUMFERENCE) * sin(toRadians(robot->heading));
    robot->pos.y += ((wheelRotDelta / 360.0f) * robot->drivetrain.GEAR_RATIO * robot->drivetrain.WHEEL_CIRCUMFERENCE) * cos(toRadians(robot->heading));
    return (wheelRotDelta / 360.0f); // Return this for debugging purposes
}

} // namespace autonAPI
