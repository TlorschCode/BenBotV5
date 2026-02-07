/*###|   autonAPI.cpp   |###*/

#include "autonAPI.h"
#include "robotAPI.h"
#include <cmath>
#include <algorithm>
#include <vector>




namespace autonAPI {

// Returns the left and right speed a robot's drivetrain should adopt in order to go towards a point using PID.
// (Uses dot product to control rotation PID)
_nodiscard_ SpeedPair AutonController::getPID_speedTo(Vec2& target) {
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
    const float curRotError = rad2deg(std::atan2(crossProd(headingVec, targRotVec), dotProd(headingVec, targRotVec))); // from -180 to 180 (radians is -pi to pi)
    // PID calc
    pRot = curRotError * pRot.weight;
    iRot += curRotError * iRot.weight;
    dRot = dRot.weight * (curRotError - prevRotError);
    prev_pRot = pRot.val;
    const float PID_rot = pRot.val;//(pRot + (iRot * iRot.weight) + (dRot * dRot.weight)).val;


    //| Position
    const float curPosError = distanceBetween(robot->pos, target); 
    // PID calc
    pPos = curPosError * pPos.weight;
    iPos += curPosError * iPos.weight;
    dPos = dPos.weight * (curPosError - prevPosError);
    prev_pPos = pPos.val;
    const float PID_pos = pPos.val;//(pPos + (iPos * iPos.weight) + (dPos * dPos.weight)).val;

    //| Cleanup
    prevRotError = curRotError;
    prevPosError = curPosError;

    //| Result
    if (curRotError < 0) {}
    return SpeedPair{
        (pPos - pRot).val,
        (pPos + pRot).val
    };
}

// Uses PID and Pure Pursuit to drive along a path.
// Pass `points` by std::move for optimal performance, so long as the orginal array is no longer needed
std::vector<Point> AutonController::driveAlongPath(std::vector<Point> points, float robotCheckRadius) {
    Point target = points.at(0);
	Point prevPoint = robot->pos;
	Vec2 curTargetLoc = {0, 0};  // The current intersect
	Vec2 prevTargetLoc = {0, 0}; // Last valid intersect
	robot->drivetrain.brakeWheels();
	for (size_t ptIdx = 0; ptIdx < points.size() - 1; ptIdx++) { // - 1 so we can have an extra point the robot doesn't visit but it aims for
		Point& point = points.at(ptIdx);
		while (!point.visited) {
            updateHeadingAndOdom();
			curTargetLoc = robot->autonController.load().get()->getPurePursuitLoc(robotCheckRadius, point.pos);
			SpeedPair result = robot->autonController.load().get()->getPID_speedTo(curTargetLoc);
			robot->drivetrain.setSpeedFromSpeedPair(result);
			printOnScreen(distanceBetween(robot->pos, point.pos));
			printOnScreen((distanceBetween(robot->pos, point.pos) < 2), 1);
			printOnScreen(std::to_string(point.pos.x) + ", " + std::to_string(point.pos.y), 2);
			// printOnScreen(robot->drivetrain.leftSpeed, 3);
			// printOnScreen(robot->drivetrain.rightSpeed, 4);
			printOnScreen(result.leftSpeed, 4);
			printOnScreen(result.rightSpeed, 5);
			// When within 2 inches of a point, mark that point as visted
			point.visited = (distanceBetween(robot->pos, point.pos) < 2);
			robot->drivetrain.moveWheels();
			prevTargetLoc = curTargetLoc;
		}
	}
    return points;
}

_nodiscard_ Vec2 AutonController::getPurePursuitLoc(const float &checkRadius, const Vec2 &target) {
    static Vec2 lastValidLookaheadPoint{0, 0};

    double selfDotProduct = std::pow(target.x - lastValidLookaheadPoint.x, 2) + std::pow(target.y - lastValidLookaheadPoint.y, 2);
    if (selfDotProduct < ELIPSON_DOUBLE) { // if the target and last valid lookahead point are basically the same
        return target;
    }
    double twoceCircleOffset = 2 * (((lastValidLookaheadPoint.x - robot->pos.x) * (target.x - lastValidLookaheadPoint.x)) + ((lastValidLookaheadPoint.y - robot->pos.y) * (target.y - lastValidLookaheadPoint.y)));
    double prevTarCurPosDist = (std::pow(lastValidLookaheadPoint.x - robot->pos.x, 2) + std::pow(lastValidLookaheadPoint.y - robot->pos.y, 2)) - std::pow(checkRadius, 2);
    double discriminant = std::pow(twoceCircleOffset, 2) - (4 * selfDotProduct * prevTarCurPosDist);

    if (discriminant >= 0) { // if the circle intersects the line at all (assuming the line extends infinitely)
        const float intersectRatio1 = (-twoceCircleOffset + sqrt(discriminant)) / (2 * selfDotProduct); // t value
        const float intersectRatio2 = (-twoceCircleOffset - sqrt(discriminant)) / (2 * selfDotProduct); // t value

        const Vec2 intersect1 = {lastValidLookaheadPoint.x + (target.x - lastValidLookaheadPoint.x) * intersectRatio1, lastValidLookaheadPoint.y + (target.y - lastValidLookaheadPoint.y) * intersectRatio1};
        const Vec2 intersect2 = {lastValidLookaheadPoint.x + (target.x - lastValidLookaheadPoint.x) * intersectRatio2, lastValidLookaheadPoint.y + (target.y - lastValidLookaheadPoint.y) * intersectRatio2};

        const bool intersect1Valid = (intersectRatio1 >= 0 && intersectRatio1 <= 1);
        const bool intersect2Valid = (intersectRatio2 >= 0 && intersectRatio2 <= 1);

        if (intersect1Valid && intersect2Valid) { // The robot has two intersects to choose from. Choose the one furthest along the line (closest to target)
            if (intersectRatio1 > intersectRatio2) {
                return lastValidLookaheadPoint = intersect1;
            } else {
                return lastValidLookaheadPoint = intersect2;
            }
        } else if (intersect1Valid) { // check if intersect1 is on the line (segment)
            return lastValidLookaheadPoint = intersect1;
        } else if (intersect2Valid) { // check if intersect2 is on the line (segment)
            return lastValidLookaheadPoint = intersect2;
        }
    }
    return target; // Robot's circle did not interesect the line. Return the target's raw position
}


float AutonController::updateHeadingAndOdom() {
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
