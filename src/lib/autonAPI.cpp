/*###|   autonAPI.cpp   |###*/

#include "autonAPI.h"
#include "robotAPI.h"


namespace autonAPI {

using Intersect = std::variant<Vec2, std::nullptr_t>;

// Returns the left and right speed a robot's drivetrain should adopt in order to go towards a point using PID.
// (Uses dot product to control rotation PID)
NODISCARD SpeedPair AutonController::getPID_speedTo(Vec2& target) {
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
    const float PID_rot = (pRot + (iRot * iRot.weight) + (dRot * dRot.weight)).val;


    //| Position
    const float curPosError = distanceBetween(robot->pos, target); 
    // PID calc
    pPos = curPosError * pPos.weight;
    iPos += curPosError * iPos.weight;
    dPos = dPos.weight * (curPosError - prevPosError);
    prev_pPos = pPos.val;
    const float PID_pos = (pPos + (iPos * iPos.weight) + (dPos * dPos.weight)).val;

    //| Cleanup
    prevRotError = curRotError;
    prevPosError = curPosError;

    //| Result
    if (curRotError < 0) {}
    // return pos - rot, pos + rot
    return SpeedPair{
        -PID_rot,
        PID_rot
    };
}

// Uses PID and Pure Pursuit to drive along a path.
// Pass `points` by std::move for optimal performance, so long as the orginal array is no longer needed
std::vector<Point> AutonController::driveAlongPath(std::vector<Point> points) {
    Point target = points.at(0);
	Point prevPoint = robot->pos;
	Vec2 curTargetLoc = {0, 0};  // The current intersect
	Vec2 prevTargetLoc = {0, 0}; // Last valid intersect
	robot->drivetrain.brakeWheels();
	for (size_t ptIdx = 0; ptIdx < points.size() - 1; ptIdx++) { // - 1 so we can have an extra point the robot doesn't visit but it aims for
		Point& point = points.at(ptIdx);
        const Point& nextPoint = points.at(ptIdx + 1);
        robot->autonController.load().get()->getPurePursuitLoc({0, 0}, {0, 0}, resetStatics); // Reset lastValidLookaheadPoint

		while (!point.visited) {
            updateHeadingAndOdom();
			curTargetLoc = robot->autonController.load().get()->getPurePursuitLoc(point, nextPoint, keepStatics);
			SpeedPair result = robot->autonController.load().get()->getPID_speedTo(curTargetLoc);
			robot->drivetrain.setSpeedFromSpeedPair(result);

            //* VVV Debug VVV
			printOnScreen(distanceBetween(robot->pos, point));
			printOnScreen((distanceBetween(robot->pos, point) < 2), 1);
			printOnScreen(std::to_string(point.x) + ", " + std::to_string(point.y), 2);
			// printOnScreen(robot->drivetrain.leftSpeed, 3);
			// printOnScreen(robot->drivetrain.rightSpeed, 4);
			printOnScreen(result.leftSpeed, 4);
			printOnScreen(result.rightSpeed, 5);
            //* ^^^ Debug ^^^

			// When within 2 inches of a point, mark that point as visted
			point.visited = (distanceBetween(robot->pos, point) < 2);
			robot->drivetrain.moveWheels();
			prevTargetLoc = curTargetLoc;
		}
	}
    return points;
}

Intersect AutonController::getIntersect(const Vec2 &target, const Vec2 lastValidLookaheadPoint) {
    double lastIntersectToTargSqr = std::pow(target.x - lastValidLookaheadPoint.x, 2) + std::pow(target.y - lastValidLookaheadPoint.y, 2);
    if (lastIntersectToTargSqr < ELIPSON_DOUBLE) { // if the target and last valid lookahead point are basically the same
        return target;
    }
    double twiceCircleOffset = 2 * (((lastValidLookaheadPoint.x - robot->pos.x) * (target.x - lastValidLookaheadPoint.x)) + ((lastValidLookaheadPoint.y - robot->pos.y) * (target.y - lastValidLookaheadPoint.y)));
    double prevTarCurPosDist = (std::pow(lastValidLookaheadPoint.x - robot->pos.x, 2) + std::pow(lastValidLookaheadPoint.y - robot->pos.y, 2)) - std::pow(checkRadius, 2);
    double discriminant = std::pow(twiceCircleOffset, 2) - (4 * lastIntersectToTargSqr * prevTarCurPosDist);

    if (discriminant >= 0) { // if the circle intersects the line at all (assuming the line extends infinitely)
        const float intersectRatio1 = (-twiceCircleOffset + sqrt(discriminant)) / (2 * lastIntersectToTargSqr); // t value
        const float intersectRatio2 = (-twiceCircleOffset - sqrt(discriminant)) / (2 * lastIntersectToTargSqr); // t value

        const Vec2 intersect1 = {lastValidLookaheadPoint.x + (target.x - lastValidLookaheadPoint.x) * intersectRatio1, lastValidLookaheadPoint.y + (target.y - lastValidLookaheadPoint.y) * intersectRatio1};
        const Vec2 intersect2 = {lastValidLookaheadPoint.x + (target.x - lastValidLookaheadPoint.x) * intersectRatio2, lastValidLookaheadPoint.y + (target.y - lastValidLookaheadPoint.y) * intersectRatio2};

        const bool intersect1Valid = (intersectRatio1 >= 0 && intersectRatio1 <= 1);
        const bool intersect2Valid = (intersectRatio2 >= 0 && intersectRatio2 <= 1);

        if (intersect1Valid && intersect2Valid) { // the robot has two intersects to choose from. Choose the one furthest along the line (closest to target)
            if (intersectRatio1 > intersectRatio2) {
                return intersect1;
            } else {
                return intersect2;
            }
        } else if (intersect1Valid) { // check if intersect1 is on the line (segment)
            return intersect1;
        } else if (intersect2Valid) { // check if intersect2 is on the line (segment)
            return intersect2;
        }
    }
    return nullptr; // Robot's circle did not interesect the line. Return the target's raw position
}

Vec2 AutonController::getPurePursuitLoc(const Vec2 &target, const Vec2& nextTarget, bool resetLastLookahead) {
    static Vec2 lastValidLookaheadPoint{0, 0};

    if (resetLastLookahead) {
        lastValidLookaheadPoint = robot->pos;
        return lastValidLookaheadPoint;
    }

    const Intersect intersectCur = getIntersect(target, lastValidLookaheadPoint);
    const Intersect intersectNext = getIntersect(nextTarget, lastValidLookaheadPoint);
    Vec2 chosenIntersect;

    if (std::holds_alternative<Vec2>(intersectCur) && std::holds_alternative<Vec2>(intersectNext)) {
        chosenIntersect = std::get<Vec2>(intersectNext);
    } else if (std::holds_alternative<Vec2>(intersectCur)) {
        chosenIntersect = std::get<Vec2>(intersectCur);
    } else {
        chosenIntersect = target;
    }


    if (chosenIntersect != target) { // update lastValidLookaheadPoint if intersect is different from target because it will be target if there wasn't a valid intercept
        lastValidLookaheadPoint = chosenIntersect;
    }

    return lastValidLookaheadPoint;
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
