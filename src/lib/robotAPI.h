#pragma once
#include "globalAPI.h"
#include <array>
class DrivetrainMotor;
class Robot;
namespace autonAPI {
	class PID_Controller;
}

namespace robotAPI {
	
// MARK: Drivetrain
class DrivetrainMotor {
	private:
		int RPM;
	public:
		pros::Motor rawMotor;
		bool isLeftSide;
		DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide) : rawMotor(_motor) {
			isLeftSide = _isLeftSide;
			if (isLeftSide) rawMotor.set_reversed(true);
			switch (_motor.get_gearing()) {
				case pros::v5::MotorGears::red:    // high torque (36:1)
					RPM = 100; break;
				case pros::v5::MotorGears::green:  // standard (18:1)
					RPM = 200; break;
				case pros::v5::MotorGears::blue:   // high speed (6:1)
					RPM = 600; break;
				default:
					RPM = 0; break;
			}
		}
		void setVelocity(float vel) {
			rawMotor.move_velocity((vel / 100) * RPM);
		}
		void brake() {
			rawMotor.brake();
		}
		void setDirection(bool reversed) {
			rawMotor.set_reversed(reversed);
		}
		inline double getActualVelocity() {
			return rawMotor.get_actual_velocity();
		}
};

class Heading {
	private:
		float degrees = 0;
	public:
		inline float getDegrees() const {
			return degrees;
		}
		inline double getRadians() const {
			return toRadians(degrees);
		}
		void setDegrees(float amount) {
			degrees = amount;
		}
		void setRadians(double amount) {
			degrees = toDegrees(amount);
		}
};

//MARK: ROBOT
class Robot {  // Robot class for more readable code
	private:
	public:
		Vector2 pos = {0, 0};
		Heading heading = {};
		std::array<DrivetrainMotor, 4> wheels;
		pros::Imu inertial;
		// PID_Controller pidController();
		Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial) : wheels(_wheels), inertial(_inertial) {}
		void moveWheels(float &speedLeftPercent, float &speedRightPercent) {
			wheels.at(0).setVelocity(wheels.at(0).isLeftSide ? speedLeftPercent : speedRightPercent); // top left
			wheels.at(1).setVelocity(wheels.at(1).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom left
			wheels.at(2).setVelocity(wheels.at(2).isLeftSide ? speedLeftPercent : speedRightPercent); // top right
			wheels.at(3).setVelocity(wheels.at(3).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom right
		}
		void brakeWheels() {
			wheels.at(0).brake();
			wheels.at(1).brake();
			wheels.at(2).brake();
			wheels.at(3).brake();
		}
		void updateOdometry() {
			float allRotPrev = 0.0f;
			uint32_t now = pros::millis();
			heading.setDegrees(truncate(inertial.get_rotation()));
			float leftMotorsPos = (wheels.at(0).rawMotor.get_raw_position(&now) + wheels.at(1).rawMotor.get_raw_position(&now)) / 2;
			float rightMotorsPos = (wheels.at(2).rawMotor.get_raw_position(&now) + wheels.at(3).rawMotor.get_raw_position(&now)) / 2;
			float averageWheelRot = (leftMotorsPos + rightMotorsPos) / 2;
			float wheelRotDelta = averageWheelRot - allRotPrev;
			pos.x += ((wheelRotDelta / 360) * GEAR_RATIO * WHEEL_CIRCUMFERENCE) * sin(heading.getRadians());
			pos.y += ((wheelRotDelta / 360) * GEAR_RATIO * WHEEL_CIRCUMFERENCE) * cos(heading.getRadians());
			allRotPrev = averageWheelRot;
		}
};

} // namespace robotAPI