#include "robotAPI.h"
#include <cmath>  // for atan2, cos, sin, abs
#include "autonAPI.h"


using namespace robotAPI;

// MARK: DrivetrainMotor
DrivetrainMotor::DrivetrainMotor()
	: rawMotor(pros::Motor(0, pros::v5::MotorGearset::green)), isLeftSide(false), RPM(0) {}

DrivetrainMotor::DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide)
	: rawMotor(_motor), isLeftSide(_isLeftSide) {
	if (isLeftSide) rawMotor.set_reversed(true);

	switch (_motor.get_gearing()) {
		case pros::v5::MotorGears::red:   RPM = 100; break;
		case pros::v5::MotorGears::green: RPM = 200; break;
		case pros::v5::MotorGears::blue:  RPM = 600; break;
		default: RPM = 0; break;
	}
}

void DrivetrainMotor::setVelocityPercent(float vel) {
	rawMotor.move_velocity((vel / 100.0f) * RPM);
}

void DrivetrainMotor::brake() {
	rawMotor.brake();
}

void DrivetrainMotor::setDirection(bool reversed) {
	rawMotor.set_reversed(reversed);
}

double DrivetrainMotor::getActualVelocity() {
	return rawMotor.get_actual_velocity();
}

// MARK: Drivetrain
Drivetrain::Drivetrain()
	: topLeft(DrivetrainMotor()), topRight(DrivetrainMotor()), bottomLeft(DrivetrainMotor()), bottomRight(DrivetrainMotor()) {}

Drivetrain::Drivetrain(std::array<DrivetrainMotor, 4> _wheels)
	: topLeft(_wheels.at(0)), topRight(_wheels.at(1)), bottomLeft(_wheels.at(2)), bottomRight(_wheels.at(3)) {}

Drivetrain::Drivetrain(DrivetrainMotor _topLeft, DrivetrainMotor _topRight, DrivetrainMotor _bottomLeft, DrivetrainMotor _bottomRight)
	: topLeft(_topLeft), topRight(_topRight), bottomLeft(_bottomLeft), bottomRight(_bottomRight) {};

// MARK: Heading
float Heading::getDegrees() const { return degrees; }
double Heading::getRadians() const { return toRadians(degrees); }
void Heading::setDegrees(float amount) { degrees = amount; }
void Heading::setRadians(double amount) { degrees = toDegrees(amount); }

// MARK: Robot
Robot::Robot()
	: inertial(pros::Imu(0)),
	  drivetrain(),
	  autonController(new autonAPI::PID_Controller()) {}

Robot::Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial)
	: drivetrain(_wheels),
	  inertial(_inertial),
	  autonController(new autonAPI::PID_Controller()) {}

void Robot::moveWheels(float &speedLeftPercent, float &speedRightPercent) {
	drivetrain.topLeft.setVelocityPercent(speedLeftPercent);
	drivetrain.topRight.setVelocityPercent(speedRightPercent);
	drivetrain.bottomLeft.setVelocityPercent(speedLeftPercent);
	drivetrain.bottomRight.setVelocityPercent(speedRightPercent);
}

void Robot::brakeWheels() {
	for (auto &wheel : drivetrain.getAll()) {
		wheel.brake();
	}
}

void Robot::setSpeedFromController(ctrlAPI::Controller &controller) {
	float leftSpeedPercent, rightSpeedPercent;

	if (driveMode == DrivingMode::SINGLE_JOYSTICK) {
		const float joystickRadians = atan2(controller.leftAnalogXPercent / 100.0f,
											controller.leftAnalogYPercent / 100.0f);
		const float alteredAnalogY = controller.leftAnalogYPercent * fabs(cos(joystickRadians));
		const float alteredAnalogX = controller.leftAnalogXPercent * fabs(sin(joystickRadians));
		leftSpeedPercent = alteredAnalogY + alteredAnalogX;
		rightSpeedPercent = alteredAnalogY - alteredAnalogX;
	} else {
		leftSpeedPercent = controller.leftAnalogYPercent;
		rightSpeedPercent = controller.rightAnalogYPercent;
	}

	moveWheels(leftSpeedPercent, rightSpeedPercent);
}
