#include "robotAPI.h"
#include "autonAPI.h"
#include <memory>
#include <cmath>  // for atan2, cos, sin, abs

using namespace robotAPI;

// MARK: DrivetrainMotor
DrivetrainMotor::DrivetrainMotor()
	: rawMotor(pros::Motor(1, pros::v5::MotorGearset::green)), reversed(false), RPM(0) {}

DrivetrainMotor::DrivetrainMotor(int port, bool _reversed, pros::v5::MotorGearset gearset)
	: rawMotor(pros::Motor(port, gearset)), reversed(_reversed) {
	rawMotor.set_reversed(_reversed);
	switch (rawMotor.get_gearing()) {
		case pros::v5::MotorGears::red:   RPM = 100; break;
		case pros::v5::MotorGears::green: RPM = 200; break;
		case pros::v5::MotorGears::blue:  RPM = 600; break;
		default: RPM = 0; break;
	}
}
DrivetrainMotor::DrivetrainMotor(pros::Motor _motor, bool _reversed)
	: rawMotor(_motor), reversed(_reversed) {
	rawMotor.set_reversed(_reversed);   
	switch (rawMotor.get_gearing()) {
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

Drivetrain::Drivetrain(std::array<DrivetrainMotor, 6> _wheels)
	: topLeft(_wheels.at(0)), middleLeft(_wheels.at(1)), bottomLeft(_wheels.at(2)), topRight(_wheels.at(3)), middleRight(_wheels.at(4)), bottomRight(_wheels.at(5)) {}

Drivetrain::Drivetrain(DrivetrainMotor _topLeft, DrivetrainMotor _middleLeft, DrivetrainMotor _bottomLeft, DrivetrainMotor _topRight, DrivetrainMotor _middleRight, DrivetrainMotor _bottomRight)
	: topLeft(_topLeft), middleLeft(_middleLeft), bottomLeft(_bottomLeft), topRight(_topRight), middleRight(_middleRight), bottomRight(_bottomRight) {};

Drivetrain::Drivetrain(std::array<pros::Motor, 6> _drivetrain, pros::motor_gearset_e _gearset)
	: topLeft(_drivetrain.at(0), false), middleLeft(_drivetrain.at(2), false), bottomLeft(_drivetrain.at(4), false), 
	  topRight(_drivetrain.at(1), false), middleRight(_drivetrain.at(3), false), bottomRight(_drivetrain.at(5), false) {}

// MARK: Heading
float Heading::getDegrees() const { return degrees; }
double Heading::getRadians() const { return toRadians(degrees); }
void Heading::setDegrees(float amount) { degrees = amount; }
void Heading::setRadians(double amount) { degrees = toDegrees(amount); }

// MARK: Robot
Robot::Robot(std::array<DrivetrainMotor, 6> _wheels, pros::Imu _inertial)
	: drivetrain(_wheels),
	  inertial(_inertial),
	  autonController(std::make_unique<autonAPI::PID_Controller>()) {
		autonController->bindRobot(this);
	  }

void Robot::moveWheels(float speedLeftPercent, float speedRightPercent) {
	drivetrain.topLeft.setVelocityPercent(speedLeftPercent);
	drivetrain.middleLeft.setVelocityPercent(speedLeftPercent);
	drivetrain.bottomLeft.setVelocityPercent(speedLeftPercent);

	drivetrain.topRight.setVelocityPercent(speedRightPercent);
	drivetrain.middleRight.setVelocityPercent(speedRightPercent);
	drivetrain.bottomRight.setVelocityPercent(speedRightPercent);
}

void Robot::brakeWheels() {
	for (auto &wheel : drivetrain.getAll_asPtr()) {
		wheel->brake();
	}
}

void Robot::setSpeedFromController(const ctrlAPI::Controller &controller) {
	float leftSpeedPercent, rightSpeedPercent;
	if (driveMode == DrivingMode::SINGLE_JOYSTICK) {
		leftSpeedPercent = controller.leftAnalogYPercent - controller.leftAnalogXPercent;
		rightSpeedPercent = controller.leftAnalogYPercent + controller.leftAnalogXPercent;
	} else {
		leftSpeedPercent = controller.rightAnalogYPercent;
		rightSpeedPercent = controller.leftAnalogYPercent;
	}
	moveWheels(leftSpeedPercent, rightSpeedPercent);
}

void Robot::swapDriveMode() {
	driveMode = (driveMode == DrivingMode::SINGLE_JOYSTICK) ? DrivingMode::TANK : DrivingMode::SINGLE_JOYSTICK;
}
