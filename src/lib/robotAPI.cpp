/*###|   robotAPI.cpp   |###*/

#include "robotAPI.h"
#include "autonAPI.h"
#include "common.h"
#include <memory>
#include <mutex>
#include <cmath>  // for atan2, cos, sin, abs

using namespace robotAPI;
//| MARK: DrivetrainMotor
DrivetrainMotor::DrivetrainMotor()
			: rawMotor(pros::Motor(1, pros::v5::MotorGearset::green)), reversed(false), RPM(0) {
	rawMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	configMotor(rawMotor, false);
}

DrivetrainMotor::DrivetrainMotor(int port, bool _reversed, pros::v5::MotorGearset gearset)
			: rawMotor(pros::Motor(port, gearset)), reversed(_reversed) {
	configMotor(rawMotor, _reversed);
	switch (rawMotor.get_gearing()) {
		case pros::v5::MotorGears::red:   RPM = 100; break;
		case pros::v5::MotorGears::green: RPM = 200; break;
		case pros::v5::MotorGears::blue:  RPM = 600; break;
		default: RPM = 0; break;
	}
}
DrivetrainMotor::DrivetrainMotor(const pros::Motor &_motor, bool _reversed)
	: rawMotor(_motor), reversed(_reversed) {
	configMotor(rawMotor, _reversed);
	switch (rawMotor.get_gearing()) {
		case pros::v5::MotorGears::red:   RPM = 100; break;
		case pros::v5::MotorGears::green: RPM = 200; break;
		case pros::v5::MotorGears::blue:  RPM = 600; break;
		default: RPM = 0; break;
	}
}
void DrivetrainMotor::configMotor(pros::Motor &_motor, bool _reversed) {
	_motor.set_reversed(_reversed);
	_motor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	_motor.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES); // Could set to rotations, but degrees is more intuitive
}

void DrivetrainMotor::setVelocityPercent(float vel) {
	rawMotor.move_velocity((vel / 100.0f) * RPM);
}
void DrivetrainMotor::brake() {
	rawMotor.brake();
}
void DrivetrainMotor::setReversed(bool reversed) {
	rawMotor.set_reversed(reversed);
}


//| MARK: Drivetrain

Drivetrain::Drivetrain()
	: w_topLeft(DrivetrainMotor()), w_topRight(DrivetrainMotor()), w_bottomLeft(DrivetrainMotor()), w_bottomRight(DrivetrainMotor()),
	  WHEEL_CIRCUMFERENCE(0), GEAR_RATIO(0) {}

// Drivetrain constructor
// hardwareGearRatio is the motor-turns to wheel-turns ratio, for robots with gearing between the motor shaft and the actual wheel
Drivetrain::Drivetrain(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio=1)
	: w_topLeft(_wheels.at(0)), w_middleLeft(_wheels.at(1)), w_bottomLeft(_wheels.at(2)), w_topRight(_wheels.at(3)), w_middleRight(_wheels.at(4)), w_bottomRight(_wheels.at(5)),
	  WHEEL_CIRCUMFERENCE(wheelCircumference), GEAR_RATIO(hardwareGearRatio) {}

// Drivetrain constructor
// hardwareGearRatio is the motor-turns to wheel-turns ratio, for robots with gearing between the motor shaft and the actual wheel
Drivetrain::Drivetrain(const DrivetrainMotor &_topLeft, const DrivetrainMotor &_middleLeft, const DrivetrainMotor &_bottomLeft, const DrivetrainMotor &_topRight, const DrivetrainMotor &_middleRight, const DrivetrainMotor &_bottomRight,
					   float wheelCircumference, float hardwareGearRatio=1)
	: w_topLeft(_topLeft), w_middleLeft(_middleLeft), w_bottomLeft(_bottomLeft), w_topRight(_topRight), w_middleRight(_middleRight), w_bottomRight(_bottomRight),
	  WHEEL_CIRCUMFERENCE(wheelCircumference), GEAR_RATIO(hardwareGearRatio) {};

Drivetrain::Drivetrain(const std::array<pros::Motor, 6> &_drivetrain, pros::motor_gearset_e _gearset, float wheelCircumference, float hardwareGearRatio=1)
	: w_topLeft(_drivetrain.at(0), false), w_middleLeft(_drivetrain.at(1), false), w_bottomLeft(_drivetrain.at(2), false), 
	  w_topRight(_drivetrain.at(3), false), w_middleRight(_drivetrain.at(4), false), w_bottomRight(_drivetrain.at(5), false),
	  WHEEL_CIRCUMFERENCE(wheelCircumference), GEAR_RATIO(hardwareGearRatio) {}

void Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
	for (DrivetrainMotor *motor : getWheelsAsPtrs()) {
		motor->rawMotor.set_brake_mode(mode);
	}
}
void Drivetrain::moveWheels() {
	w_topLeft.setVelocityPercent(leftSpeed);
	w_middleLeft.setVelocityPercent(leftSpeed);
	w_bottomLeft.setVelocityPercent(leftSpeed);

	w_topRight.setVelocityPercent(rightSpeed);
	w_middleRight.setVelocityPercent(rightSpeed);
	w_bottomRight.setVelocityPercent(rightSpeed);
}
void Drivetrain::moveWheels(float speedLeft, float speedRight) {
	this->leftSpeed = speedLeft;
	this->rightSpeed = speedRight;
	moveWheels();
}
void Drivetrain::setSpeedFromController(const ctrlAPI::Controller &controller) {
	if (driveMode == DrivingMode::SINGLE_JOYSTICK) {
		this->leftSpeed = controller.leftAnalogYPercent - controller.leftAnalogXPercent;
		this->rightSpeed = controller.leftAnalogYPercent + controller.leftAnalogXPercent;
	} else {
		this->rightSpeed = controller.rightAnalogYPercent;
		this->leftSpeed = controller.leftAnalogYPercent;
	}
	moveWheels();
}
void Drivetrain::brakeWheels() {
	for (DrivetrainMotor* motor : getWheelsAsPtrs()) {
		motor->brake();
	}
	this->leftSpeed = 0;
	this->rightSpeed = 0;
}
void Drivetrain::setSpeedFromSpeedPair(const SpeedPair &pair) {
	this->leftSpeed = pair.leftSpeed;
	this->rightSpeed = pair.rightSpeed;
}


//| MARK: Robot

Robot* Robot::instance = nullptr; // Initialize instance to nullptr

Robot& Robot::Init(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio, const pros::Imu &_inertial) {
	if (Robot::instance == nullptr) {
		Robot::instance = new Robot(_wheels, wheelCircumference, hardwareGearRatio, _inertial);
		return *Robot::instance;

	} else {
		throw std::logic_error("Cannot create multiple instances of singleton Robot.");
	}
}

Robot& Robot::Get() {
	if (instance == nullptr) {
		throw std::logic_error("Cannot call Get on singleton Robot with uninitialized instance");
	} else {
		return *Robot::instance;
	}
}

Robot::Robot(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio, const pros::Imu &_inertial)
		: drivetrain(_wheels, wheelCircumference, hardwareGearRatio), inertial(_inertial),
		  autonController(std::make_unique<autonAPI::AutonController>()) {
	autonController.load()->_bindRobot(this);
}
