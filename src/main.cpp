#include "mainAPI.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <tuple>

using namespace std;
using namespace ctrlAPI;
// MARK: Templates
struct Vector2;
struct Point;
class Heading;
class PID_Controller;
void checkPauseProgram();



// MARK: Enums
enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};





// MARK: Hardware Init
Controller controller(pros::Controller(pros::E_CONTROLLER_MASTER));
pros::Motor topLeft(1, pros::v5::MotorGearset::blue);
pros::Motor bottomLeft(2, pros::v5::MotorGearset::blue);
pros::Motor topRight(3, pros::v5::MotorGearset::blue);
pros::Motor bottomRight(4, pros::v5::MotorGearset::blue);
pros::Motor conveyor(5, pros::v5::MotorGearset::green);
pros::Motor bandRotatorTop(6, pros::v5::MotorGearset::blue);
pros::Motor bandRotatorBottom(7, pros::v5::MotorGearset::green);
pros::Motor intake(20, pros::v5::MotorGearset::green);
pros::Motor agitator(9, pros::v5::MotorGearset::green);
void configureMotors() {
    conveyor.set_reversed(true);
	intake.set_reversed(true);
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	conveyor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorTop.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorBottom.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	agitator.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}
pros::Imu inertial(5);
pros::adi::Pneumatics loaderRod('A', false);




// MARK: Globals
float all_rot_prev = {0};
float rightAnalogX = {0};
float rightAnalogY = {0};
float leftAnalogX = {0};
float leftAnalogY = {0};
float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels

bool pressingPneumatics = false;
array<robotAPI::DrivetrainMotor, 4> drivetrain = {robotAPI::DrivetrainMotor(topLeft, true), robotAPI::DrivetrainMotor(bottomLeft, true), robotAPI::DrivetrainMotor(topRight, false), robotAPI::DrivetrainMotor(bottomRight, false)};
robotAPI::Robot robot = robotAPI::Robot(drivetrain, inertial);
vector<autonAPI::Point> autonPoints = {
	{0, 0},
	{0, 24},
	{12, 12}
};
uint16_t buttons = {0};
uint16_t buttonsNewPress = {0};
DrivingMode drivingMode = DrivingMode::SINGLE_JOYSTICK;





//| NON-DEFAULT FUNCTIONS |//
// MARK: Cust. Utils

void wait(int time) {
	pros::delay(time);
}

void clear_screen() {
	pros::lcd::clear_line(1);
	pros::lcd::clear_line(2);
	pros::lcd::clear_line(3);
	pros::lcd::clear_line(4);
	pros::lcd::clear_line(5);
	pros::lcd::clear_line(6);
}




//| RUNTIME FUNCTIONS:

// Updates button as a uint16_t





void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	if (controller.getNewPress(Button::X)) {
		brakeWheels();
		while (!controller.getNewPress(Button::X)) {
			wait(10);
		}
	}
}

// MARK: Move Funcs
//| MOVEMENT FUNCTIONS

void brakeWheels() {
	topLeft.brake();
	bottomLeft.brake();
	topRight.brake();
	bottomRight.brake();
}

void trackPosition() {
	
}

//| DEFAULT FUNCTIONS |//
void initialize() {
	configureMotors();
	pros::lcd::initialize();
	inertial.reset();
	wait(2300); // Wait for inertial to calibrate
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// MARK: Autonomous
void autonomous() {
	constexpr float robotCheckRadius = 10.0f;
	autonAPI::Point target = autonPoints.at(0);
	autonAPI::Point prevPoint = robot.pos;
	Vector2 curTargetLoc = {0, 0};
	Vector2 prevTargetLoc = {0, 0};
	float left_speed = 100, right_speed = 100;
	wait(12000);
	robot.moveWheels(left_speed, right_speed);
	wait(1000);
	robot.brakeWheels();
	// for (size_t ptIdx = 0; ptIdx < autonPoints.size(); ptIdx++) {
	// 	Point &point = autonPoints.at(ptIdx);
	// 	while (!point.visited) {
			// curTargetLoc = getPurePursuitLoc(robotCheckRadius, point.pos, prevTargetLoc);
	// 		// robot.pidController.update(curTargetLoc, robot);
	// 	}
	// }
}


// MARK: Driving
void drivePipeline(float driveSpeedMult) {
	// Controller analog is -1 to 1
	float left_speed = 0, right_speed = 0;
	if (buttonsNewPress & Button::X) {
		drivingMode = (drivingMode == DrivingMode::SINGLE_JOYSTICK) ? DrivingMode::TANK : DrivingMode::SINGLE_JOYSTICK;
	}
	if (drivingMode == DrivingMode::SINGLE_JOYSTICK) {
		const float joystickRadians = atan2(leftAnalogX / 100, leftAnalogY / 100);
		const float alteredAnalogX = leftAnalogX * abs(sin(joystickRadians)) * (driveSpeedMult / 100);
		const float alteredAnalogY = leftAnalogY * abs(cos(joystickRadians)) * (driveSpeedMult / 100);
		left_speed = alteredAnalogY + alteredAnalogX;
		right_speed = alteredAnalogY - alteredAnalogX;
		// right_speed = (leftAnalogY * cos(joystickRadians) * (driveSpeedMult / 100));
		// left = analogX * abs(sin(atan2(analogX, analogY)));
		// up = analogY * abs(cos(atan2(analogX, analogY)));
		// left_speed = (up * reversed) - left;
		// right_speed = (up * reversed) + left;
	} else if (drivingMode == DrivingMode::TANK) {
		left_speed = (leftAnalogY * (driveSpeedMult / 100));
		right_speed = (rightAnalogY * (driveSpeedMult / 100));
	}
	robot.moveWheels(left_speed, right_speed);
	trackPosition();
}

inline bool _otherBitsOn(const Button &curBtn, const uint16_t &mask) {
	return mask & ~curBtn;
}

// MARK: Scoring
void scorePipeline(float *driveSpeedMult) {
	constexpr uint16_t elevatorButtonMask = Button::R2 | Button::R1 | Button::L2 | Button::L1;
	float maxElevatorSpeed = 200.0f;
	uint16_t first_pressed = 0;
	if (buttonsNewPress & Button::A) {
		loaderRod.toggle();
	}
	if (buttonsNewPress & Button::Up) {
		*driveSpeedMult += 10;
	} if (buttons & Button::Down && first_pressed != Button::Up) {
		*driveSpeedMult -= 10;
	}
	if (buttons & Button::R2 && !_otherBitsOn(Button::R2, buttons & elevatorButtonMask)) {
		conveyor.move_velocity(-200);
		intake.brake();
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		agitator.brake();
	} else if (buttons & Button::R1 && !_otherBitsOn(Button::R1, buttons & elevatorButtonMask)) {
		first_pressed = Button::R1;
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(-275);
		intake.move_velocity(200);
		bandRotatorBottom.brake();
		agitator.move_velocity(200);
	} else if (buttons & Button::L1 && !_otherBitsOn(Button::L1, buttons & elevatorButtonMask)) {
		first_pressed = Button::L1;
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(275);
		bandRotatorBottom.move_velocity(200);
		intake.brake();
		agitator.move_velocity(200);
	} else if (buttons & Button::L2 && !_otherBitsOn(Button::L2, buttons & elevatorButtonMask)) {
		conveyor.move_velocity(-200);
		intake.move_velocity(-200);
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		agitator.brake();
	} else {
		first_pressed = Button::None;
		conveyor.brake();
		bandRotatorTop.brake();
		intake.brake();
		bandRotatorBottom.brake();
		agitator.brake();
	}
}


// MARK: opcontrol
void opcontrol() {
	// autonomous();
	// 72 inches across the field
	float driveSpeedMult = 100.0f;
	clear_screen();
	// robot.moveWheels(driveSpeedMult, driveSpeedMult);
	// wait(3000);
	// robot.brakeWheels();
	while (true) {
		controller.updateInputData();
		robot.updateOdometry();
		if (robot.wheels.at(0).rawMotor.is_over_temp() || robot.wheels.at(1).rawMotor.is_over_temp() || robot.wheels.at(2).rawMotor.is_over_temp() || robot.wheels.at(3).rawMotor.is_over_temp()) {
			break;
		}
		drivePipeline(driveSpeedMult);
		scorePipeline(&driveSpeedMult);
		wait(FRAME);
	}
	controller.rawController.rumble("---");
	brakeWheels();
}
