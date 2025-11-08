#include "lib/mainAPI.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <tuple>
#include <vector>

using namespace std;
using namespace ctrlAPI;
using namespace robotAPI;

// MARK: Hardware Init
Controller controller(pros::Controller(pros::E_CONTROLLER_MASTER));

pros::Motor conveyor(5, pros::v5::MotorGearset::green);
pros::Motor bandRotatorTop(6, pros::v5::MotorGearset::blue);
pros::Motor bandRotatorBottom(7, pros::v5::MotorGearset::green);
pros::Motor intake(20, pros::v5::MotorGearset::green);
pros::Motor agitator(19, pros::v5::MotorGearset::green);
void configureMotors() {
    conveyor.set_reversed(true);
	intake.set_reversed(true);
	
	conveyor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorTop.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorBottom.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	agitator.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}
pros::Imu inertial(5);
pros::adi::Pneumatics loaderRod('A', false);
pros::adi::Pneumatics descorer('B', false);




// MARK: Globals
float all_rot_prev = {0};

float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels

bool pressingPneumatics = false;
// MARK: Init robot
Robot init_robot() {
	pros::Motor topLeft(1, pros::v5::MotorGearset::blue);
	pros::Motor bottomLeft(2, pros::v5::MotorGearset::blue);
	pros::Motor topRight(3, pros::v5::MotorGearset::blue);
	pros::Motor bottomRight(4, pros::v5::MotorGearset::blue);
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	array<DrivetrainMotor, 4> drivetrain = {DrivetrainMotor(1, true), DrivetrainMotor(2, true), DrivetrainMotor(3, false), DrivetrainMotor(4, false)};
	return Robot(drivetrain, inertial);
}
Robot robot = init_robot();
vector<autonAPI::Point> autonPoints = {
	{0, 0},
	{0, 24},
	{12, 12}
};
DrivingMode drivingMode = DrivingMode::SINGLE_JOYSTICK;


//| NON-DEFAULT FUNCTIONS |//
// MARK: Utils

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

inline bool _otherButtonsOn(const Button &curBtn, const uint16_t &mask) {
	return btn_to_bool(mask & ~curBtn);
}

inline bool _otherBitsOn(const uint32_t &itm, const uint32_t &mask) {
	return mask & ~itm;
}




//| RUNTIME FUNCTIONS:

void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	if (controller.getNewPress(Button::X)) {
		robot.brakeWheels();
		while (!controller.getNewPress(Button::X)) {
			wait(10);
		}
	}
}


// MARK: Initialize
//| DEFAULT FUNCTIONS |//
void initialize() {
	configureMotors();
	init_robot();
	pros::lcd::initialize();
	robot.inertial.reset();
	wait(2300); // Wait for inertial to calibrate
}

// MARK: Disabled
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
	Vec2 curTargetLoc = {0, 0};
	Vec2 prevTargetLoc = {0, 0};
	float left_speed = 1000, right_speed = 1000;
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
void drivePipeline() {
	if (controller.getNewPress(Button::X)) {
		robot.swapDriveMode();
	}
	robot.setSpeedFromController(controller);
}

void brakeScoring(vector<pros::Motor*> scoringMotors) {
	for (int i = 0; i < scoringMotors.size(); i++) {
		scoringMotors.at(i)->brake();
	}
}

// MARK: Scoring
void scorePipeline() {
	float maxElevatorSpeed = 200.0f;
	if (controller.getNewPress(Button::A)) {
		loaderRod.toggle();
	}
	if (controller.getNewPress(Button::B)) {
		descorer.toggle();
	}
	if (controller.getPressing(Button::R2) && !controller.otherL_or_RPressed(Button::R2)) {
		conveyor.move_velocity(-200);
		brakeScoring({&intake, &bandRotatorTop, &bandRotatorBottom, &agitator});
	} else if (controller.getPressing(Button::R1) && !controller.otherL_or_RPressed(Button::R1)) {
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(-275);
		intake.move_velocity(200);
		agitator.move_velocity(-200);
		brakeScoring({&bandRotatorBottom});
	} else if (controller.getPressing(Button::L1) && !controller.otherL_or_RPressed(Button::L1)) {
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(275);
		bandRotatorBottom.move_velocity(200);
		agitator.move_velocity(-200);
		brakeScoring({&intake});
	} else if (controller.getPressing(Button::L2) && !controller.otherL_or_RPressed(Button::L2)) {
		conveyor.move_velocity(-200);
		intake.move_velocity(-200);
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		agitator.brake();
		brakeScoring({&agitator, &bandRotatorTop, &bandRotatorBottom});
	} else {
		brakeScoring({&conveyor, &intake, &bandRotatorTop, &bandRotatorBottom, &agitator});
	}
}


// MARK: opcontrol
void opcontrol() {
	// autonomous();
	clear_screen();
	// robot.moveWheels(driveSpeedMult, driveSpeedMult);
	// wait(3000);
	// robot.brakeWheels();
	initialize();
	printOnScreen(to_string(robot.pos.y));
	wait(2500);
	while (robot.pos.y < 24) {
		// if (robot.drivetrain.topLeft.rawMotor.is_over_temp() || robot.drivetrain.topRight.rawMotor.is_over_temp() || robot.drivetrain.bottomLeft.rawMotor.is_over_temp() || robot.drivetrain.bottomRight.rawMotor.is_over_temp()) controller.rawController.rumble("---");
		// controller.updateInputData();
		robot.autonController->updateOdom();
		robot.moveWheels(100, 100);
		// drivePipeline();
		// scorePipeline();
		wait(FRAME);
	}
	// while (true) {
	// 	if (robot.drivetrain.topLeft.rawMotor.is_over_temp() || robot.drivetrain.topRight.rawMotor.is_over_temp() || robot.drivetrain.bottomLeft.rawMotor.is_over_temp() || robot.drivetrain.bottomRight.rawMotor.is_over_temp()) controller.rawController.rumble("---");
	// 	controller.updateInputData();
	// 	// robot.autonController->updateOdom();
	// 	drivePipeline();
	// 	scorePipeline();
	// 	wait(FRAME);
	// }
	printOnScreen(to_string(robot.pos.y));
	robot.brakeWheels();
	conveyor.brake();
	intake.brake();
	bandRotatorBottom.brake();
	bandRotatorTop.brake();
	agitator.brake();
}
