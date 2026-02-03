#include "lib/common_includes.h"
#include "pros/misc.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <tuple>
#include <vector>
#include <thread>
#include <atomic>


constexpr pros::motor_brake_mode_e BRAKE_MODE_BRAKE = pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE;
constexpr pros::motor_brake_mode_e BRAKE_MODE_HOLD = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
constexpr pros::motor_brake_mode_e BRAKE_MODE_COAST = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
constexpr pros::motor_brake_mode_e BRAKE_MODE_INVALID = pros::motor_brake_mode_e::E_MOTOR_BRAKE_INVALID;

using namespace ctrlAPI;
using namespace robotAPI;

std::atomic<bool> running_program(true);

pros::Task* odomThread = nullptr;

// MARK: Hardware Init
Controller controller(pros::Controller(pros::E_CONTROLLER_MASTER));

pros::Motor conveyor(7, pros::v5::MotorGearset::green);
pros::Motor bandRotatorTop(8, pros::v5::MotorGearset::blue);
pros::Motor bandRotatorBottom(9, pros::v5::MotorGearset::green);
pros::Motor intake(10, pros::v5::MotorGearset::green);
void configureMotors() {
    conveyor.set_reversed(true);
	intake.set_reversed(true);
	
	conveyor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorTop.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bandRotatorBottom.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}
pros::Imu inertial(11);
pros::adi::Pneumatics loaderRod('A', false);
pros::adi::Pneumatics descorer('B', false);




// MARK: Globals
float all_rot_prev = {0};

float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels

bool pressingPneumatics = false;
// MARK: Init robot
inline Robot init_robot() {
	DrivetrainMotor topLeft(4, false, pros::v5::MotorGearset::blue);    // Reverse
	DrivetrainMotor middleLeft(5, false, pros::v5::MotorGearset::blue); // Reverse
	DrivetrainMotor bottomLeft(6, true, pros::v5::MotorGearset::blue);  // Normal
	DrivetrainMotor topRight(1, true, pros::v5::MotorGearset::blue);    // Reverse
	DrivetrainMotor middleRight(2, true, pros::v5::MotorGearset::blue); // Reverse
	DrivetrainMotor bottomRight(3, false, pros::v5::MotorGearset::blue);// Normal
	std::array<DrivetrainMotor, 6> drivetrain = {
		topLeft,
		middleLeft,
		bottomLeft,
		topRight,
		middleRight,
		bottomRight
	};
	return Robot(drivetrain, 8.639379f, 0.75f, inertial);
}

// MARK: Robot
Robot robot = init_robot();

//TODO: Make robot atomic, pos, and inertial atomimc to prevent race conditions (we have an async method running auton, but that is accessing inertial, pos, and heading)



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

//MARK: Thread
void thread_UpdateOdom() {
	while (running_program.load()) {
		robot.autonController.load().get()->updateHeadingAndOdom();
		wait(FRAME);
	}
}

void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	static bool isPaused = false;
	if (controller.getNewPress(Button::A)) {
		isPaused = !isPaused;
		if (isPaused) {
			robot.drivetrain.brakeWheels();
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
	while (robot.inertial.is_calibrating()) {
		wait(FRAME);
	}
	// MARK: Thread
	odomThread = new pros::Task(thread_UpdateOdom);
}

// MARK-: Disabled
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


//| RUDIMENTARY
void skillsAuton() {
	// Skills auton
	while (robot.inertial.is_calibrating()) {}
	while (robot.pos.y < 12) {
		
		if (robot.drivetrain.leftSpeed < 50) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() + 1, robot.drivetrain.getRightSpeed() + 1);
			robot.drivetrain.leftSpeed += 1;
			robot.drivetrain.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
	while (robot.pos.y < 24) {
		
		if (robot.drivetrain.leftSpeed > 10) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() - 0.85f, robot.drivetrain.getRightSpeed() - 0.85f);
			robot.drivetrain.leftSpeed -= 0.85f;
			robot.drivetrain.rightSpeed -= 0.85f;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(1000);
	robot.drivetrain.moveWheels(0, 0);
	while (robot.pos.y > -24) {
		
		if (robot.drivetrain.leftSpeed > -100) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() - 2, robot.drivetrain.getRightSpeed() - 2);
			robot.drivetrain.leftSpeed -= 2;
			robot.drivetrain.rightSpeed -= 2;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
}

//| RUDIMENTARY
void quickAuton() {
	robot.drivetrain.setBrakeMode(BRAKE_MODE_HOLD);
	// Score 3 points auton
	while (robot.pos.y < 9) {
		if (robot.drivetrain.leftSpeed < 50) {
			robot.drivetrain.leftSpeed += 0.85f;
			robot.drivetrain.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.pos.y < 18) {
		if (robot.drivetrain.leftSpeed > 10) {
			robot.drivetrain.leftSpeed -= 1;
			robot.drivetrain.rightSpeed -= 1;
		}
		robot.drivetrain.moveWheels();
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.inertial.get_heading() < 130) {
		robot.drivetrain.moveWheels(30, -30);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.pos.x < 40 && robot.pos.y > 12) {
		if (robot.drivetrain.leftSpeed < 50) {
			robot.drivetrain.leftSpeed += 1;
			robot.drivetrain.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.inertial.get_heading() > 0) {
		robot.drivetrain.moveWheels(-30, 30);
	}
	while (robot.pos.y < 18) {
		if (robot.drivetrain.leftSpeed < 50) {
			robot.drivetrain.leftSpeed += 1;
			robot.drivetrain.rightSpeed += 1;
		}
	}
	conveyor.move_velocity(200);
	bandRotatorBottom.move_velocity(200);
}

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
	printOnScreen("AUTON!");
	wait(100);
	robot.drivetrain.brakeWheels();
	// Original auton
	// Init
	std::vector<Point> autonPoints = {
		{0, 0},
		{0, 24},
		{12, 12},
		{0, -24}
	};
	constexpr float robotCheckRadius = 10.0f;
	Point target = autonPoints.at(0);
	Point prevPoint = robot.pos;
	Vec2 curTargetLoc = {0, 0};  // The current intersect
	Vec2 prevTargetLoc = {0, 0}; // Last valid intersect
	robot.drivetrain.brakeWheels();
	for (size_t ptIdx = 0; ptIdx < autonPoints.size() - 1; ptIdx++) { // - 1 so we can have an extra point the robot doesn't visit but it aims for
		Point &point = autonPoints.at(ptIdx);
		while (!point.visited) {
			curTargetLoc = robot.autonController.load().get()->getPurePursuitLoc(robotCheckRadius, point.pos);
			prevTargetLoc = curTargetLoc;
			SpeedPair result = robot.autonController.load().get()->getSpeedFromPID_to(point.pos);
			robot.drivetrain.setSpeedFromSpeedPair(result);
			printOnScreen(distanceBetween(robot.pos, point.pos));
			printOnScreen((distanceBetween(robot.pos, point.pos) < 2), 1);
			printOnScreen(std::to_string(point.pos.x) + ", " + std::to_string(point.pos.y), 2);
			// printOnScreen(robot.drivetrain.leftSpeed, 3);
			// printOnScreen(robot.drivetrain.rightSpeed, 4);
			printOnScreen(result.leftSpeed, 4);
			printOnScreen(result.rightSpeed, 5);
			// When within 2 inches of a point, mark that point as visted
			point.visited = (distanceBetween(robot.pos, point.pos) < 2);
			robot.drivetrain.moveWheels();
			checkPauseProgram();
			wait(FRAME);
		}
	}
	printOnScreen("WE DID IT!");
	wait(100000);
}

void brakeScoring(std::vector<pros::Motor*> scoringMotors) {
	for (int i = 0; i < scoringMotors.size(); i++) {
		scoringMotors.at(i)->brake();
	}
}

// MARK: Driving
void drivePipeline() {
	if (controller.getNewPress(Button::X)) {
		robot.drivetrain.swapDriveMode();
	}
	robot.drivetrain.setSpeedFromController(controller);
	robot.drivetrain.moveWheels();
}

// MARK: Scoring
void scorePipeline() {
	float maxElevatorSpeed = 200.0f;
	if (controller.getNewPress(Button::B)) {
		loaderRod.toggle();
	}
	if (controller.getNewPress(Button::A)) {
		descorer.toggle();
	}
	if (controller.getPressing(Button::R2)) {
		conveyor.move_velocity(-200);
		brakeScoring({&intake, &bandRotatorTop, &bandRotatorBottom});
	} else if (controller.getPressing(Button::R1)) {
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(-275);
		intake.move_velocity(200);
		brakeScoring({&bandRotatorBottom});
	} else if (controller.getPressing(Button::L1)) {
		conveyor.move_velocity(200);
		bandRotatorTop.move_velocity(275);
		bandRotatorBottom.move_velocity(200);
		brakeScoring({&intake});
	} else if (controller.getPressing(Button::L2)) {
		conveyor.move_velocity(-200);
		intake.move_velocity(-200);
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		brakeScoring({&bandRotatorTop, &bandRotatorBottom});
	} else {
		brakeScoring({&conveyor, &intake, &bandRotatorTop, &bandRotatorBottom});
	}
}


// MARK: opcontrol
void opcontrol() {
	autonomous();
	// clear_screen();
	// bool motors_overheated = false;
	// while (!motors_overheated) {
	// 	for (DrivetrainMotor* motor : robot.drivetrain.getWheelsAsPtrs()) {
	// 		if (motor->rawMotor.is_over_temp()) {
	// 			motors_overheated = true;
	// 		}
	// 	}
	// 	if (robot.drivetrain.w_topLeft.rawMotor.is_over_temp() || robot.drivetrain.w_topRight.rawMotor.is_over_temp() || robot.drivetrain.w_bottomLeft.rawMotor.is_over_temp() || robot.drivetrain.w_bottomRight.rawMotor.is_over_temp()) break;
	// 	controller.updateInputData();
	// 	// robot.autonController->updateHeadingAndOdom();
	// 	drivePipeline();
	// 	scorePipeline();
	// 	wait(FRAME);
	// }
	controller.rawController.rumble("...");
	// printOnScreen(to_string(robot.pos.y));
	robot.drivetrain.brakeWheels();
	conveyor.brake();
	intake.brake();
	bandRotatorBottom.brake();
	bandRotatorTop.brake();
	running_program.store(false);
	odomThread->remove();
	delete odomThread;
	odomThread = nullptr;
}
