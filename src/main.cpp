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

constexpr pros::motor_brake_mode_e BRAKE_MODE_BRAKE = pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE;
constexpr pros::motor_brake_mode_e BRAKE_MODE_HOLD = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
constexpr pros::motor_brake_mode_e BRAKE_MODE_COAST = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
constexpr pros::motor_brake_mode_e BRAKE_MODE_INVALID = pros::motor_brake_mode_e::E_MOTOR_BRAKE_INVALID;

using namespace std;
using namespace ctrlAPI;
using namespace robotAPI;

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
Robot init_robot() {
	DrivetrainMotor topLeft(4, false, pros::v5::MotorGearset::blue);    // Reverse
	DrivetrainMotor middleLeft(5, false, pros::v5::MotorGearset::blue); // Reverse
	DrivetrainMotor bottomLeft(6, true, pros::v5::MotorGearset::blue);  // Normal
	DrivetrainMotor topRight(1, true, pros::v5::MotorGearset::blue);    // Reverse
	DrivetrainMotor middleRight(2, true, pros::v5::MotorGearset::blue); // Reverse
	DrivetrainMotor bottomRight(3, false, pros::v5::MotorGearset::blue);// Normal
	array<DrivetrainMotor, 6> drivetrain = {
		topLeft,
		middleLeft,
		bottomLeft,
		topRight,
		middleRight,
		bottomRight
	};
	return Robot(drivetrain, 8.639379f, 0.75f, inertial);
}

Robot robot = init_robot();
vector<Point> autonPoints = {
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
	robot.drivetrain.setBrakeMode(BRAKE_MODE_HOLD);
	while (true) {
		controller.updateInputData();
		checkPauseProgram();
		robot.drivetrain.rightSpeed = controller.leftAnalogYPercent;
		robot.drivetrain.leftSpeed = controller.leftAnalogYPercent;
		robot.drivetrain.moveWheels();
		printOnScreen(robot.pos.y);
		robot.autonController->updateOdom();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	while (true) {
		printOnScreen(robot.pos.y);
		wait(FRAME);
	}

	constexpr float robotCheckRadius = 10.0f;
	Point target = autonPoints.at(0);
	Point prevPoint = robot.pos;
	Vec2 curTargetLoc = {0, 0};
	Vec2 prevTargetLoc = {0, 0};
	wait(12000);
	robot.drivetrain.moveWheels();
	wait(1000);
	robot.drivetrain.brakeWheels();
	for (size_t ptIdx = 0; ptIdx < autonPoints.size(); ptIdx++) {
		Point &point = autonPoints.at(ptIdx);
		while (!point.visited) {
			curTargetLoc = robot.autonController->getPurePursuitLoc(robotCheckRadius, point.pos, prevTargetLoc);
			// robot.pidController.update(curTargetLoc, robot);
		}
	}
}

void brakeScoring(vector<pros::Motor*> scoringMotors) {
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
	clear_screen();
	initialize();
	autonomous();
	while (true) {
		if (robot.drivetrain.topLeft.rawMotor.is_over_temp() || robot.drivetrain.topRight.rawMotor.is_over_temp() || robot.drivetrain.bottomLeft.rawMotor.is_over_temp() || robot.drivetrain.bottomRight.rawMotor.is_over_temp()) break;
		controller.updateInputData();
		// robot.autonController->updateOdom();
		drivePipeline();
		scorePipeline();
		wait(FRAME);
	}
	controller.rawController.rumble("...");
	printOnScreen(to_string(robot.pos.y));
	robot.drivetrain.brakeWheels();
	conveyor.brake();
	intake.brake();
	bandRotatorBottom.brake();
	bandRotatorTop.brake();
}
