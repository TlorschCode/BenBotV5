/*###|   main.cpp   |###*/

#include "lib/common_includes.h"
#include "pros/misc.h"
#include <fstream>
#include <string>
#include <iostream>
#include <format>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <tuple>
#include <vector>
#include <atomic>


constexpr pros::motor_brake_mode_e BRAKE_MODE_BRAKE = pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE;
constexpr pros::motor_brake_mode_e BRAKE_MODE_HOLD = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
constexpr pros::motor_brake_mode_e BRAKE_MODE_COAST = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
constexpr pros::motor_brake_mode_e BRAKE_MODE_INVALID = pros::motor_brake_mode_e::E_MOTOR_BRAKE_INVALID;

using namespace ctrlAPI;
using namespace robotAPI;


// MARK: Game state

constexpr float COLOR_TOLERANCE = 200.0f;
constexpr float BRIGHTNESS_TOLERANCE = 0.001f;
enum class TeamColor {
	RED,
	BLUE,
	UNKNOWN
};
enum class StartingSide { 
	LEFT,
	RIGHT
};
TeamColor TEAM;
StartingSide STARTING_SIDE;


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
pros::Optical colorChecker(12);
// tri-ports
pros::adi::Pneumatics scraper('A', false);
pros::adi::Pneumatics descorer('B', false);
pros::adi::DigitalIn sideSwitcher('C');
pros::adi::DigitalIn teamSwitcher('D');


// MARK: Globals
float all_rot_prev = {0};

float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels

bool pressingPneumatics = false;
// MARK: Init robot
NODISCARD inline Robot& init_robot() {
	DrivetrainMotor topLeft(4, false, pros::v5::MotorGearset::blue);     // Normal
	DrivetrainMotor middleLeft(5, false, pros::v5::MotorGearset::blue);  // Normal
	DrivetrainMotor bottomLeft(6, true, pros::v5::MotorGearset::blue);   // Reversed
	DrivetrainMotor topRight(1, true, pros::v5::MotorGearset::blue);    // Normal
	DrivetrainMotor middleRight(2, true, pros::v5::MotorGearset::blue); // Normal
	DrivetrainMotor bottomRight(3, false, pros::v5::MotorGearset::blue);  // Reversed
	std::array<DrivetrainMotor, 6> drivetrain = {
		topLeft,
		middleLeft,
		bottomLeft,
		topRight,
		middleRight,
		bottomRight
	};
	Robot::Init(drivetrain, 8.639379f, 0.75f, inertial);
	return Robot::Get();
}

// MARK: Robot
Robot& robot = init_robot();

//TODO: Make robot atomic, pos, and inertial atomimc to prevent race conditions (we have an async method running auton, but that is accessing inertial, pos, and heading)



//| NON-DEFAULT FUNCTIONS |//
// MARK: Utils


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
		robot.autonController->updateHeadingAndOdom();
		wait(FRAME);
	}
}

void startOdomThread() {
	// odomThread = new pros::Task(thread_UpdateOdom);
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




// MARK: Helper funcs

NODISCARD TeamColor _getTeamColorViewed() {
	static TeamColor lastColor = TeamColor::UNKNOWN;
	auto rgb = colorChecker.get_rgb();
	if (colorChecker.get_brightness() < BRIGHTNESS_TOLERANCE) return lastColor;
	if (rgb.blue - rgb.red > COLOR_TOLERANCE) {
		return lastColor = TeamColor::BLUE;
	}
	if (rgb.red - rgb.blue > COLOR_TOLERANCE) {
		return lastColor = TeamColor::RED;
	}
	return lastColor;
}

void _update() {
	robot.autonController.get()->updateHeadingAndOdom();
}

void brakeScoring(std::vector<pros::Motor*> scoringMotors) {
	for (int i = 0; i < scoringMotors.size(); i++) {
		scoringMotors.at(i)->brake();
	}
}

void _scoreBalls() {
	conveyor.move_velocity(200);
	if (colorChecker.get_saturation() > BRIGHTNESS_TOLERANCE) {
		if (TEAM == _getTeamColorViewed()) {
			bandRotatorTop.move_velocity(275); // score out
		} else {
			bandRotatorTop.move_velocity(-275); // suck back in so it doesn't get scored
		}
	}
	bandRotatorBottom.move_velocity(200);
	brakeScoring({&intake});
}

void _suckBalls() {
	conveyor.move_velocity(200);
	bandRotatorTop.move_velocity(-275);
	intake.move_velocity(200);
	brakeScoring({&bandRotatorBottom});
}

void _spitOutBalls() {
	conveyor.move_velocity(200);
	bandRotatorTop.move_velocity(-275);
	intake.move_velocity(200);
	brakeScoring({&bandRotatorBottom});
}

void stepRobotSpeedTo(float targ, float amount) {
    float& left  = robot.drivetrain.speed.leftSpeed;
    float& right = robot.drivetrain.speed.rightSpeed;

    float step = std::abs(amount);

    float deltaL = targ - left;
    float deltaR = targ - right;

    left  = (std::abs(deltaL) <= step)
        ? targ
        : left  + sign(deltaL) * step;

    right = (std::abs(deltaR) <= step)
        ? targ
        : right + sign(deltaR) * step;

    robot.drivetrain.moveWheels();
	_update();
	wait(FRAME);
}

void stepRobotSpeedTo(float targ_l, float targ_r, float amount_l, float amount_r) {
    float& cur_l = robot.drivetrain.speed.leftSpeed;
    float& cur_r = robot.drivetrain.speed.rightSpeed;

    float delta_l = targ_l - cur_l;
    float delta_r = targ_r - cur_r;

    float step_l = std::abs(amount_l);
    float step_r = std::abs(amount_r);

    cur_l = (std::abs(delta_l) <= step_l)
        ? targ_l
        : cur_l + sign(delta_l) * step_l;

    cur_r = (std::abs(delta_r) <= step_r)
        ? targ_r
        : cur_r + sign(delta_r) * step_r;

    robot.drivetrain.moveWheels();
    _update();
    wait(FRAME);
}

void stepRobotSpeedToBalanced(float targ_l, float targ_r, float amount) {
    float& cur_l = robot.drivetrain.speed.leftSpeed;
    float& cur_r = robot.drivetrain.speed.rightSpeed;
    const float delta_l = targ_l - cur_l;
    const float delta_r = targ_r - cur_r;
    const float abs_l = std::abs(delta_l);
    const float abs_r = std::abs(delta_r);
    const float base = std::abs(amount);

    float step_l = 0.0f;
    float step_r = 0.0f;

    if (abs_l >= abs_r) {
        step_l = base;
        step_r = (abs_l > 0.0f) ? base * (abs_r / abs_l) : 0.0f;
    }
    else {
        step_r = base;
        step_l = (abs_r > 0.0f) ? base * (abs_l / abs_r) : 0.0f;
    }

    cur_l = (abs_l <= step_l)
        ? targ_l
        : cur_l + sign(delta_l) * step_l;

    cur_r = (abs_r <= step_r)
        ? targ_r
        : cur_r + sign(delta_r) * step_r;

    robot.drivetrain.moveWheels();
    _update();
    wait(FRAME);
}

void stepRobotSpeedToBalanced(float targ, float amount) {
    float& cur_l = robot.drivetrain.speed.leftSpeed;
    float& cur_r = robot.drivetrain.speed.rightSpeed;

    const float delta_l = targ - cur_l;
    const float delta_r = targ - cur_r;

    const float abs_l = std::abs(delta_l);
    const float abs_r = std::abs(delta_r);

    const float base = std::abs(amount);

    float step_l = 0.0f;
    float step_r = 0.0f;

    if (abs_l >= abs_r) {
        step_l = base;
        step_r = (abs_l > 0.0f) ? base * (abs_r / abs_l) : 0.0f;
    }
    else {
        step_r = base;
        step_l = (abs_r > 0.0f) ? base * (abs_l / abs_r) : 0.0f;
    }

    cur_l = (abs_l <= step_l)
        ? targ
        : cur_l + sign(delta_l) * step_l;

    cur_r = (abs_r <= step_r)
        ? targ
        : cur_r + sign(delta_r) * step_r;

    robot.drivetrain.moveWheels();
    _update();
    wait(FRAME);
}

void _stopRobot() {
	while (robot.drivetrain.speed.rightSpeed + robot.drivetrain.speed.leftSpeed > ELIPSON_FLOAT * 2) {
		stepRobotSpeedToBalanced(0, 2);
	}
}

void accelerateRobotSpeedTo(float targ, float amount, float threshold=ELIPSON_FLOAT) {
	bool leftEquals = isEqual(robot.drivetrain.speed.leftSpeed, targ); 
	bool rightEquals = isEqual(robot.drivetrain.speed.rightSpeed, targ);
	while (!(leftEquals && rightEquals)) {
		stepRobotSpeedTo(targ, amount);
	}
}

void accelerateRobotSpeedTo(float targ_l, float targ_r, float amount_l, float amount_r, float threshold=ELIPSON_FLOAT) {
	bool leftEquals = isEqual(robot.drivetrain.speed.leftSpeed, targ_l); 
	bool rightEquals = isEqual(robot.drivetrain.speed.rightSpeed, targ_r);
	while (!(leftEquals && rightEquals)) {
		stepRobotSpeedTo(targ_l, targ_r, amount_l, amount_r);
	}
}

void accelerateRobotSpeedToRel(float targ_l, float targ_r, float amount, float threshold=ELIPSON_FLOAT) {
	bool leftEquals = isEqual(robot.drivetrain.speed.leftSpeed, targ_l); 
	bool rightEquals = isEqual(robot.drivetrain.speed.rightSpeed, targ_r);
	while (!(leftEquals && rightEquals)) {
		stepRobotSpeedToBalanced(targ_l, targ_r, amount);
	}
}



// MARK: Initialize
//| DEFAULT FUNCTIONS |//
void initialize() {
	configureMotors();
	pros::lcd::initialize();
	robot.inertial.reset();
	while (robot.inertial.is_calibrating()) {
		wait(FRAME);
	}
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

void startColorAndSideSelection() {
	TEAM = TeamColor::BLUE;
	STARTING_SIDE = StartingSide::RIGHT; // TODO: Change to left for consistency's sake
	printOnScreen("Robot starting side is: Right Side", 1); // TODO: Change to left for consistency's sake
	wait(FRAME);
	printOnScreen("Robot team is: Blue", 2);

	while (true) {
		controller.updateInputData();
		if (sideSwitcher.get_value()) {
			STARTING_SIDE = (STARTING_SIDE == StartingSide::LEFT) ? StartingSide::RIGHT : StartingSide::LEFT;
			printOnScreen(std::format("Robot starting side set to: {}", (STARTING_SIDE == StartingSide::LEFT) ? "Left Side" : "Right Side", 1));
			while (sideSwitcher.get_value()) {
				wait(FRAME);
			}
		}

		if (teamSwitcher.get_value()) {
			TEAM = (TEAM == TeamColor::BLUE) ? TeamColor::RED : TeamColor::BLUE;
			printOnScreen(std::format("Robot team set to: {}", (TEAM == TeamColor::BLUE) ? "Blue" : "Red"), 2);
			while (teamSwitcher.get_value()) {
				wait(FRAME);
			}
		}

		while (controller.getPressing(Button::X)) {
			controller.rawController.rumble("...");
			return;
		}
		wait(FRAME);
	}
}

void competition_initialize() {
	printOnScreen("STARTING");
	startColorAndSideSelection();
}


//| RUDIMENTARY
void skillsAuton() {
	// Skills auton
	while (robot.inertial.is_calibrating()) {}
	while (robot.pos.y < 12) {
		
		if (robot.drivetrain.speed.leftSpeed < 50) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() + 1, robot.drivetrain.getRightSpeed() + 1);
			robot.drivetrain.speed.leftSpeed += 1;
			robot.drivetrain.speed.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
	while (robot.pos.y < 24) {
		
		if (robot.drivetrain.speed.leftSpeed > 10) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() - 0.85f, robot.drivetrain.getRightSpeed() - 0.85f);
			robot.drivetrain.speed.leftSpeed -= 0.85f;
			robot.drivetrain.speed.rightSpeed -= 0.85f;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(1000);
	robot.drivetrain.moveWheels(0, 0);
	while (robot.pos.y > -24) {
		
		if (robot.drivetrain.speed.leftSpeed > -100) {
			// robot.drivetrain.moveWheels(robot.drivetrain.getLeftSpeed() - 2, robot.drivetrain.getRightSpeed() - 2);
			robot.drivetrain.speed.leftSpeed -= 2;
			robot.drivetrain.speed.rightSpeed -= 2;
		}
		robot.drivetrain.moveWheels();
		wait(FRAME);
	}
}

//| RUDIMENTARY
void autonPark() {
	robot.drivetrain.setBrakeMode(BRAKE_MODE_HOLD);
	// Score 3 points auton
	while (robot.pos.y < 9) {
		if (robot.drivetrain.speed.leftSpeed < 50) {
			robot.drivetrain.speed.leftSpeed += 0.85f;
			robot.drivetrain.speed.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.pos.y < 18) {
		if (robot.drivetrain.speed.leftSpeed > 10) {
			robot.drivetrain.speed.leftSpeed -= 1;
			robot.drivetrain.speed.rightSpeed -= 1;
		}
		robot.drivetrain.moveWheels();
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.inertial.get_heading() < 130) {
		robot.drivetrain.moveWheels(30, -30);
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.pos.x < 40 && robot.pos.y > 12) {
		if (robot.drivetrain.speed.leftSpeed < 50) {
			robot.drivetrain.speed.leftSpeed += 1;
			robot.drivetrain.speed.rightSpeed += 1;
		}
		robot.drivetrain.moveWheels();
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.inertial.get_heading() > 0) {
		robot.drivetrain.moveWheels(-30, 30);
		_update();
		wait(FRAME);
	}
	while (robot.pos.y < 18) {
		if (robot.drivetrain.speed.leftSpeed < 50) {
			robot.drivetrain.speed.leftSpeed += 1;
			robot.drivetrain.speed.rightSpeed += 1;
			_update();
			wait(FRAME);
		}
		robot.drivetrain.moveWheels();
	}
	conveyor.move_velocity(200);
	bandRotatorBottom.move_velocity(200);
}



void simple_auton() {
	// while (true) {
	// 	_update();
	// 	printOnScreen(robot.heading);
	// 	wait(FRAME);
	// }
	int loopCount = 0;
	printOnScreen("BEGINNING SIMPLE AUTON");
	while (robot.pos.y < 24) {
		if (robot.pos.y > 17) {
			stepRobotSpeedTo(25, 1);
		} else {
			stepRobotSpeedTo(50, 1);
		}
	}
	_stopRobot();
	wait(100);
	while (robot.heading < 5) {
		stepRobotSpeedToBalanced(-20, 20, 2);
	}
	robot.drivetrain.brakeWheels();
	_suckBalls();
	wait(100);
	while (robot.pos.y < 31) {
		if (robot.heading < 9) {
			stepRobotSpeedToBalanced(20, -20, 2);
		}
		stepRobotSpeedToBalanced(20, 1);
	}
	while (robot.pos.y < 48) {
		if (robot.pos.y < 38) {
			stepRobotSpeedToBalanced(10, 1); // Pick up balls
		} else {
			stepRobotSpeedToBalanced(10, 1); // Pick up balls
		}
	}
	while (robot.pos.y > 44) {
		stepRobotSpeedToBalanced(-20, 1);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.heading > -34) { // point to lower goal
		if (robot.heading < -15) {
		}
		stepRobotSpeedTo(10, -10, 1, 1);
	}
	controller.rawController.rumble(".");
	brakeScoring({&conveyor, &intake, &bandRotatorTop, &bandRotatorBottom});
	while (robot.pos.y < 51) { // drive to lower goal
		stepRobotSpeedTo(20, 2);
	}
	brakeScoring({&conveyor});
	_update();
	robot.drivetrain.brakeWheels();
	wait(100);
	scraper.toggle();
	wait(300);
	scraper.toggle(); // place ball into bottom
	// while (robot.pos.y < 51) {
	// 	stepRobotSpeedTo(50, 5);
	// }
	_update();
	robot.drivetrain.brakeWheels();
	wait(100);
	brakeScoring({&conveyor});
	// robot.drivetrain.moveWheels(-100, -100);
	// wait(100'000);
	while (robot.pos.y > 20) {
		if (robot.pos.y > 30) {
			stepRobotSpeedToBalanced(-50, -50, 2); // back up
		} else {
			stepRobotSpeedToBalanced(-25, -25, 2); // back up
		}
	}
	robot.drivetrain.brakeWheels();
	controller.rawController.rumble("..."); // check to make sure it's not tracking the abs() of the wheel positions and thinking the robot is going forward
	while (robot.heading < 0) {
		stepRobotSpeedToBalanced(-10, 10, 2);
	}
	robot.drivetrain.brakeWheels();
	robot.drivetrain.moveWheels(25, 25);
	wait(1500);
	robot.drivetrain.brakeWheels();
	_scoreBalls();
	robot.drivetrain.setBrakeMode(BRAKE_MODE_COAST);
	wait(5000);
	brakeScoring({&intake, &conveyor, &bandRotatorTop});
	printOnScreen("DONE!");
}

void skills_auton() {
	const Vec2 rightHomeGoalPos = {63, 3};
	printOnScreen("BEGINNING SKILLS AUTON");
	//| Pick up balls
	while (robot.pos.y < 25) {
		robot.drivetrain.moveWheels(30, 30);
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	wait(100);
	while (robot.heading < 5) {
		robot.drivetrain.moveWheels(10, -10);
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	_suckBalls();
	wait(100);
	while (robot.pos.y < 31) {
		if (robot.heading < 9) {
			robot.drivetrain.moveWheels(20, 5);
		}
		robot.drivetrain.moveWheels(20, 20);
		_update();
		wait(FRAME);
	}
	while (robot.pos.y < 46) {
		robot.drivetrain.moveWheels(10, 10);
		_update();
		wait(FRAME);
	}
	wait(100);
	float targ_heading = degreesTill(robot.pos, {rightHomeGoalPos.x, 10});
	while (robot.heading < targ_heading) {
		robot.drivetrain.moveWheels(30, -30);
		_update(); 
		wait(FRAME);
	}
	brakeScoring({&intake, &conveyor});
	// scraper.toggle();
	float dist_till;
	while (robot.pos.x < rightHomeGoalPos.x) {
		dist_till = degreesTill(robot.pos, rightHomeGoalPos);
		if (std::abs(dist_till) > 5) {
			robot.drivetrain.moveWheels(dist_till * 0.3f * sign(dist_till), dist_till * -0.3f * sign(dist_till));
		} else {
			robot.drivetrain.moveWheels(30, 30);
		}
		printOnScreen(dist_till);
		_update();
		wait(FRAME);
	}
	robot.drivetrain.brakeWheels();
	printOnScreen("DONE");
	wait(100000);
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
	if (STARTING_SIDE == StartingSide::RIGHT) {
		simple_auton();
	} else {
		// nothing yet
	}

	{ //# REAL AUTON
	
	// robot.autonController.get()->setPIDposVal(PID_P, {0, 0.3f});
	// robot.autonController.get()->setPIDposVal(PID_I, {0, 0.01f});
	// robot.autonController.get()->setPIDposVal(PID_D, {0, 0.01f});

	// robot.autonController.get()->setPIDrotVal(PID_P, {0, 0.3f});
	// robot.autonController.get()->setPIDrotVal(PID_I, {0, 0.01f});
	// robot.autonController.get()->setPIDrotVal(PID_D, {0, 0.01f});

	// printOnScreen("AUTON!");
	// wait(FRAME);
	// robot.drivetrain.brakeWheels();
	// // Original auton
	// // Init
	// std::vector<Point> autonPoints = {
	// 	{0, 0},
	// 	{0, 24},
	// 	{-24, -24},
	// 	{0, -24}
	// };

	// printOnScreen("ABOUT TO BEGIN DRIVE");
	// wait(FRAME);
	// printOnScreen("BEGINNING DRIVE");
	// wait(FRAME);
	// autonPoints = robot.autonController->driveAlongPath(std::move(autonPoints), PID_P);
	// printOnScreen("WE DID IT!");
	// wait(100'000);
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
	static int x_counter = 0;
	static uint32_t x_timer = 0;

	// max elevator speed is 200.0f;
	
	if (controller.getNewPress(Button::X)) {
		x_counter++;
		x_timer--;
		
		if (x_counter >= 3) {
			x_timer = 800;
			x_counter = 0;
			TEAM = (TEAM == TeamColor::BLUE) ? TeamColor::RED : TeamColor::BLUE;
			controller.rawController.rumble("..");
		}

		if (x_timer <= 0) {
			x_timer = 0;
			x_counter = 0;
		}
	}
	if (controller.getNewPress(Button::B)) {
		scraper.toggle();
	}
	if (controller.getNewPress(Button::A)) {
		descorer.toggle();
	}
	if (controller.getPressing(Button::R2)) {
		conveyor.move_velocity(-200);
		brakeScoring({&intake, &bandRotatorTop, &bandRotatorBottom});
	} else if (controller.getPressing(Button::R1)) { // suck in
		_suckBalls();
	} else if (controller.getPressing(Button::L1)) { // score out
		_scoreBalls();
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
	// startColorAndSideSelection();
	// autonomous();
	controller.rawController.rumble("...");
	robot.drivetrain.setBrakeMode(BRAKE_MODE_COAST);
	clear_screen();
	bool motors_overheated = false;
	colorChecker.set_led_pwm(100);
	while (!motors_overheated) {
		for (DrivetrainMotor* motor : robot.drivetrain.getWheelsAsPtrs()) {
			if (motor->rawMotor.is_over_temp()) {
				motors_overheated = true;
			}
		}
		if (robot.drivetrain.w_topLeft.rawMotor.is_over_temp() || robot.drivetrain.w_topRight.rawMotor.is_over_temp() || robot.drivetrain.w_bottomLeft.rawMotor.is_over_temp() || robot.drivetrain.w_bottomRight.rawMotor.is_over_temp()) break;
		controller.updateInputData();
		// robot.autonController->updateHeadingAndOdom();
		drivePipeline();
		scorePipeline();
		auto rgb = colorChecker.get_rgb();
		// if (_getTeamColorViewed() == TeamColor::UNKNOWN) controller.rawController.rumble("...");
		printOnScreen(rgb.red);
		printOnScreen(rgb.green, 1);
		printOnScreen(rgb.blue, 2);
		printOnScreen(colorChecker.get_saturation(), 3);
		printOnScreen(rgb.brightness, 4);
		wait(FRAME);
	}
	controller.rawController.rumble("...");
	robot.drivetrain.brakeWheels();
	conveyor.brake();
	intake.brake();
	bandRotatorBottom.brake();
	bandRotatorTop.brake();
	running_program.store(false);
	odomThread->remove();
	// delete odomThread;
	odomThread = nullptr;
}
