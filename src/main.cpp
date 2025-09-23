#include "main.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>

// MARK: Hardware Init
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor topLeft(1, pros::v5::MotorGearset::blue);
pros::Motor bottomLeft(2, pros::v5::MotorGearset::blue);
pros::Motor topRight(3, pros::v5::MotorGearset::blue);
pros::Motor bottomRight(4, pros::v5::MotorGearset::blue);
pros::Motor conveyor(5, pros::v5::MotorGearset::green);
pros::Motor bandRotatorTop(6, pros::v5::MotorGearset::blue);
pros::Motor bandRotatorBottom(7, pros::v5::MotorGearset::green);
pros::Motor intake(8, pros::v5::MotorGearset::green);

pros::Imu inertial(5);
void configureMotors() {
	topLeft.set_reversed(true);
	bottomLeft.set_reversed(true);
    conveyor.set_reversed(true);
	intake.set_reversed(true);
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

// MARK: Variables
const long double PI = 3.14159265358979323846;

float posX = {0};
float posY = {0};
const float gear_ratio = {0.5f}; // Wheel-motor gear ratio
const float wheel_circumference = {12.56f};
const float frame = {100.0f / 1000.0f}; // Frame time
float all_rot_prev = {0};
enum Button {R1, L1, UP, DOWN, NONE};
// enum ControllerButton {A, B, C, D, E,}
float analog_left_x = {};
float analog_left_y = {};
float analog_right_x = {};
float analog_right_y = {};

// MARK: Templates
void brakeWheels();
void checkPauseProgram();
void updateControllerData();
void moveWheels(float speedLeft, float speedRight);






// MARK: Utilities
//| NON-DEFAULT FUNCTIONS |//
inline double toRadians(float degrees) {
	return degrees * (PI / 180);
}

inline double toDegrees(double radians) {
	return radians * (180 / PI);
}

inline double truncate(double num, int cutoff = 2) {
	return floor(num * pow(10, cutoff)) / pow(10, cutoff);
}

inline int sign(float input) {
	return (input >= 0) ? 1 : -1;
}

inline double map_value(float input, float input_start, float input_end, float output_start, double output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

void wait(float time) {
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

template <typename T>
void println(const T& input, int row = 1) {
    std::string printtext;
    if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char*>) {
        // Handle string and C-style string types
        printtext = input;
    } else {
        // Handle other types using stringstream
        std::stringstream ss;
        ss << input;
        printtext = ss.str();
    }
    pros::lcd::set_text(row, printtext);
}

//| RUNTIME FUNCTIONS:
void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		brakeWheels();
		while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			wait(10);
		}
	}
}

// MARK: Move Funcs
//| MOVEMENT FUNCTIONS

// Moves drivetrain wheels
void moveWheels(float speedLeft, float speedRight) {
	topLeft.move_velocity(speedLeft * 600);
	bottomLeft.move_velocity(speedLeft * 600);
	topRight.move_velocity(speedRight * 600);
	bottomRight.move_velocity(speedRight * 600);
}

void brakeWheels() {
	topLeft.brake();
	bottomLeft.brake();
	topRight.brake();
	bottomRight.brake();
}

void trackPosition() {
	std::uint32_t now = pros::millis();
	float heading = truncate(inertial.get_heading());
	float left_pos = (topLeft.get_raw_position(&now) + bottomLeft.get_raw_position(&now)) / 2;
	float right_pos = (topRight.get_raw_position(&now) + bottomRight.get_raw_position(&now)) / 2;
	float all_rot_now = (left_pos + right_pos) / 2;
	float all_rot_delta = all_rot_now - all_rot_prev;
	posX += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * sin(toRadians(heading));
	posY += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * cos(toRadians(heading));
	all_rot_prev = all_rot_now;
}

//| DEFAULT FUNCTIONS |//
void initialize() {
	configureMotors();
	pros::lcd::initialize();
	inertial.reset();
	wait(2300);
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
void autonomous() {

}

// MARK: Driving
void drivePipeline(float driveSpeed) {
	// Controller analog is -1 to 1
	float left_speed = (analog_left_y * (driveSpeed / 100)) + (analog_left_x * (driveSpeed / 150));
	float right_speed = (analog_left_y * (driveSpeed / 100)) - (analog_left_x * (driveSpeed / 150));
	moveWheels(std::min(left_speed * 600, 600.0f), std::min(right_speed * 600, 600.0f));
	// trackPosition();
	println(posX);
	println(posY, 2);
}

// MARK: Scoring
void scorePipeline(float *driveSpeed, float *elevatorSpeed) {
	float maxElevatorSpeed = 200.0f;
	Button first_pressed = NONE;
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) and first_pressed != DOWN) {
		first_pressed = UP;
		*driveSpeed += 10;
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) and first_pressed != UP) {
		first_pressed = DOWN;
		*driveSpeed -= 10;
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		conveyor.move_velocity(*elevatorSpeed);
		intake.brake();
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		*elevatorSpeed -= 3;
		*elevatorSpeed = std::max(*elevatorSpeed, -1.0f * maxElevatorSpeed);
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) and first_pressed != L1) {
		first_pressed = R1;
		conveyor.move_velocity(*elevatorSpeed);
		bandRotatorTop.set_reversed(true);
		bandRotatorTop.move_velocity(275);
		intake.move_velocity(200);
		bandRotatorBottom.brake();
		*elevatorSpeed += 3;
		*elevatorSpeed = std::min(*elevatorSpeed, maxElevatorSpeed);
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) and first_pressed != R1) {
		first_pressed = L1;
		conveyor.move_velocity(*elevatorSpeed);
		bandRotatorTop.set_reversed(false);
		bandRotatorTop.move_velocity(275);
		bandRotatorBottom.move_velocity(200);
		intake.brake();
		*elevatorSpeed += 3;
		*elevatorSpeed = std::min(*elevatorSpeed, maxElevatorSpeed);
	} else {
		*elevatorSpeed = 0;
		first_pressed = NONE;
		conveyor.brake();
		bandRotatorTop.brake();
		intake.brake();
		bandRotatorBottom.brake();
	}
}

void updateControllerData() {
	analog_left_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127);
	analog_left_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127);
	analog_right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127);
	analog_right_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127);
}

// MARK: opcontrol
void opcontrol() {
	// 72 inches across the field
	float drive_speed = 100.0f;
	float elevator_speed = 100.0f;
	clear_screen();
	wait(1000);
	while (true) {
		updateControllerData();
		drivePipeline(drive_speed);
		scorePipeline(&drive_speed, &elevator_speed);
		wait(frame);
	}
	controller.rumble("-");
	brakeWheels();
}
