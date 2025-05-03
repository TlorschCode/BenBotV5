#include "main.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor topLeft(2, pros::v5::MotorGearset::blue);
pros::Motor bottomLeft(4, pros::v5::MotorGearset::blue);
pros::Motor topRight(1, pros::v5::MotorGearset::blue);
pros::Motor bottomRight(3,pros::v5::MotorGearset::blue);
pros::Imu inertial(5);

const long double PI = 3.14159265358979323846;

float posX = {0};
float posY = {0};
float gear_ratio = {0.5f};
float wheel_circumference = {12.56f};
float time_amount = {60.0f / 0.01f};
float prev_all = {0};

//| NON-DEFAULT FUNCTIONS |//
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

double to_radians(float degrees) {
	return degrees * (PI / 180);
}

float to_degrees(double radians) {
	return radians * (180 / PI);
}

double truncate(double num, int cutoff = 2) {
	return floor(num * pow(10, cutoff)) / pow(10, cutoff);
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

void move_motors(float speedleft, float speedright) {
	topLeft.move_velocity(speedleft);
	bottomLeft.move_velocity(speedleft);
	topRight.move_velocity(speedright);
	bottomRight.move_velocity(speedright);
}

void brake_wheels() {
	topLeft.brake();
	bottomLeft.brake();
	topRight.brake();
	bottomRight.brake();
}

int sign(float input) {
	if (input >= 0) {
		return 1;
	} else {
		return -1;
	}
}

double map_value(float input, float input_start, float input_end, float output_start, double output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

void check_pause_program() { /// REMOVE THIS FUNCTION
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		brake_wheels();
		while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			wait(10);
		}
	}
}

void track_position() {
	std::uint32_t now = pros::millis();
	float heading = truncate(inertial.get_heading());
	float left_pos = (topLeft.get_raw_position(&now) + bottomLeft.get_raw_position(&now)) / 2;
	float right_pos = (topRight.get_raw_position(&now) + bottomRight.get_raw_position(&now)) / 2;
	float all_now = (left_pos + right_pos) / 2;
	float delta_all = all_now - prev_all;
	posX += ((delta_all / 360) * gear_ratio * wheel_circumference) * sin(to_radians(heading));
	posY += ((delta_all / 360) * gear_ratio * wheel_circumference) * cos(to_radians(heading));
	prev_all = all_now;
}

//| DEFAULT FUNCTIONS |//
void initialize() {
	pros::lcd::initialize();
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topLeft.set_reversed(true);
	bottomLeft.set_reversed(true);
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

void opcontrol() {
	// 72 inches across the field
	clear_screen();
	wait(1000);
	while (posY < (144 - (16.5f * 2))) {
		float left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2);
		float right = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2);
		move_motors(left, right);
		track_position();
		println(posX);
		println(posY, 2);
		wait(10);
	}
	controller.rumble("-");
	brake_wheels();
}
