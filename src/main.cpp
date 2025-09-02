#include "main.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor topLeft(1, pros::v5::MotorGearset::blue);
pros::Motor bottomLeft(2, pros::v5::MotorGearset::blue);
pros::Motor topRight(3, pros::v5::MotorGearset::blue);
pros::Motor bottomRight(4, pros::v5::MotorGearset::blue);
pros::Motor conveyor(5, pros::v5::MotorGearset::green);
pros::Motor bandRotator(6, pros::v5::MotorGearset::green);

pros::Imu inertial(5);
void configureMotors() {
	topLeft.set_reversed(true);
	bottomLeft.set_reversed(true);
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

const long double PI = 3.14159265358979323846;

float posX = {0};
float posY = {0};
const float gear_ratio = {0.5f}; // Wheel-motor gear ratio
const float wheel_circumference = {12.56f};
const float frame = {100.0f / 1000.0f}; // Frame time
float all_rot_prev = {0};

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

void moveWheels(float speedleft, float speedright) {
	topLeft.move_velocity(speedleft);
	bottomLeft.move_velocity(speedleft);
	topRight.move_velocity(speedright);
	bottomRight.move_velocity(speedright);
}

void brakeWheels() {
	topLeft.brake();
	bottomLeft.brake();
	topRight.brake();
	bottomRight.brake();
}

void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		brakeWheels();
		while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			wait(10);
		}
	}
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

void drivePipeline() {
	float left = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 2) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 1.5f);
	float right = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 2) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 1.5f);
	moveWheels(left, right);
	trackPosition();
	println(posX);
	println(posY, 2);
}

void scorePipeline() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		conveyor.move_velocity(200);
		bandRotator.set_reversed(false);
		bandRotator.move_velocity(200);
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
		conveyor.move_velocity(200);
		bandRotator.set_reversed(true);
		bandRotator.move_velocity(200);
	} else {
		conveyor.brake();
		bandRotator.brake();
	}
}

void opcontrol() {
	// 72 inches across the field
	clear_screen();
	wait(1000);
	// while (posY < (144 - (16.5f * 2))) {
	// 	float left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2);
	// 	float right = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2);
	// 	moveWheels(left, right);
	// 	trackPosition();
	// 	println(posX);
	// 	println(posY, 2);
	// 	wait(10);
	// }
	while (true) {
		drivePipeline();
		scorePipeline();
		wait(frame);
	}
	controller.rumble("-");
	brakeWheels();
}
