#include "main.h"
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>

using namespace std;

#define ControllerAnalogRightX pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ControllerAnalogRightY pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ControllerAnalogLeftX pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ControllerAnalogLeftY pros::E_CONTROLLER_ANALOG_RIGHT_Y

#define ControllerDigitalA pros::E_CONTROLLER_DIGITAL_A
#define ControllerDigitalB pros::E_CONTROLLER_DIGITAL_B
#define ControllerDigitalX pros::E_CONTROLLER_DIGITAL_X
#define ControllerDigitalY pros::E_CONTROLLER_DIGITAL_Y
#define ControllerDigitalUp pros::E_CONTROLLER_DIGITAL_UP
#define ControllerDigitalDown pros::E_CONTROLLER_DIGITAL_DOWN
#define ControllerDigitalLeft pros::E_CONTROLLER_DIGITAL_LEFT
#define ControllerDigitalRight pros::E_CONTROLLER_DIGITAL_RIGHT
#define ControllerDigitalR1 pros::E_CONTROLLER_DIGITAL_R1
#define ControllerDigitalR2 pros::E_CONTROLLER_DIGITAL_R2
#define ControllerDigitalL1 pros::E_CONTROLLER_DIGITAL_L1
#define ControllerDigitalL2 pros::E_CONTROLLER_DIGITAL_L2


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
pros::Motor agitator(9, pros::v5::MotorGearset::green);

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
const float gear_ratio = {0.5f}; // Wheel-motor gear ratio
const float wheel_circumference = {12.56f};
const float frame = {100.0f / 1000.0f}; // Frame time

Vector2 pos = {0, 0};
float all_rot_prev = {0};
float rightAnalogX = {0};
float rightAnalogY = {0};
float leftAnalogX = {0};
float leftAnalogY = {0};
float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels

vector<Point> autonPoints = {
	{0, 0},
	{0, 24},
	{12, 12}
};

uint16_t buttons = {0};
DrivingMode drivingMode = SINGLE_JOYSTICK;

// MARK: Templates
void brakeWheels();
void checkPauseProgram();
void updateControllerData();
void moveWheels(float speedLeft, float speedRight);

enum Button : uint16_t {
	None = 0,
	A     = 1 << 0,
	B     = 1 << 1,
	X     = 1 << 2,
	Y     = 1 << 3,
	Up    = 1 << 4,
	Down  = 1 << 5,
	Left  = 1 << 6,
	Right = 1 << 7,
	R1    = 1 << 8,
	R2    = 1 << 9,
	L1    = 1 << 10,
	L2    = 1 << 11
};

enum DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

struct Vector2 {
	float x;
	float y;
	Vector2(float _x, float _y) : x(_x), y(_y) {}

    // Example: Overload the addition operator
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
	Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
	Vector2 operator/(const Vector2& other) const {
        return Vector2(x / other.x, y / other.y);
    }
	Vector2 operator*(const Vector2& other) const {
        return Vector2(x * other.x, y * other.y);
    }
};

struct Point {
	Vector2 pos = {0, 0};
	Point(float _x, float _y) {
		pos.x = _x;
		pos.y = _y;
	}
	bool visited = false;
};

inline uint8_t operator|(Button &a, Button &b) {
	return static_cast<uint8_t>(a) | static_cast<uint8_t>(b);
}
inline uint8_t operator&(Button &a, Button &b) {
	return static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}
inline uint8_t operator|=(uint8_t &a, Button &b) {
	return a = static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}
inline uint8_t operator&=(uint8_t &a, Button &b) {
	return a = static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}

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
    string printtext;
    if constexpr (is_same_v<T, string> || is_same_v<T, const char*>) {
        // Handle string and C-style string types
        printtext = input;
    } else {
        // Handle other types using stringstream
        stringstream ss;
        ss << input;
        printtext = ss.str();
    }
    pros::lcd::set_text(row, printtext);
}


//| RUNTIME FUNCTIONS:

// Updates button as a uint16_t
void updateControllerData() {
	buttons = buttons
	| (static_cast<uint8_t>(Button::A) * controller.get_digital(ControllerDigitalA))
	| (static_cast<uint8_t>(Button::B) * controller.get_digital(ControllerDigitalB))
	| (static_cast<uint8_t>(Button::X) * controller.get_digital(ControllerDigitalX))
	| (static_cast<uint8_t>(Button::Y) * controller.get_digital(ControllerDigitalY))
	| (static_cast<uint8_t>(Button::Up) * controller.get_digital(ControllerDigitalUp))
	| (static_cast<uint8_t>(Button::Down) * controller.get_digital(ControllerDigitalDown))
	| (static_cast<uint8_t>(Button::Left) * controller.get_digital(ControllerDigitalLeft))
	| (static_cast<uint8_t>(Button::Right) * controller.get_digital(ControllerDigitalRight))
	| (static_cast<uint8_t>(Button::R1) * controller.get_digital(ControllerDigitalR1))
	| (static_cast<uint8_t>(Button::R1) * controller.get_digital(ControllerDigitalR2))
	| (static_cast<uint8_t>(Button::L1) * controller.get_digital(ControllerDigitalL1))
	| (static_cast<uint8_t>(Button::L1) * controller.get_digital(ControllerDigitalL2));
	rightAnalogX = controller.get_analog(ControllerAnalogRightX);
	rightAnalogY = controller.get_analog(ControllerAnalogRightY);
	leftAnalogX = controller.get_analog(ControllerAnalogLeftY);
	leftAnalogY = controller.get_analog(ControllerAnalogLeftY);
}




void checkPauseProgram() { /// REMOVE THIS FUNCTION FOR FINAL COMPETITION
	if (controller.get_digital_new_press(ControllerDigitalX)) {
		brakeWheels();
		while (!controller.get_digital_new_press(ControllerDigitalX)) {
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
	uint32_t now = pros::millis();
	float heading = truncate(inertial.get_heading());
	float left_pos = (topLeft.get_raw_position(&now) + bottomLeft.get_raw_position(&now)) / 2;
	float right_pos = (topRight.get_raw_position(&now) + bottomRight.get_raw_position(&now)) / 2;
	float all_rot_now = (left_pos + right_pos) / 2;
	float all_rot_delta = all_rot_now - all_rot_prev;
	pos.x += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * sin(toRadians(heading));
	pos.y += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * cos(toRadians(heading));
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


Vector2 getPurePursuitLoc() {
	// float a = pow(tarx - prevx, 2) + pow(tary - prevy, 2);
	// float b = 2 * (((prevx - x) * (tarx - prevx)) + ((prevy - y) * (tary - prevy)));
	// float c = (pow(prevx - x, 2) + pow(prevy - y, 2)) - pow(r, 2);
	// float discriminate = pow(b, 2) - (4 * a * c);
	// float t1 = (-b + sqrt(discriminate)) / (2 * a);
	// float t2 = (-b - sqrt(discriminate)) / (2 * a);
	// float x_intercept1 = prevx + (tarx - prevx) * t1;
	// float y_intercept1 = prevy + (tary - prevy) * t1;

	// float x_intercept2 = prevx + (tarx - prevx) * t2;
	// float y_intercept2 = prevy + (tary - prevy) * t2;
	// bool within_x = (minX <= x_intercept1 && x_intercept1 <= maxX) || (minX <= x_intercept2 && x_intercept2 <= maxX);
	// bool within_y = (minY <= y_intercept1 && y_intercept1 <= maxY) || (minY <= y_intercept2 && y_intercept2 <= maxY);
	// //| lahjick :/
	// if (discriminate >= 0) {
	// 	if (within_x && within_y) {
	// 		if (abs(x_intercept2 - tarx) + abs(y_intercept2 - tary) < abs(x_intercept1 - tarx) + abs(y_intercept1 - tary)) {
	// 			PID(x_intercept2, y_intercept2);
	// 			println(x_intercept2, 1);
	// 			println(y_intercept2, 2);
	// 			println(rot, 3);
	// 			println("Intercept 2", 4);
	// 		} else {
	// 			PID(x_intercept1, y_intercept1);
	// 			println(x_intercept1, 1);
	// 			println(y_intercept1, 2);
	// 			println(p_x);
	// 		}
	// 	}
	// }
	// left_speed = PID_dist - PID_rot;
	// right_speed = PID_dist + PID_rot;
}

void autonomous() {
	Point target = autonPoints.at(0);
	Point prevPoint = pos;
	for (int ptIdx = 0; ptIdx < autonPoints.size(); ptIdx++) {
		Point &point = autonPoints.at(ptIdx);
		while (!point.visited) {
			//| DO AUTON CODE
		}
	}
}

// MARK: Driving
void drivePipeline(float driveSpeed) {
	// Controller analog is -1 to 1
	float left_speed;
	float right_speed;
	if (buttons & Button::A) {
		drivingMode = drivingMode == SINGLE_JOYSTICK ? TANK : SINGLE_JOYSTICK;
	}
	if (drivingMode == SINGLE_JOYSTICK) {
		float left_speed = (leftAnalogY * (driveSpeed / 100)) + (leftAnalogX * (driveSpeed / 150));
		float right_speed = (leftAnalogY * (driveSpeed / 100)) - (leftAnalogX * (driveSpeed / 150));
	} else {
		float left_speed = (leftAnalogY * (driveSpeed / 100));
		float right_speed = (rightAnalogY * (driveSpeed / 100));
	}
	moveWheels(min(left_speed * 600, 600.0f), min(right_speed * 600, 600.0f));
	trackPosition();
	println(pos.x);
	println(pos.y, 2);
}

// MARK: Scoring
void scorePipeline(float *driveSpeed, float *elevatorSpeed) {
	float maxElevatorSpeed = 200.0f;
	uint16_t first_pressed = 0;
	if (buttons & Button::Up && first_pressed != static_cast<uint16_t>(Button::Down)) {
		first_pressed = static_cast<uint16_t>(Button::Up);
		*driveSpeed += 10;
	} if (buttons & Button::Down && first_pressed != static_cast<uint16_t>(Button::Up)) {
		first_pressed = static_cast<uint16_t>(Button::Down);
		*driveSpeed -= 10;
	}
	if (controller.get_digital(ControllerDigitalR2)) {
		conveyor.move_velocity(*elevatorSpeed);
		intake.brake();
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		agitator.brake();
		*elevatorSpeed -= 3;
		*elevatorSpeed = max(*elevatorSpeed, -1.0f * maxElevatorSpeed);
	} else if (controller.get_digital(ControllerDigitalR1) and first_pressed != static_cast<uint16_t>(Button::L1)) {
		first_pressed = static_cast<uint16_t>(Button::R1);
		conveyor.move_velocity(*elevatorSpeed);
		bandRotatorTop.set_reversed(true);
		bandRotatorTop.move_velocity(275);
		intake.move_velocity(200);
		bandRotatorBottom.brake();
		*elevatorSpeed += 3;
		*elevatorSpeed = min(*elevatorSpeed, maxElevatorSpeed);
		agitator.move_velocity(200);
	} else if (controller.get_digital(ControllerDigitalL1) and first_pressed != static_cast<uint16_t>(Button::R1)) {
		first_pressed = static_cast<uint16_t>(Button::L1);
		conveyor.move_velocity(*elevatorSpeed);
		bandRotatorTop.set_reversed(false);
		bandRotatorTop.move_velocity(275);
		bandRotatorBottom.move_velocity(200);
		intake.brake();
		*elevatorSpeed += 3;
		*elevatorSpeed = min(*elevatorSpeed, maxElevatorSpeed);
		agitator.move_velocity(200);
	} else {
		*elevatorSpeed = 0;
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
