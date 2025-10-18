#include "main.h" // pros
#include <fstream>
#include <string>
#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <tuple>

using namespace std;


// MARK: Definitions
#define ControllerAnalogRightX pros::E_CONTROLLER_ANALOG_RIGHT_X
#define ControllerAnalogRightY pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ControllerAnalogLeftX pros::E_CONTROLLER_ANALOG_LEFT_X
#define ControllerAnalogLeftY pros::E_CONTROLLER_ANALOG_LEFT_Y

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
#define PI 3.14159265358979323846

constexpr float gear_ratio = {0.5f}; // Wheel-motor gear ratio
constexpr float wheel_circumference = {12.56f};
constexpr float frame = {20}; // Frame time

// MARK: Templates
struct Vector2;
struct Point;
class Heading;
class DrivetrainMotor;
class Robot;
class PID_Controller;












// MARK: Enum/Struct
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

enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

struct Vector2 {
	float x;
	float y;
	Vector2(float _x, float _y) : x(_x), y(_y) {}
	Vector2() : x(0), y(0) {}

    // Example: Overload the addition operator
    inline Vector2 operator+(const Vector2 &other) const {
        return Vector2(x + other.x, y + other.y);
    }
	inline Vector2 operator-(const Vector2 &other) const {
        return Vector2(x - other.x, y - other.y);
    }
	inline Vector2 operator/(const Vector2 &other) const {
        return Vector2(x / other.x, y / other.y);
    }
	inline Vector2 operator*(const Vector2 &other) const {
        return Vector2(x * other.x, y * other.y);
    }
};

struct Point {
	Vector2 pos = {0, 0};
	Point(float _x, float _y) {
		pos.x = _x;
		pos.y = _y;
	}
	Point(const Vector2 &v) : pos(v) {};
	bool visited = false;
	inline Point& operator=(const Vector2 &other) {
		pos = other;
		return *this;
	}
};

void brakeWheels();
void checkPauseProgram();
void updateControllerData();

// MARK: Utilities
inline double toRadians(float degrees) {
	return degrees * (PI / 180);
}

inline double toDegrees(double radians) {
	return radians * (180 / PI);
}

inline double truncate(double num, int cutoff = 2) {
	return floor(num * pow(10, cutoff)) / pow(10, cutoff);
}

inline int sign(float &input) {
	return (input >= 0) ? 1 : -1;
}

inline double map_value(double &input, double &input_start, double &input_end, double &output_start, double &output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

inline float degreesTill(Vector2 &from, Vector2 &to) {
	return toDegrees(atan2(to.y - from.y, to.x - from.x));
}

inline float radiansTill(Vector2 &from, Vector2 &to) {
	return atan2(to.y - from.y, to.x - from.x);
}

// MARK: Norm Globals
float all_rot_prev = {0};
float rightAnalogX = {0};
float rightAnalogY = {0};
float leftAnalogX = {0};
float leftAnalogY = {0};
float allRotPrev = {0}; // The previous rotation of all the drivetrain wheels
bool pressingPneumatics = false;












// MARK: Hardware Init
pros::Controller controller(pros::E_CONTROLLER_MASTER);
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
	topLeft.set_reversed(true);
	bottomLeft.set_reversed(true);
    conveyor.set_reversed(true);
	intake.set_reversed(true);
	topLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	topRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	bottomRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}
pros::Imu inertial(5);


pros::adi::Pneumatics loaderRod('A', false);


// MARK: Heading
class Heading {
	private:
		float degrees = 0;
	public:
		inline float getDegrees() {
			return degrees;
		}
		inline double getRadians() {
			return toRadians(degrees);
		}
		void setDegrees(float amount) {
			degrees = amount;
		}
		void setRadians(double amount) {
			degrees = toDegrees(amount);
		}
};

// MARK: Drivetrain
class DrivetrainMotor {
	private:
		int RPM;
	public:
		pros::Motor motor;
		bool isLeftSide;
		DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide) : motor(_motor) {
			isLeftSide = _isLeftSide;
			switch (_motor.get_gearing()) {
				case pros::v5::MotorGears::red:    // high torque (36:1)
					RPM = 100; break;
				case pros::v5::MotorGears::green:  // standard (18:1)
					RPM = 200; break;
				case pros::v5::MotorGears::blue:   // high speed (6:1)
					RPM = 600; break;
				default:
					RPM = 0; break;
			}
		}
		void setVelocityPercent(float vel) {
			motor.move_velocity((vel / 100) * RPM);
		}
		void brake() {
			motor.brake();
		}
		void setDirection(bool reversed) {
			motor.set_reversed(reversed);
		}
		inline double getActualVelocity() {
			return motor.get_actual_velocity();
		}
};


//MARK: ROBOT
class Robot {  // Robot class for more readable code
	public:
		Vector2 pos = {0, 0};
		Heading heading = {};
		array<DrivetrainMotor, 4> wheels;
		Robot(array<DrivetrainMotor, 4> _wheels) : wheels(_wheels) {}
		void moveWheels(float speedLeftPercent, float speedRightPercent) {
			wheels.at(0).setVelocityPercent(wheels.at(0).isLeftSide ? speedLeftPercent : speedRightPercent);
			wheels.at(1).setVelocityPercent(wheels.at(1).isLeftSide ? speedLeftPercent : speedRightPercent);
			wheels.at(2).setVelocityPercent(wheels.at(2).isLeftSide ? speedLeftPercent : speedRightPercent);
			wheels.at(3).setVelocityPercent(wheels.at(3).isLeftSide ? speedLeftPercent : speedRightPercent);
		}
		void updateData() {
			uint32_t now = pros::millis();
			heading.setDegrees(truncate(inertial.get_rotation()));
			float left_pos = (topLeft.get_raw_position(&now) + bottomLeft.get_raw_position(&now)) / 2;
			float right_pos = (topRight.get_raw_position(&now) + bottomRight.get_raw_position(&now)) / 2;
			float all_rot_now = (left_pos + right_pos) / 2;
			float all_rot_delta = all_rot_now - all_rot_prev;
			pos.x += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * sin(heading.getRadians());
			pos.y += ((all_rot_delta / 360) * gear_ratio * wheel_circumference) * cos(heading.getRadians());
			all_rot_prev = all_rot_now;
		}
};

// MARK: PID
constexpr float pPosWeight = {6.0f};
constexpr float iPosWeight = {0.05f};
constexpr float dPosWeight = {4.2f};
constexpr float pRotWeight = {1.0f};
constexpr float iRotWeight = {0.05f};
constexpr float dRotWeight = {0.85f};
class PID_Controller {
	private:
		Vector2 pPos = {0, 0};
		Vector2 iPos = {0, 0};
		Vector2 dPos = {0, 0};
		float pRot = 0;
		float iRot = 0;
		float dRot = 0;
		float PID_rot = 0;
		float prev_rot = 0;
	public:
		PID_Controller() {};
		void update(Vector2 &_target, Robot &_robot) {
			float original_x = 0;
			float original_y = 0;
			float original_rot = 0;
			bool auton_rot = true; // flag if heading is close enough to target
			double rot_radians = _robot.heading.getRadians();
			pPos = {(_target.x - _robot.pos.x) * pPosWeight, (_target.y - _robot.pos.y) * pPosWeight};
			iPos = {(iPos.x + pPos.x) * cos(rot_radians), (iPos.y + pPos.y) * cos(rot_radians)};
			dPos = {original_x - (_robot.pos.x * dPosWeight), original_y - (_robot.pos.y * dPosWeight)};
			Vector2 PID = {(pPos.x + (iPos.x * iPosWeight) + (dPos.x * dPosWeight)) * sin(rot_radians), (pPos.y + (iPos.y * iPosWeight) + (dPos.y * dPosWeight)) * cos(rot_radians)};
			//| rotation -
			float target_rotation = degreesTill(_robot.pos, _target);
			pRot = (target_rotation - pRot) * pRotWeight;
			iRot += pRot;
			// FIXME:
			dRot = original_rot - _robot.heading.getDegrees();
			PID_rot = ((pRot + (iRot * iRotWeight) + (dRot * dRotWeight)) * auton_rot);
		}
};


// MARK: Cust Globals
array<DrivetrainMotor, 4> drivetrain = {DrivetrainMotor(topLeft, true), DrivetrainMotor(bottomLeft, true), DrivetrainMotor(topRight, false), DrivetrainMotor(bottomRight, false)};
Robot robot = Robot(drivetrain);
PID_Controller pidController = PID_Controller();
vector<Point> autonPoints = {
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
	buttons = 0;
	buttons = buttons
	| (static_cast<uint16_t>(Button::A) * controller.get_digital(ControllerDigitalA))
	| (static_cast<uint16_t>(Button::B) * controller.get_digital(ControllerDigitalB))
	| (static_cast<uint16_t>(Button::X) * controller.get_digital(ControllerDigitalX))
	| (static_cast<uint16_t>(Button::Y) * controller.get_digital(ControllerDigitalY))
	| (static_cast<uint16_t>(Button::Up) * controller.get_digital(ControllerDigitalUp))
	| (static_cast<uint16_t>(Button::Down) * controller.get_digital(ControllerDigitalDown))
	| (static_cast<uint16_t>(Button::Left) * controller.get_digital(ControllerDigitalLeft))
	| (static_cast<uint16_t>(Button::Right) * controller.get_digital(ControllerDigitalRight))
	| (static_cast<uint16_t>(Button::R1) * controller.get_digital(ControllerDigitalR1))
	| (static_cast<uint16_t>(Button::R2) * controller.get_digital(ControllerDigitalR2))
	| (static_cast<uint16_t>(Button::L1) * controller.get_digital(ControllerDigitalL1))
	| (static_cast<uint16_t>(Button::L2) * controller.get_digital(ControllerDigitalL2));
	buttonsNewPress = 0;
	buttonsNewPress = buttonsNewPress
	| (static_cast<uint16_t>(Button::A) * controller.get_digital_new_press(ControllerDigitalA))
	| (static_cast<uint16_t>(Button::B) * controller.get_digital_new_press(ControllerDigitalB))
	| (static_cast<uint16_t>(Button::X) * controller.get_digital_new_press(ControllerDigitalX))
	| (static_cast<uint16_t>(Button::Y) * controller.get_digital_new_press(ControllerDigitalY))
	| (static_cast<uint16_t>(Button::Up) * controller.get_digital_new_press(ControllerDigitalUp))
	| (static_cast<uint16_t>(Button::Down) * controller.get_digital_new_press(ControllerDigitalDown))
	| (static_cast<uint16_t>(Button::Left) * controller.get_digital_new_press(ControllerDigitalLeft))
	| (static_cast<uint16_t>(Button::Right) * controller.get_digital_new_press(ControllerDigitalRight))
	| (static_cast<uint16_t>(Button::R1) * controller.get_digital_new_press(ControllerDigitalR1))
	| (static_cast<uint16_t>(Button::R2) * controller.get_digital_new_press(ControllerDigitalR2))
	| (static_cast<uint16_t>(Button::L1) * controller.get_digital_new_press(ControllerDigitalL1))
	| (static_cast<uint16_t>(Button::L2) * controller.get_digital_new_press(ControllerDigitalL2));
	rightAnalogX = (controller.get_analog(ControllerAnalogRightX) / 127) * 100; // 127 is max value from get_analog, turn to percent
	rightAnalogY = (controller.get_analog(ControllerAnalogRightY) / 127) * 100;
	leftAnalogX = (controller.get_analog(ControllerAnalogLeftX) / 127) * 100;
	leftAnalogY = (controller.get_analog(ControllerAnalogLeftY) / 127) * 100;
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

Vector2 updatePID(Vector2 target) {
	//|    PID    |//
	//| distance -
	
	double rot_radians = robot.heading.getRadians();
	pidController.update(target, robot);
	
}


Vector2 getPurePursuitLoc(float checkRadius, Vector2 target, Vector2 prevTarget) {
	Vector2 minTarget = Vector2(min(target.x, prevTarget.x), min(target.y, prevTarget.y));
	Vector2 maxTarget = Vector2(max(target.x, prevTarget.x), max(target.y, prevTarget.y));

	// FIXME: CHECK MATH vvv
	float dotProduct = pow(target.x - prevTarget.x, 2) + pow(target.y - prevTarget.y, 2);
	float twoceCircleOffset = 2 * (((prevTarget.x - robot.pos.x) * (target.x - prevTarget.x)) + ((prevTarget.y - robot.pos.y) * (target.y - prevTarget.y)));
	float prevTarCurPosDist = (pow(prevTarget.x - robot.pos.x, 2) + pow(prevTarget.y - robot.pos.y, 2)) - pow(checkRadius, 2);
	float discriminant = pow(twoceCircleOffset, 2) - (4 * dotProduct * prevTarCurPosDist);
	float intersectRatio1 = (-twoceCircleOffset + sqrt(discriminant)) / (2 * dotProduct);
	float intersectRatio2 = (-twoceCircleOffset - sqrt(discriminant)) / (2 * dotProduct);
	// FIXME: CHECK MATH ^^^

	Vector2 intercept1 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio1, prevTarget.y + (target.y - prevTarget.y) * intersectRatio1};
	Vector2 intercept2 = {prevTarget.x + (target.x - prevTarget.x) * intersectRatio2, prevTarget.y + (target.y - prevTarget.y) * intersectRatio2};

	bool within_x = (minTarget.x <= intercept1.x <= maxTarget.x) || (minTarget.x <= intercept2.y <= maxTarget.x);
	bool within_y = (minTarget.y <= intercept1.y <= maxTarget.y) || (minTarget.y <= intercept2.y <= maxTarget.y);
	if (discriminant >= 0) {
		if (within_x && within_y) {
			if (abs(intercept2.x - target.x) + abs(intercept2.y - target.y) < abs(intercept1.x - target.x) + abs(intercept1.y - target.y)) {
				return intercept2;
			} else {
				return prevTarget;
			}
		}
	}
}

void autonomous() {
	constexpr float robotCheckRadius = 10.0f;
	Point target = autonPoints.at(0);
	Point prevPoint = robot.pos;
	Vector2 curTargetLoc = {0, 0};
	Vector2 prevTargetLoc = {0, 0};
	// for (size_t ptIdx = 0; ptIdx < autonPoints.size(); ptIdx++) {
	// 	Point &point = autonPoints.at(ptIdx);
	// 	while (!point.visited) {
	// 		curTargetLoc = getPurePursuitLoc(robotCheckRadius, point.pos, prevTargetLoc);
	// 		updatePID(curTargetLoc);
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
		left_speed = (leftAnalogY * (driveSpeedMult / 100)) + (leftAnalogX * (driveSpeedMult / 100) * 0.75f);
		right_speed = (leftAnalogY * (driveSpeedMult / 100)) - (leftAnalogX * (driveSpeedMult / 100) * 0.75f);
	} else if (drivingMode == DrivingMode::TANK) {
		left_speed = (leftAnalogY * -(driveSpeedMult / 100));
		right_speed = (rightAnalogY * (driveSpeedMult / 100));
	}
	robot.moveWheels(left_speed, right_speed);
	trackPosition();
}

// MARK: Scoring
void scorePipeline(float *driveSpeedMult, float *elevatorSpeed) {
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
	if (buttons & Button::R2) {
		conveyor.move_velocity(*elevatorSpeed);
		intake.brake();
		bandRotatorBottom.brake();
		bandRotatorTop.brake();
		agitator.brake();
		*elevatorSpeed -= 3;
		*elevatorSpeed = max(*elevatorSpeed, -1.0f * maxElevatorSpeed);
	} else if (buttons & Button::R1 && first_pressed != Button::L1) {
		first_pressed = Button::R1;
		conveyor.move_velocity(*elevatorSpeed);
		bandRotatorTop.set_reversed(true);
		bandRotatorTop.move_velocity(275);
		intake.move_velocity(200);
		bandRotatorBottom.brake();
		*elevatorSpeed += 3;
		*elevatorSpeed = min(*elevatorSpeed, maxElevatorSpeed);
		agitator.move_velocity(200);
	} else if (buttons & Button::L1 && first_pressed != Button::R1) {
		first_pressed = Button::L1;
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
	float driveSpeedMult = 100.0f;
	float elevator_speed = 100.0f;
	clear_screen();
	wait(1000);
	while (true) {
		updateControllerData();
		robot.updateData();
		drivePipeline(driveSpeedMult);
		scorePipeline(&driveSpeedMult, &elevator_speed);
		wait(frame);
	}
	controller.rumble("-");
	brakeWheels();
}
