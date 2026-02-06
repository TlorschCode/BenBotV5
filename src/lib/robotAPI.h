#pragma once
#include "common.h"
#include "controllerAPI.h"
#include <stdexcept>
#include <array>
#include <memory>

namespace autonAPI {
	class AutonController;
}

namespace robotAPI {
// MARK: Forward declarations
class DrivetrainMotor;
class Robot;

// MARK: Drive Mode
enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

// MARK: DrivetrainMotor
class DrivetrainMotor {
private:
	// Configures a motor during construction.
	static void configMotor(pros::Motor &_motor, bool _reversed);
public:
	pros::Motor rawMotor;
	bool reversed;
	int RPM;

	DrivetrainMotor();
	DrivetrainMotor(int port, bool _reversed, pros::v5::MotorGearset gearset);
	DrivetrainMotor(const pros::Motor &_motor, bool _reversed);


	void setVelocityPercent(float vel);
	void brake();
	void setDirection(bool reversed);
	inline double getActualVelocity() const {
		return rawMotor.get_actual_velocity();
	}
	inline double getPosition() const {
		return rawMotor.get_position();
	}

	constexpr inline DrivetrainMotor& operator=(const DrivetrainMotor& _new) {
		RPM = _new.RPM;
		reversed = _new.reversed;
		return *this;
	}
};

// MARK: Drivetrain
// TODO: Put motors into a struct called motors for clarity
class Drivetrain {
private:
public:
	const float WHEEL_CIRCUMFERENCE; // inches
	const float GEAR_RATIO;
	DrivingMode driveMode = DrivingMode::TANK;
	DrivetrainMotor w_topLeft;
	DrivetrainMotor w_middleLeft;
	DrivetrainMotor w_bottomLeft;
	DrivetrainMotor w_topRight;
	DrivetrainMotor w_middleRight;
	DrivetrainMotor w_bottomRight;
	float leftSpeed = 0;
	float rightSpeed = 0;

	// Empty Drivetrain constructor
	Drivetrain();

	// Drivetrain constructor
	// hardwareGearRatio is the motor-turns to wheel-turns ratio, for robots with gearing between the motor shaft and the actual wheel.
	Drivetrain(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio);

	// Drivetrain constructor
    // hardwareGearRatio is the motor-turns to wheel-turns ratio, for robots with gearing between the motor shaft and the actual wheel.
	Drivetrain(const DrivetrainMotor &_topLeft, const DrivetrainMotor &_middleLeft, const DrivetrainMotor &_bottomLeft,
		       const DrivetrainMotor &_topRight, const DrivetrainMotor &_middleRight, const DrivetrainMotor &_bottomRight,
			   float wheelCircumference, float hardwareGearRatio);
	
	// Drivetrain constructor
	// hardwareGearRatio is the motor-turns to wheel-turns ratio, for robots with gearing between the motor shaft and the actual wheel.
	Drivetrain(const std::array<pros::Motor, 6> &drivetrain, pros::motor_gearset_e _gearset,
			   float wheelCircumference, float hardwareGearRatio);
	

	constexpr inline Drivetrain& operator=(const Drivetrain& _new) noexcept {
		w_topLeft = _new.w_topLeft;
		w_middleLeft = _new.w_middleLeft;
		w_bottomLeft = _new.w_bottomLeft;
		w_topRight = _new.w_topRight;
		w_middleRight = _new.w_middleRight;
		w_bottomRight = _new.w_bottomRight;
		return *this;
	}

	// Returns the drivetrain's wheels as an array of pointers to them.
	// Order:
	// - top-left, middle-left, bottom-left, top-right, middle-right, bottom-right
	inline std::array<DrivetrainMotor*, 6> getWheelsAsPtrs() noexcept {
		return {&w_topLeft, &w_middleLeft, &w_bottomLeft, &w_topRight, &w_middleRight,&w_bottomRight};
	};
	// Sets driveMode. Can be either: SINGLE_JOYSTICK or TANK.
	// driveMode determines how the drivetrain can be controlled with joysticks.
	// - SINGLE_JOYSTICK means the drivetrain will be controlled solely with the left joystick.
	// - TANK means the drivetrain will be controlled by both joysticks, only accounting for their vertical positions.
	inline void setDriveMode(DrivingMode mode) noexcept {
		driveMode = mode;
	}
	// Swaps driveMode between DrivingMode::SINGLE_JOYSTICK and DrivingMode::TANK
	inline void swapDriveMode() noexcept {
		driveMode = (driveMode == DrivingMode::SINGLE_JOYSTICK) ? DrivingMode::TANK : DrivingMode::SINGLE_JOYSTICK;
	}

	// Returns the averaged position of all left motors
	// Return unit: the encoding unit of each motor
	inline float getLeftMotorsPos() const {
		return (w_topLeft.getPosition() + w_middleLeft.getPosition() + w_bottomLeft.getPosition()) / 3;
	}
	// Returns the averaged position of all right motors
	// Return unit: the encoding unit of each motor
	inline float getRightMotorsPos() const {
		return (w_topRight.getPosition() + w_middleRight.getPosition() + w_bottomRight.getPosition()) / 3;
	}

	// Sets the drivetrarin's leftSpeed and rightSpeed according to a Vec2.
	// leftSpeed is set to the 'x' attribute of the Vec2, and rightSpeed is set to the 'y' attribute.
	void setSpeedFromSpeedPair(const SpeedPair &pair);

	// Sets the brake mode of each of the wheels.
	void setBrakeMode(pros::motor_brake_mode_e mode);

	// Moves the wheels according to the drivetrain's leftSpeed and rightSpeed.
	void moveWheels();

	// Moves the wheels according to the inputted speedLeft and speedRight.
	// Sets the drivetrain's leftSpeed and rightSpeed to the corresponding inputs.
	void moveWheels(float speedLeft, float speedRight);

	// Brakes all of the wheels
	void brakeWheels();

	// Sets the wheels' speeds from an inputted controller.
	// Uses driveMode and calculates speed for tank drive or single-joystick accordingly.
	void setSpeedFromController(const ctrlAPI::Controller &controller);
};

// Struct for heading which forces the radian and degree values to remain synchronized

// MARK: Robot

// Singleton.
// A container for a drivetrain, an auton controlller, and various other parts.
// pos: position of the robot
// heading: the heading of the robot (in degrees)
class Robot {
private:
	// hardwareGearRatio is the external gear ratio for the drivetrain from motor shaft rotations to actual wheel rotations. This is common for drivetrains with motors placed between wheels.
	Robot(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio, const pros::Imu &_inertial);
	static Robot* instance;
	~Robot() = default;
public:
	static Robot& Init(const std::array<DrivetrainMotor, 6> &_wheels, float wheelCircumference, float hardwareGearRatio, const pros::Imu &_inertial);
	static Robot& Get();
	Robot& operator=(const Robot&) = delete;
	Robot(const Robot&) = delete;
	Vec2 pos = {0, 0};
	float heading_deg = 0;
	Drivetrain drivetrain;
	pros::Imu inertial;
	std::atomic<std::shared_ptr<autonAPI::AutonController>> autonController;
};

} // namespace robotAPI