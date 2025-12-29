#pragma once
#include "globalAPI.h"
#include "controllerAPI.h"
#include <array>
#include <memory>

namespace autonAPI {
	class PID_Controller;
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
	double getActualVelocity();
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
	DrivingMode driveMode = DrivingMode::TANK;
	DrivetrainMotor topLeft;
	DrivetrainMotor middleLeft;
	DrivetrainMotor bottomLeft;
	DrivetrainMotor topRight;
	DrivetrainMotor middleRight;
	DrivetrainMotor bottomRight;
	float leftSpeed = 0;
	float rightSpeed = 0;

	Drivetrain();
	Drivetrain(std::array<DrivetrainMotor, 6> _wheels);
	Drivetrain(DrivetrainMotor _topLeft, DrivetrainMotor _middleLeft, DrivetrainMotor _bottomLeft, DrivetrainMotor _topRight, DrivetrainMotor _middleRight, DrivetrainMotor _bottomRight);
	Drivetrain(std::array<pros::Motor, 6> drivetrain, pros::motor_gearset_e _gearset);
	constexpr inline Drivetrain& operator=(const Drivetrain& _new) noexcept {
		topLeft = _new.topLeft;
		middleLeft = _new.middleLeft;
		bottomLeft = _new.bottomLeft;
		topRight = _new.topRight;
		middleRight = _new.middleRight;
		bottomRight = _new.bottomRight;
		return *this;
	}

	inline std::array<DrivetrainMotor*, 6> getWheelsAsPtrs() noexcept {
		return {&topLeft, &topRight, &middleLeft, &middleRight, &bottomLeft, &bottomRight};
	};
	inline void setDriveMode(DrivingMode mode) noexcept {
		driveMode = mode;
	}
	// Swaps driveMode between DrivingMode::SINGLE_JOYSTICK and DrivingMode::TANK
	inline void swapDriveMode() noexcept {
		driveMode = (driveMode == DrivingMode::SINGLE_JOYSTICK) ? DrivingMode::TANK : DrivingMode::SINGLE_JOYSTICK;
	}

	void setBrakeMode(pros::motor_brake_mode_e mode);
	void moveWheels();
	void moveWheels(float speedLeft, float speedRight);
	void brakeWheels();
	void setSpeedFromController(const ctrlAPI::Controller &controller);
};

// MARK: Heading
class Heading {
private:
	float degrees = 0;
public:
	Heading() = default;
	float getDegrees() const;
	double getRadians() const;
	void setDegrees(float amount);
	void setRadians(double amount);
};
// MARK: Robot
// A container for a drivetrain, an auton controlller, and various other parts.
class Robot {
public:
	Vec2 pos = {0, 0};
	Heading heading = {};
	Drivetrain drivetrain;
	pros::Imu inertial;
	std::unique_ptr<autonAPI::PID_Controller> autonController;

	Robot(std::array<DrivetrainMotor, 6> _wheels, pros::Imu _inertial);

	constexpr inline Robot& operator=(const Robot& _new) noexcept {
		pos = _new.pos;
		heading = _new.heading;
		drivetrain = _new.drivetrain;
		return *this;
	}
};

} // namespace robotAPI