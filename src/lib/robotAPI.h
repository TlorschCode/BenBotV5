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

// MARK: DrivetrainMotor
class DrivetrainMotor {
private:
public:
	pros::Motor rawMotor;
	bool isLeftSide;
	int RPM;

	DrivetrainMotor();
	DrivetrainMotor(int port, bool _isLeftSide);

	void setVelocityPercent(float vel);
	void brake();
	void setDirection(bool reversed);
	double getActualVelocity();
	constexpr inline DrivetrainMotor& operator=(const DrivetrainMotor& _new) {
		RPM = _new.RPM;
		isLeftSide = _new.isLeftSide;
		return *this;
	}
};

// MARK: Drivetrain
struct Drivetrain {
	DrivetrainMotor topLeft;
	DrivetrainMotor topRight;
	DrivetrainMotor bottomLeft;
	DrivetrainMotor bottomRight;
	Drivetrain();
	Drivetrain(std::array<DrivetrainMotor, 4> _wheels);
	Drivetrain(DrivetrainMotor _topLeft, DrivetrainMotor _topRight, DrivetrainMotor _bottomLeft, DrivetrainMotor _bottomRight);
	inline std::array<DrivetrainMotor*, 4> getAll_asPtr() noexcept { return {&topLeft, &topRight, &bottomLeft, &bottomRight}; };
	constexpr inline Drivetrain& operator=(const Drivetrain& _new) noexcept {
		topLeft = _new.topLeft;
		topRight = _new.topRight;
		bottomLeft = _new.bottomLeft;
		bottomRight = _new.bottomRight;
		return *this;
	}
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

// MARK: Drive Mode Enum
enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

// MARK: Robot
class Robot {
public:
	Vec2 pos = {0, 0};
	Heading heading = {};
	Drivetrain drivetrain;
	pros::Imu inertial;
	std::unique_ptr<autonAPI::PID_Controller> autonController;
	DrivingMode driveMode = DrivingMode::TANK;

	Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial);

	void moveWheels(float speedLeftPercent, float speedRightPercent);
	void brakeWheels();
	void setSpeedFromController(ctrlAPI::Controller &controller);
	void swapDriveMode();
	constexpr inline Robot& operator=(const Robot& _new) noexcept {
		pos = _new.pos;
		heading = _new.heading;
		drivetrain = _new.drivetrain;
		return *this;
	}
};

} // namespace robotAPI