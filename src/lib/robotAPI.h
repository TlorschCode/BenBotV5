#pragma once
#include "globalAPI.h"
#include "controllerAPI.h"
#include <array>

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
	int RPM;
public:
	pros::Motor rawMotor;
	bool isLeftSide;

	DrivetrainMotor();
	DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide);

	void setVelocityPercent(float vel);
	void brake();
	void setDirection(bool reversed);
	double getActualVelocity();
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
	inline std::array<DrivetrainMotor, 4> getAll() const { return {topLeft, topRight, bottomLeft, bottomRight}; };
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
	autonAPI::PID_Controller *autonController;
	DrivingMode driveMode = DrivingMode::TANK;

	Robot();
	Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial);

	void moveWheels(float &speedLeftPercent, float &speedRightPercent);
	void brakeWheels();
	void setSpeedFromController(ctrlAPI::Controller &controller);
};

} // namespace robotAPI