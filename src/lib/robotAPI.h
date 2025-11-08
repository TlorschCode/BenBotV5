// #pragma once
// #include "globalAPI.h"
// #include "controllerAPI.h"
// #include <array>

// namespace autonAPI {
// 	class PID_Controller;
// }

// namespace robotAPI {
// class DrivetrainMotor;
// class Robot;
// // MARK: Drivetrain
// class DrivetrainMotor {
// 	private:
// 		int RPM;
// 	public:
		
// 		pros::Motor rawMotor;
// 		bool isLeftSide;
// 		DrivetrainMotor() : rawMotor(pros::Motor(0, pros::v5::MotorGearset::green)) {}
// 		DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide) : rawMotor(_motor) {
// 			isLeftSide = _isLeftSide;
// 			if (isLeftSide) rawMotor.set_reversed(true);
// 			switch (_motor.get_gearing()) {
// 				case pros::v5::MotorGears::red:    // high torque (36:1)
// 					RPM = 100; break;
// 				case pros::v5::MotorGears::green:  // standard (18:1)
// 					RPM = 200; break;
// 				case pros::v5::MotorGears::blue:   // high speed (6:1)
// 					RPM = 600; break;
// 				default:
// 					RPM = 0; break;
// 			}
// 		}
// 		void setVelocityPercent(float vel) {
// 			rawMotor.move_velocity((vel / 100) * RPM);
// 		}
// 		void brake() {
// 			rawMotor.brake();
// 		}
// 		void setDirection(bool reversed) {
// 			rawMotor.set_reversed(reversed);
// 		}
// 		inline double getActualVelocity() {
// 			return rawMotor.get_actual_velocity();
// 		}
// };

// struct Drivetrain {
// 	std::array<DrivetrainMotor, 4> wheels;
// 	Drivetrain() : wheels({DrivetrainMotor(), DrivetrainMotor(), DrivetrainMotor(), DrivetrainMotor()}) {};
// 	Drivetrain(std::array<DrivetrainMotor, 4> _wheels) : wheels(_wheels) {};
// };

// class Heading {
// 	private:
// 		float degrees = 0;
// 	public:
// 		Heading() {};
// 		inline float getDegrees() const {
// 			return degrees;
// 		}
// 		inline double getRadians() const {
// 			return toRadians(degrees);
// 		}
// 		void setDegrees(float amount) {
// 			degrees = amount;
// 		}
// 		void setRadians(double amount) {
// 			degrees = toDegrees(amount);
// 		}
// };

// enum class DrivingMode {
// 	SINGLE_JOYSTICK,
// 	TANK
// };

// //MARK: ROBOT
// class Robot {  // Robot class for more readable code
// 	private:
// 	public:
// 		Vec2 pos = {0, 0};
// 		Heading heading = {};
// 		Drivetrain drivetrain;
// 		pros::Imu inertial;
// 		autonAPI::PID_Controller *autonController;
// 		DrivingMode driveMode = DrivingMode::TANK;

// 		Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial) : drivetrain(_wheels), 
// 																			 inertial(_inertial), 
// 																			 autonController(new autonAPI::PID_Controller()) {}
// 		Robot() : inertial(pros::Imu(0)), 
// 				  drivetrain(),
// 				  autonController(new autonAPI::PID_Controller()) {};
// 		void moveWheels(float &speedLeftPercent, float &speedRightPercent) {
// 			drivetrain.wheels.at(0).setVelocityPercent(drivetrain.wheels.at(0).isLeftSide ? speedLeftPercent : speedRightPercent); // top left
// 			drivetrain.wheels.at(1).setVelocityPercent(drivetrain.wheels.at(1).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom left
// 			drivetrain.wheels.at(2).setVelocityPercent(drivetrain.wheels.at(2).isLeftSide ? speedLeftPercent : speedRightPercent); // top right
// 			drivetrain.wheels.at(3).setVelocityPercent(drivetrain.wheels.at(3).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom right
// 		}
// 		void brakeWheels() {
// 			drivetrain.wheels.at(0).brake();
// 			drivetrain.wheels.at(1).brake();
// 			drivetrain.wheels.at(2).brake();
// 			drivetrain.wheels.at(3).brake();
// 		}
// 		void moveDrivetrain(float leftSpeedPercent, float rightSpeedPercent) {
// 			drivetrain.wheels.at(0).setVelocityPercent(leftSpeedPercent);
// 			drivetrain.wheels.at(1).setVelocityPercent(leftSpeedPercent);
// 			drivetrain.wheels.at(2).setVelocityPercent(rightSpeedPercent);
// 			drivetrain.wheels.at(3).setVelocityPercent(rightSpeedPercent);
// 		}
// 		void setSpeedFromController(ctrlAPI::Controller &controller) {
// 			float leftSpeedPercent;
// 			float rightSpeedPercent;
//             if (driveMode == DrivingMode::SINGLE_JOYSTICK) {
// 		        const float joystickRadians = atan2(controller.leftAnalogXPercent / 100, controller.leftAnalogYPercent / 100);
// 		        const float alteredAnalogY = controller.leftAnalogYPercent * abs(cos(joystickRadians));
// 		        const float alteredAnalogX = controller.leftAnalogXPercent * abs(sin(joystickRadians));
//                 leftSpeedPercent = alteredAnalogY + alteredAnalogX;
// 		        rightSpeedPercent = alteredAnalogY - alteredAnalogX;
//             } else {
//                 leftSpeedPercent = controller.leftAnalogYPercent;
// 		        rightSpeedPercent = controller.rightAnalogYPercent;
//             }
// 			moveDrivetrain(leftSpeedPercent, rightSpeedPercent);
//         }
		
// };

// } // namespace robotAPI



// //| MARK: JUST DRIVE
// #pragma once
// #include "globalAPI.h"
// #include "controllerAPI.h"
// #include <array>

// // namespace autonAPI {
// // 	class PID_Controller;
// // }

// namespace robotAPI {
// class DrivetrainMotor;
// class Robot;

// // MARK: Drivetrain
// class DrivetrainMotor {
// 	private:
// 		int RPM;
// 	public:
// 		pros::Motor rawMotor;
// 		bool isLeftSide;

// 		DrivetrainMotor() : rawMotor(pros::Motor(0, pros::v5::MotorGearset::green)) {}
// 		DrivetrainMotor(pros::Motor &_motor, bool _isLeftSide) : rawMotor(_motor) {
// 			isLeftSide = _isLeftSide;
// 			if (isLeftSide) rawMotor.set_reversed(true);
// 			switch (_motor.get_gearing()) {
// 				case pros::v5::MotorGears::red:    // high torque (36:1)
// 					RPM = 100; break;
// 				case pros::v5::MotorGears::green:  // standard (18:1)
// 					RPM = 200; break;
// 				case pros::v5::MotorGears::blue:   // high speed (6:1)
// 					RPM = 600; break;
// 				default:
// 					RPM = 0; break;
// 			}
// 		}

// 		void setVelocityPercent(float vel) {
// 			rawMotor.move_velocity((vel / 100) * RPM);
// 		}

// 		void brake() {
// 			rawMotor.brake();
// 		}

// 		void setDirection(bool reversed) {
// 			rawMotor.set_reversed(reversed);
// 		}

// 		inline double getActualVelocity() {
// 			return rawMotor.get_actual_velocity();
// 		}
// };

// struct Drivetrain {
// 	std::array<DrivetrainMotor, 4> wheels;
// 	Drivetrain() : wheels({DrivetrainMotor(), DrivetrainMotor(), DrivetrainMotor(), DrivetrainMotor()}) {};
// 	Drivetrain(std::array<DrivetrainMotor, 4> _wheels) : wheels(_wheels) {};
// };

// class Heading {
// 	private:
// 		float degrees = 0;
// 	public:
// 		Heading() {};
// 		inline float getDegrees() const {
// 			return degrees;
// 		}
// 		inline double getRadians() const {
// 			return toRadians(degrees);
// 		}
// 		void setDegrees(float amount) {
// 			degrees = amount;
// 		}
// 		void setRadians(double amount) {
// 			degrees = toDegrees(amount);
// 		}
// };

// enum class DrivingMode {
// 	SINGLE_JOYSTICK,
// 	TANK
// };

// // MARK: ROBOT
// class Robot {
// 	public:
// 		Vec2 pos = {0, 0};
// 		Heading heading = {};
// 		Drivetrain drivetrain;
// 		pros::Imu inertial;
// 		// autonAPI::PID_Controller *autonController;
// 		DrivingMode driveMode = DrivingMode::TANK;

// 		Robot(std::array<DrivetrainMotor, 4> _wheels, pros::Imu _inertial)
// 			: drivetrain(_wheels),
// 			  inertial(_inertial)
// 			  /*, autonController(new autonAPI::PID_Controller())*/ {}

// 		Robot()
// 			: inertial(pros::Imu(0)),
// 			  drivetrain()
// 			  /*, autonController(new autonAPI::PID_Controller())*/ {}

// 		void moveWheels(float &speedLeftPercent, float &speedRightPercent) {
// 			drivetrain.wheels.at(0).setVelocityPercent(drivetrain.wheels.at(0).isLeftSide ? speedLeftPercent : speedRightPercent); // top left
// 			drivetrain.wheels.at(1).setVelocityPercent(drivetrain.wheels.at(1).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom left
// 			drivetrain.wheels.at(2).setVelocityPercent(drivetrain.wheels.at(2).isLeftSide ? speedLeftPercent : speedRightPercent); // top right
// 			drivetrain.wheels.at(3).setVelocityPercent(drivetrain.wheels.at(3).isLeftSide ? speedLeftPercent : speedRightPercent); // bottom right
// 		}

// 		void brakeWheels() {
// 			drivetrain.wheels.at(0).brake();
// 			drivetrain.wheels.at(1).brake();
// 			drivetrain.wheels.at(2).brake();
// 			drivetrain.wheels.at(3).brake();
// 		}

// 		void moveDrivetrain(float leftSpeedPercent, float rightSpeedPercent) {
// 			drivetrain.wheels.at(0).setVelocityPercent(leftSpeedPercent);
// 			drivetrain.wheels.at(1).setVelocityPercent(leftSpeedPercent);
// 			drivetrain.wheels.at(2).setVelocityPercent(rightSpeedPercent);
// 			drivetrain.wheels.at(3).setVelocityPercent(rightSpeedPercent);
// 		}

// 		void setSpeedFromController(ctrlAPI::Controller &controller) {
// 			float leftSpeedPercent;
// 			float rightSpeedPercent;

// 			if (driveMode == DrivingMode::SINGLE_JOYSTICK) {
// 				const float joystickRadians = atan2(controller.leftAnalogXPercent / 100, controller.leftAnalogYPercent / 100);
// 				const float alteredAnalogY = controller.leftAnalogYPercent * abs(cos(joystickRadians));
// 				const float alteredAnalogX = controller.leftAnalogXPercent * abs(sin(joystickRadians));
// 				leftSpeedPercent = alteredAnalogY + alteredAnalogX;
// 				rightSpeedPercent = alteredAnalogY - alteredAnalogX;
// 			} else {
// 				leftSpeedPercent = controller.leftAnalogYPercent;
// 				rightSpeedPercent = controller.rightAnalogYPercent;
// 			}

// 			moveDrivetrain(leftSpeedPercent, rightSpeedPercent);
// 		}
// };

// } // namespace robotAPI

//| MARK: Just Header
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
	std::array<DrivetrainMotor, 4> wheels;
	Drivetrain();
	Drivetrain(std::array<DrivetrainMotor, 4> _wheels);
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
	void moveDrivetrain(float leftSpeedPercent, float rightSpeedPercent);
	void setSpeedFromController(ctrlAPI::Controller &controller);
};

} // namespace robotAPI