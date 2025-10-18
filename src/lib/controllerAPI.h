#pragma once
#include "globalAPI.h"

namespace ctrlAPI {
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

class Controller {
    private:
        static constexpr uint16_t L_R_btnMask = Button::R2 | Button::R1 | Button::L2 | Button::L1;
        static constexpr pros::controller_analog_e_t ControllerAnalogLeftX = pros::E_CONTROLLER_ANALOG_LEFT_X;
        static constexpr pros::controller_analog_e_t ControllerAnalogLeftY = pros::E_CONTROLLER_ANALOG_LEFT_Y;
        static constexpr pros::controller_analog_e_t ControllerAnalogRightX = pros::E_CONTROLLER_ANALOG_RIGHT_X;
        static constexpr pros::controller_analog_e_t ControllerAnalogRightY = pros::E_CONTROLLER_ANALOG_RIGHT_Y;

        static constexpr pros::controller_digital_e_t ControllerDigitalA = pros::E_CONTROLLER_DIGITAL_A;
        static constexpr pros::controller_digital_e_t ControllerDigitalB = pros::E_CONTROLLER_DIGITAL_B;
        static constexpr pros::controller_digital_e_t ControllerDigitalX = pros::E_CONTROLLER_DIGITAL_X;
        static constexpr pros::controller_digital_e_t ControllerDigitalY = pros::E_CONTROLLER_DIGITAL_Y;
        static constexpr pros::controller_digital_e_t ControllerDigitalUp = pros::E_CONTROLLER_DIGITAL_UP;
        static constexpr pros::controller_digital_e_t ControllerDigitalDown = pros::E_CONTROLLER_DIGITAL_DOWN;
        static constexpr pros::controller_digital_e_t ControllerDigitalLeft = pros::E_CONTROLLER_DIGITAL_LEFT;
        static constexpr pros::controller_digital_e_t ControllerDigitalRight = pros::E_CONTROLLER_DIGITAL_RIGHT;
        static constexpr pros::controller_digital_e_t ControllerDigitalR1 = pros::E_CONTROLLER_DIGITAL_R1;
        static constexpr pros::controller_digital_e_t ControllerDigitalR2 = pros::E_CONTROLLER_DIGITAL_R2;
        static constexpr pros::controller_digital_e_t ControllerDigitalL1 = pros::E_CONTROLLER_DIGITAL_L1;
        static constexpr pros::controller_digital_e_t ControllerDigitalL2 = pros::E_CONTROLLER_DIGITAL_L2;
    public:
        pros::Controller rawController;
        uint16_t buttons = 0;
        uint16_t buttonsNewPress = 0;
        float rightAnalogX = {0};
        float rightAnalogY = {0};
        float leftAnalogX = {0};
        float leftAnalogY = {0};
        Controller(pros::Controller _controller) : rawController(_controller) {};
        void updateInputData() {
            buttons = 0;
            buttons = buttons
            | (static_cast<uint16_t>(ctrlAPI::Button::A) * rawController.get_digital(ControllerDigitalA))
            | (static_cast<uint16_t>(ctrlAPI::Button::B) * rawController.get_digital(ControllerDigitalB))
            | (static_cast<uint16_t>(ctrlAPI::Button::X) * rawController.get_digital(ControllerDigitalX))
            | (static_cast<uint16_t>(ctrlAPI::Button::Y) * rawController.get_digital(ControllerDigitalY))
            | (static_cast<uint16_t>(ctrlAPI::Button::Up) * rawController.get_digital(ControllerDigitalUp))
            | (static_cast<uint16_t>(ctrlAPI::Button::Down) * rawController.get_digital(ControllerDigitalDown))
            | (static_cast<uint16_t>(ctrlAPI::Button::Left) * rawController.get_digital(ControllerDigitalLeft))
            | (static_cast<uint16_t>(ctrlAPI::Button::Right) * rawController.get_digital(ControllerDigitalRight))
            | (static_cast<uint16_t>(ctrlAPI::Button::R1) * rawController.get_digital(ControllerDigitalR1))
            | (static_cast<uint16_t>(ctrlAPI::Button::R2) * rawController.get_digital(ControllerDigitalR2))
            | (static_cast<uint16_t>(ctrlAPI::Button::L1) * rawController.get_digital(ControllerDigitalL1))
            | (static_cast<uint16_t>(ctrlAPI::Button::L2) * rawController.get_digital(ControllerDigitalL2));
            buttonsNewPress = 0;
            buttonsNewPress = buttonsNewPress
            | (static_cast<uint16_t>(ctrlAPI::Button::A) * rawController.get_digital_new_press(ControllerDigitalA))
            | (static_cast<uint16_t>(ctrlAPI::Button::B) * rawController.get_digital_new_press(ControllerDigitalB))
            | (static_cast<uint16_t>(ctrlAPI::Button::X) * rawController.get_digital_new_press(ControllerDigitalX))
            | (static_cast<uint16_t>(ctrlAPI::Button::Y) * rawController.get_digital_new_press(ControllerDigitalY))
            | (static_cast<uint16_t>(ctrlAPI::Button::Up) * rawController.get_digital_new_press(ControllerDigitalUp))
            | (static_cast<uint16_t>(ctrlAPI::Button::Down) * rawController.get_digital_new_press(ControllerDigitalDown))
            | (static_cast<uint16_t>(ctrlAPI::Button::Left) * rawController.get_digital_new_press(ControllerDigitalLeft))
            | (static_cast<uint16_t>(ctrlAPI::Button::Right) * rawController.get_digital_new_press(ControllerDigitalRight))
            | (static_cast<uint16_t>(ctrlAPI::Button::R1) * rawController.get_digital_new_press(ControllerDigitalR1))
            | (static_cast<uint16_t>(ctrlAPI::Button::R2) * rawController.get_digital_new_press(ControllerDigitalR2))
            | (static_cast<uint16_t>(ctrlAPI::Button::L1) * rawController.get_digital_new_press(ControllerDigitalL1))
            | (static_cast<uint16_t>(ctrlAPI::Button::L2) * rawController.get_digital_new_press(ControllerDigitalL2));
            rightAnalogX = (rawController.get_analog(ControllerAnalogRightX) / 127) * 100; // 127 is max value from get_analog, turn to percent
            rightAnalogY = (rawController.get_analog(ControllerAnalogRightY) / 127) * 100;
            leftAnalogX = (rawController.get_analog(ControllerAnalogLeftX) / 127) * 100;
            leftAnalogY = (rawController.get_analog(ControllerAnalogLeftY) / 127) * 100;
        }
        inline bool getPressing(Button btn) const {
            return buttons & btn;
        }
        inline bool getNewPress(Button btn) const {
            return buttonsNewPress & btn;
        }
        inline bool otherScoringPressed(Button btn) const {
            return (buttons & L_R_btnMask) & btn;
        }
};
} // namespace controllerAPI