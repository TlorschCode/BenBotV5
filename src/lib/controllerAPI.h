#pragma once
#include "globalAPI.h"


namespace ctrlAPI {
enum class Button : uint16_t {
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
}
constexpr inline uint16_t btn_to_u16(ctrlAPI::Button b) noexcept { return static_cast<uint16_t>(b); }
constexpr inline bool btn_to_bool(ctrlAPI::Button b) noexcept { return static_cast<uint16_t>(b) != 0; }
namespace ctrlAPI {
constexpr inline Button operator|(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) | btn_to_u16(rhs));
}
constexpr inline Button operator&(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) & btn_to_u16(rhs));
}
constexpr inline Button operator^(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) ^ btn_to_u16(rhs));
}
constexpr inline Button operator~(Button b) noexcept {
	return static_cast<Button>(~btn_to_u16(b));
}
constexpr inline Button& operator|=(Button& lhs, Button rhs) noexcept { return lhs = lhs | rhs; }
constexpr inline Button& operator&=(Button& lhs, Button rhs) noexcept { return lhs = lhs & rhs; }
constexpr inline Button& operator^=(Button& lhs, Button rhs) noexcept { return lhs = lhs ^ rhs; }
constexpr inline Button operator|(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) | rhs); }
constexpr inline Button operator|(uint16_t lhs, Button rhs) noexcept { return static_cast<Button>(lhs | btn_to_u16(rhs)); }
constexpr inline Button operator&(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) & rhs); }
constexpr inline Button operator&(uint16_t lhs, Button rhs) noexcept { return static_cast<Button>(lhs & btn_to_u16(rhs)); }
constexpr inline Button operator^(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) ^ rhs); }
constexpr inline Button operator^(uint16_t lhs, Button rhs) noexcept { return static_cast<Button>(lhs ^ btn_to_u16(rhs)); }
constexpr inline Button& operator|=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs | rhs; }
constexpr inline Button& operator&=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs & rhs; }
constexpr inline Button& operator^=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs ^ rhs; }
constexpr inline Button operator+(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) + btn_to_u16(rhs));
}
constexpr inline Button operator-(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) - btn_to_u16(rhs));
}
constexpr inline Button operator*(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) * btn_to_u16(rhs));
}
constexpr inline Button operator/(Button lhs, Button rhs) noexcept {
	return static_cast<Button>(btn_to_u16(lhs) / btn_to_u16(rhs));
}
constexpr inline Button& operator+=(Button& lhs, Button rhs) noexcept { return lhs = lhs + rhs; }
constexpr inline Button& operator-=(Button& lhs, Button rhs) noexcept { return lhs = lhs - rhs; }
constexpr inline Button& operator*=(Button& lhs, Button rhs) noexcept { return lhs = lhs * rhs; }
constexpr inline Button& operator/=(Button& lhs, Button rhs) noexcept { return lhs = lhs / rhs; }
constexpr inline Button operator+(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) + rhs); }
constexpr inline Button operator-(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) - rhs); }
constexpr inline Button operator*(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) * rhs); }
constexpr inline Button operator/(Button lhs, uint16_t rhs) noexcept { return static_cast<Button>(btn_to_u16(lhs) / rhs); }
constexpr inline Button& operator+=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs + rhs; }
constexpr inline Button& operator-=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs - rhs; }
constexpr inline Button& operator*=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs * rhs; }
constexpr inline Button& operator/=(Button& lhs, uint16_t rhs) noexcept { return lhs = lhs / rhs; }
constexpr inline bool operator==(Button lhs, Button rhs) noexcept { return btn_to_u16(lhs) == btn_to_u16(rhs); }
constexpr inline bool operator!=(Button lhs, Button rhs) noexcept { return btn_to_u16(lhs) != btn_to_u16(rhs); }
constexpr inline bool operator<(Button lhs, Button rhs)  noexcept { return btn_to_u16(lhs) <  btn_to_u16(rhs); }
constexpr inline bool operator>(Button lhs, Button rhs)  noexcept { return btn_to_u16(lhs) >  btn_to_u16(rhs); }
constexpr inline bool operator<=(Button lhs, Button rhs) noexcept { return btn_to_u16(lhs) <= btn_to_u16(rhs); }
constexpr inline bool operator>=(Button lhs, Button rhs) noexcept { return btn_to_u16(lhs) >= btn_to_u16(rhs); }
constexpr inline bool operator==(Button lhs, uint16_t rhs) noexcept { return btn_to_u16(lhs) == rhs; }
constexpr inline bool operator==(uint16_t lhs, Button rhs) noexcept { return lhs == btn_to_u16(rhs); }
constexpr inline bool operator!=(Button lhs, uint16_t rhs) noexcept { return btn_to_u16(lhs) != rhs; }
constexpr inline bool operator!=(uint16_t lhs, Button rhs) noexcept { return lhs != btn_to_u16(rhs); }
constexpr inline bool operator<(Button lhs, uint16_t rhs)  noexcept { return btn_to_u16(lhs) < rhs; }
constexpr inline bool operator<(uint16_t lhs, Button rhs)  noexcept { return lhs < btn_to_u16(rhs); }
constexpr inline bool operator>(Button lhs, uint16_t rhs)  noexcept { return btn_to_u16(lhs) > rhs; }
constexpr inline bool operator>(uint16_t lhs, Button rhs)  noexcept { return lhs > btn_to_u16(rhs); }
constexpr inline bool operator<=(Button lhs, uint16_t rhs) noexcept { return btn_to_u16(lhs) <= rhs; }
constexpr inline bool operator<=(uint16_t lhs, Button rhs) noexcept { return lhs <= btn_to_u16(rhs); }
constexpr inline bool operator>=(Button lhs, uint16_t rhs) noexcept { return btn_to_u16(lhs) >= rhs; }
constexpr inline bool operator>=(uint16_t lhs, Button rhs) noexcept { return lhs >= btn_to_u16(rhs); }
constexpr inline uint16_t operator+(Button b) noexcept {
	return static_cast<uint16_t>(b);
}
constexpr inline bool operator!(Button b) noexcept {
	return static_cast<uint16_t>(b) == 0;
}

class Controller {
    private:
        static constexpr uint16_t L_R_btnMask = btn_to_u16(Button::R2 | Button::R1 | Button::L2 | Button::L1);
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
        float rightAnalogXPercent = {0};
        float rightAnalogYPercent = {0};
        float leftAnalogXPercent = {0};
        float leftAnalogYPercent = {0};
        Controller(pros::Controller _controller) : rawController(_controller) {};
        Controller() : rawController(pros::Controller(pros::E_CONTROLLER_MASTER)) {};
        void updateInputData() {
            buttons = 0;
            buttons = buttons
            | btn_to_u16((Button::A * rawController.get_digital(ControllerDigitalA))
            | (Button::B * rawController.get_digital(ControllerDigitalB))
            | (Button::X * rawController.get_digital(ControllerDigitalX))
            | (Button::Y * rawController.get_digital(ControllerDigitalY))
            | (Button::Up * rawController.get_digital(ControllerDigitalUp))
            | (Button::Down * rawController.get_digital(ControllerDigitalDown))
            | (Button::Left * rawController.get_digital(ControllerDigitalLeft))
            | (Button::Right * rawController.get_digital(ControllerDigitalRight))
            | (Button::R1 * rawController.get_digital(ControllerDigitalR1))
            | (Button::R2 * rawController.get_digital(ControllerDigitalR2))
            | (Button::L1 * rawController.get_digital(ControllerDigitalL1))
            | (Button::L2 * rawController.get_digital(ControllerDigitalL2)));
            buttonsNewPress = 0;
            buttonsNewPress = buttonsNewPress
            | btn_to_u16((Button::A * rawController.get_digital_new_press(ControllerDigitalA))
            | (Button::B * rawController.get_digital_new_press(ControllerDigitalB))
            | (Button::X * rawController.get_digital_new_press(ControllerDigitalX))
            | (Button::Y * rawController.get_digital_new_press(ControllerDigitalY))
            | (Button::Up * rawController.get_digital_new_press(ControllerDigitalUp))
            | (Button::Down * rawController.get_digital_new_press(ControllerDigitalDown))
            | (Button::Left * rawController.get_digital_new_press(ControllerDigitalLeft))
            | (Button::Right * rawController.get_digital_new_press(ControllerDigitalRight))
            | (Button::R1 * rawController.get_digital_new_press(ControllerDigitalR1))
            | (Button::R2 * rawController.get_digital_new_press(ControllerDigitalR2))
            | (Button::L1 * rawController.get_digital_new_press(ControllerDigitalL1))
            | (Button::L2 * rawController.get_digital_new_press(ControllerDigitalL2)));
            rightAnalogXPercent = (rawController.get_analog(ControllerAnalogRightX) / 127.0f) * 100.0f; // 127 is max value from get_analog, turn to percent
            rightAnalogYPercent = (rawController.get_analog(ControllerAnalogRightY) / 127.0f) * 100.0f;
            leftAnalogXPercent = (rawController.get_analog(ControllerAnalogLeftX) / 127.0f) * 100.0f;
            leftAnalogYPercent = (rawController.get_analog(ControllerAnalogLeftY) / 127.0f) * 100.0f;
        }
        constexpr inline bool getPressing(Button btn) const {
            return btn_to_bool(buttons & btn);
        }
        constexpr inline bool getNewPress(Button btn) const {
            return btn_to_bool(buttonsNewPress & btn);
        }
        constexpr inline bool otherL_or_RPressed(Button btn) const {
            return btn_to_bool((buttons & L_R_btnMask) & btn);
        }
};
} // namespace controllerAPI