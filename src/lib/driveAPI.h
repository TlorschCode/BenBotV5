#pragma once
#include "globalAPI.h"


namespace ctrlAPI {
    class Controller;
} // namespace ctrlAPI

namespace driveAPI {

enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

class DriveInstance {
    private:
    public:
        UnitInterval speedMult = 1.0f;
        float leftSpeedPercent = 0;
        float rightSpeedPercent = 0;
        DrivingMode mode = DrivingMode::TANK;
        DriveInstance() {}
        DriveInstance(DrivingMode drivingMode) {mode = drivingMode;}
        void setSpeedFromController(ctrlAPI::Controller &controller) {
            if (mode == DrivingMode::SINGLE_JOYSTICK) {
		        const float joystickRadians = atan2(controller.leftAnalogX / 100, controller.leftAnalogY / 100);
		        const float alteredAnalogY = controller.leftAnalogY * abs(cos(joystickRadians));
		        const float alteredAnalogX = controller.leftAnalogX * abs(sin(joystickRadians));
                leftSpeedPercent = alteredAnalogY + alteredAnalogX;
		        rightSpeedPercent = alteredAnalogY - alteredAnalogX;
            } else {
                leftSpeedPercent = controller.leftAnalogY;
		        rightSpeedPercent = controller.rightAnalogY;
            }
        }
};


} // namespace driveAPI
