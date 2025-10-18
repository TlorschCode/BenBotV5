#pragma once
#include "globalAPI.h"

namespace robotAPI {
    class Robot;
} // namespace robotAPI

namespace driveAPI {

enum class DrivingMode {
	SINGLE_JOYSTICK,
	TANK
};

class driveInstance {
    private:
    public:
        UnitInterval speedMult = 1.0f;
};


} // namespace driveAPI
