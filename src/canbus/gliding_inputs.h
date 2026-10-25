#pragma once
#include <stdint.h>

// Unknown is an explicit veto. A missing CAN frame is never cruise-off proof.
enum class CruiseControlState : uint8_t { Unknown = 0, Inactive = 1, Active = 2 };

struct GlidingCanInputs {
    uint32_t received_at_ms[7] = {};
    bool rpm_valid=false, torque_valid=false;
    uint16_t engine_rpm=UINT16_MAX;
    bool driver_valid = false;
    uint8_t pedal = UINT8_MAX;
    CruiseControlState cruise = CruiseControlState::Unknown;
    uint8_t speed_control_bits = 0; // Raw cruise/limiter/ART veto bits for diagnostics.
    bool dynamics_valid = false;
    bool dynamics_active = true;
    bool brake_released = false;
    bool selector_d = false;
    bool wheels_valid = false;
    uint16_t left_wheel_2x_rpm = UINT16_MAX;
    uint16_t right_wheel_2x_rpm = UINT16_MAX;
};
