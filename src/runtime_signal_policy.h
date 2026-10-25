#pragma once
#include <stdint.h>

// Numerical SensorData caches are not evidence that the current sample is valid.
// A lost engine frame must not be treated as an observed stopped engine.
struct RuntimeSignalPolicy {
    bool engine_valid;
    bool pedal_valid;
    bool speeds_valid;
    bool atf_valid;
    uint16_t engine_rpm;

    bool forward_control_allowed() const {
        return engine_valid && engine_rpm > 100 && pedal_valid && speeds_valid && atf_valid;
    }
    bool engine_stopped() const {
        return engine_valid && engine_rpm <= 100;
    }
    bool new_shift_allowed(bool actual_forward, bool target_forward, bool target_disengaged) const {
        if (!engine_valid || engine_rpm <= 100) return false;
        // Selecting N/P only releases to idle and needs no ATF-dependent fill.
        if (target_disengaged) return true;
        if (!atf_valid) return false;
        // Garage engagements retain their existing hydraulic sequence. A new
        // forward-to-forward shift additionally needs pedal and shaft signals.
        return !(actual_forward && target_forward) || forward_control_allowed();
    }
};
