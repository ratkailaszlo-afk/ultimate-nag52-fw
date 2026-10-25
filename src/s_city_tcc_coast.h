#ifndef S_CITY_TCC_COAST_H
#define S_CITY_TCC_COAST_H

#include <stdint.h>

// Same-pass gearbox observations only. No CAN query, timer, latch or allocation.
// Exact pedal zero keeps positive-demand target selection unchanged. Braking
// retains the previous TCC policy so this addition does not remove engine brake.
namespace SCityTccCoast {
constexpr uint32_t BRAKE_RELEASE_EXPIRY_MS = 100;

inline bool ratio_is_third(uint16_t input_rpm, uint16_t output_rpm, float lower, float upper) {
    if (input_rpm == 0 || output_rpm <= 250 || !(lower > 0.0f) || !(upper >= lower)) {
        return false;
    }
    const float ratio = float(input_rpm) / float(output_rpm);
    return ratio >= lower && ratio <= upper;
}

struct Context {
    bool standard_profile;
    bool stable_third;
    bool gearbox_shifting;
    bool ratio_agrees;
    bool forward_signals_valid;
    bool raw_speeds_valid;
    bool safe_engine_rpm;
    bool brake_released_valid;
    bool brake_pressed;
    uint8_t raw_pedal;
    uint8_t cached_pedal;
};

inline bool allow(const Context& context) {
    return context.standard_profile && context.stable_third &&
        !context.gearbox_shifting && context.ratio_agrees &&
        context.forward_signals_valid && context.raw_speeds_valid &&
        context.safe_engine_rpm && context.brake_released_valid && !context.brake_pressed &&
        context.raw_pedal == 0 && context.cached_pedal == 0;
}
}

#endif
