#ifndef C34_COAST_GUARD_H
#define C34_COAST_GUARD_H
#include <stdint.h>

// Comfort lift-off eligibility only; no pressure or shift-time control.
class C34CoastGuard {
public:
    static constexpr uint32_t DEBOUNCE_MS = 300;
    static constexpr uint32_t TREND_WINDOW_MS = 200;
    static constexpr uint32_t MAX_SAMPLE_GAP_MS = 100;
    static constexpr uint32_t MAX_HOLD_MS = 3000;
    static constexpr uint32_t DECEL_OUTPUT_RPM_PER_S = 40;
    static constexpr uint8_t ARM_PEDAL_RAW = 5; // 2% of 0..250.
    void reset() {
        have_sample = false;
        positive_seen = false;
        active = false;
        decelerating = false;
        blocked = false;
    }
    // Called once per controller loop, including inactive/invalid contexts.
    void observe(bool eligible, uint8_t pedal, uint16_t output_rpm, uint32_t now) {
        if (!eligible || pedal > 250 || output_rpm == UINT16_MAX) {
            reset();
            return;
        }
        if (!have_sample || now - sample_time > MAX_SAMPLE_GAP_MS) {
            reset();
            have_sample = true;
            positive_seen = pedal >= ARM_PEDAL_RAW;
            sample_time = now;
            return;
        }
        // A returned pedal releases the hold even if called twice at one clock tick.
        if (pedal != 0) {
            positive_seen = positive_seen || pedal >= ARM_PEDAL_RAW;
            active = false;
            decelerating = false;
            blocked = false;
            sample_time = now;
            return;
        }
        // Duplicate clock ticks must not create a lift edge or accumulate time.
        if (now == sample_time) {
            return;
        }
        sample_time = now;
        if (positive_seen) {
            positive_seen = false;
            active = true;
            decelerating = false;
            lift_time = now;
            trend_time = now;
            trend_output = output_rpm;
        }
        if (!active) {
            blocked = false;
            return;
        }
        const uint32_t since_lift = now - lift_time;
        if (since_lift >= MAX_HOLD_MS) {
            active = false;
            blocked = false;
            return;
        }
        const uint32_t trend_dt = now - trend_time;
        if (trend_dt >= TREND_WINDOW_MS) {
            const int32_t drop = static_cast<int32_t>(trend_output) - output_rpm;
            decelerating = drop > 0 &&
                static_cast<uint32_t>(drop) * 1000 >= DECEL_OUTPUT_RPM_PER_S * trend_dt;
            trend_time = now;
            trend_output = output_rpm;
        }
        blocked = since_lift < DEBOUNCE_MS || decelerating;
        if (!blocked) {
            // A resolved steady coast cannot re-arm at zero pedal.
            active = false;
        }
    }
    bool blocks_upshift() const { return blocked; }
private:
    bool have_sample = false;
    bool positive_seen = false;
    bool active = false;
    bool decelerating = false;
    bool blocked = false;
    uint32_t sample_time = 0;
    uint32_t lift_time = 0;
    uint32_t trend_time = 0;
    uint16_t trend_output = 0;
};
#endif
