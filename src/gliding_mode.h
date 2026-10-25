#pragma once
#include <stdint.h>
#include <limits.h>
#include "canbus/gliding_inputs.h"

// Offline candidate constants. These are bounded engineering starting values,
// not vehicle-calibrated fill, pressure, temperature or comfort measurements.
namespace Gliding {
constexpr uint32_t CAN_EXPIRY_MS = 100;
constexpr uint16_t MIN_SPEED_X100 = 8000; // Strictly greater than 80 km/h.
constexpr int16_t MIN_ATF_C = 60;
constexpr int16_t MAX_ATF_C = 105;
constexpr uint8_t ARM_PEDAL_MIN = 5;     // 2% of 0..250.
constexpr uint8_t ARM_PEDAL_MAX = 62;    // Less than 25%.
constexpr uint32_t ARM_DWELL_MS = 100;
constexpr uint32_t ARM_MAX_MS = 1500;
constexpr uint32_t RELEASE_CONFIRM_MS = 40;
constexpr uint32_t MAX_COAST_MS = 12000;
constexpr uint32_t TIP_IN_WAIT_MS = 250;
constexpr uint32_t RECOVERY_MAX_MS = 4000;
constexpr uint32_t COOLDOWN_MS = 1500;
constexpr uint32_t MAX_SAMPLE_GAP_MS = 100;
constexpr uint32_t MAX_SLEW_DT_MS = 40;
constexpr uint16_t RISE_MBAR_PER_SECOND = 2500;
constexpr uint16_t FALL_MBAR_PER_SECOND = 8000;
constexpr uint16_t SYNC_RPM = 80;
constexpr uint32_t SYNC_DWELL_MS = 100;

enum class State : uint8_t { Idle, Armed, Opening, Coast, TipIn, Reapply, Paused };
enum class Reason : uint8_t {
    None, CruiseActive, CruiseUnknown, Signals, Gear, Speed, Brake, Dynamics,
    Temperature, Profile, Pedal, Expired, Throttle, Shift, RecoveryTimeout,
    ClockGap, Cooldown, RecoveryComplete
};

struct Context {
    GlidingCanInputs can;
    bool feedback_valid = false;
    bool atf_valid = false;
    bool automatic_profile = false;
    bool shifting = false;
    bool ratio_valid = false;
    uint8_t actual_gear = 0;
    uint8_t target_gear = 0;
    uint16_t speed_x100 = 0;
    uint16_t engine_rpm = 0;
    uint16_t input_rpm = 0;
    int16_t torque_nm = 0;
    int16_t atf_c = 0;
};

inline uint16_t speed_x100(const GlidingCanInputs& can, uint16_t circumference_mm) {
    if (!can.wheels_valid || circumference_mm < 1000 || circumference_mm > 4000)
        return 0;
    const uint32_t left = can.left_wheel_2x_rpm, right = can.right_wheel_2x_rpm;
    const uint32_t mean = (left + right) / 2;
    const uint32_t difference = left > right ? left - right : right - left;
    // Allow ordinary wheel-speed noise/turning; reject a gross disagreement.
    const uint32_t allowance = mean / 20 > 20 ? mean / 20 : 20;
    if (difference > allowance) return 0;
    const uint64_t value = uint64_t(mean) * circumference_mm * 3 / 1000;
    return value > UINT16_MAX ? 0 : uint16_t(value);
}

inline Reason veto(const Context& c) {
    if (c.can.cruise == CruiseControlState::Active) return Reason::CruiseActive;
    if (c.can.cruise != CruiseControlState::Inactive) return Reason::CruiseUnknown;
    if (!c.feedback_valid || !c.atf_valid || !c.can.driver_valid ||
        !c.can.selector_d || !c.can.wheels_valid || c.can.pedal > 250 ||
        c.engine_rpm <= 500 || c.input_rpm == 0)
        return Reason::Signals;
    if (c.shifting) return Reason::Shift;
    if (c.actual_gear != 5 || c.target_gear != 5 || !c.ratio_valid) return Reason::Gear;
    if (c.speed_x100 <= MIN_SPEED_X100) return Reason::Speed;
    if (!c.can.brake_released) return Reason::Brake;
    if (!c.can.dynamics_valid || c.can.dynamics_active) return Reason::Dynamics;
    if (c.atf_c < MIN_ATF_C || c.atf_c > MAX_ATF_C) return Reason::Temperature;
    if (!c.automatic_profile) return Reason::Profile;
    return Reason::None;
}

struct Status {
    State state = State::Idle;
    Reason reason = Reason::None;
    CruiseControlState cruise = CruiseControlState::Unknown;
    uint16_t flags = 0;
    uint8_t pedal = UINT8_MAX;
    uint8_t gear = 0;
    uint16_t speed_x100 = 0;
    int16_t signed_slip_rpm = 0;
    uint16_t pressure_mbar = 0;
    uint16_t normal_pressure_mbar = 0;
    uint32_t timestamp_ms = 0;
    uint32_t activations = 0;
    uint32_t exits = 0;
    uint16_t recoveries = 0;
    uint8_t speed_control_bits = 0;
};

class Controller {
public:
    bool managed() const { return state_ >= State::Opening; }
    bool requests_open() const {
        return state_ == State::Opening || state_ == State::Coast || state_ == State::TipIn;
    }
    bool show_g() const { return show_g_; }

    // Called only by the gearbox/TCC owner, never by the CAN or diagnostic task.
    void observe(const Context* context, uint32_t now) {
        show_g_ = false;
        context_ = context ? *context : Context{};
        const uint32_t gap = sampled_ ? now - last_sample_ : 0;
        last_sample_ = now;
        sampled_ = true;
        if (!context) {
            if (managed()) pause(Reason::Signals);
            else clear_arm(Reason::Signals);
            return;
        }
        const Context& c = context_;
        const Reason blocked = veto(c);
        const bool safe_feedback = c.feedback_valid && c.atf_valid &&
            c.engine_rpm > 400 && c.input_rpm > 0;
        if (gap > MAX_SAMPLE_GAP_MS) {
            if (managed()) pause(Reason::ClockGap);
            else clear_arm(Reason::ClockGap);
            return;
        }
        if (managed()) {
            if (!safe_feedback || c.shifting || c.actual_gear != c.target_gear) {
                pause(c.shifting ? Reason::Shift : Reason::Signals);
                return;
            }
            if (state_ == State::Paused) begin_reapply(now, reason_);
            if (state_ == State::Opening || state_ == State::Coast) {
                if (blocked != Reason::None) begin_reapply(now, blocked);
                else if (c.can.pedal != 0) {
                    state_ = State::TipIn;
                    phase_start_ = now;
                    reason_ = Reason::Throttle;
                    ++exits_;
                } else if (now - coast_start_ >= MAX_COAST_MS) begin_reapply(now, Reason::Expired);
                else if (c.engine_rpm <= 1100 && int32_t(c.input_rpm)-c.engine_rpm >= 150) {
                    state_ = State::Coast;
                    show_g_ = true;
                }
            }
            if (state_ == State::TipIn) {
                if (blocked != Reason::None) begin_reapply(now, blocked);
                else if (absolute_slip(c) <= SYNC_RPM || now-phase_start_ >= TIP_IN_WAIT_MS)
                    begin_reapply(now, Reason::Throttle);
            }
            return;
        }
        if (locked_out_) { clear_arm(Reason::RecoveryTimeout); return; }
        if (blocked != Reason::None) { clear_arm(blocked); return; }
        if (completed_ && now - completed_at_ < COOLDOWN_MS) { clear_arm(Reason::Cooldown); return; }
        const bool mild = c.can.pedal >= ARM_PEDAL_MIN && c.can.pedal <= ARM_PEDAL_MAX;
        if (mild) {
            if (!positive_) { positive_=true; arm_start_=now; }
            release_seen_=false;last_positive_=now;
            if (now-arm_start_ >= ARM_DWELL_MS) { state_=State::Armed; reason_=Reason::None; }
            return;
        }
        if (c.can.pedal != 0) { clear_arm(Reason::Pedal); return; }
        if (state_ != State::Armed || now-last_positive_ > ARM_MAX_MS) {
            clear_arm(Reason::Pedal); return;
        }
        if (!release_seen_) { release_seen_=true; release_at_=now; }
        if (now-release_at_ >= RELEASE_CONFIRM_MS) {
            state_=State::Opening;reason_=Reason::None;coast_start_=now;
            pressure_=0;slew_remainder_=0;slew_at_=now;
            positive_=release_seen_=false;++activations_;
        }
    }

    // Applied to the actual post-temperature-compensation pressure command.
    // A normal/ECU zero command always has priority. No 10000 mbar prefill burst
    // is allowed while managed; the caller bypasses that legacy branch.
    uint16_t limit(uint16_t normal_pressure, uint32_t now) {
        normal_pressure_=normal_pressure;
        if (!managed()) { pressure_=normal_pressure;slew_at_=now;return normal_pressure; }
        if (requests_open() || state_==State::Paused || normal_pressure==0) {
            pressure_=0;slew_remainder_=0;slew_at_=now;return 0;
        }
        const uint32_t elapsed=now-slew_at_;
        const uint32_t dt=elapsed>MAX_SLEW_DT_MS?MAX_SLEW_DT_MS:elapsed;
        slew_at_=now;
        const bool up=normal_pressure>pressure_;
        const uint32_t budget=dt*(up?RISE_MBAR_PER_SECOND:FALL_MBAR_PER_SECOND)+slew_remainder_;
        const uint32_t change=budget/1000;
        slew_remainder_=budget%1000;
        if (up) pressure_=uint16_t(normal_pressure-pressure_>change?pressure_+change:normal_pressure);
        else pressure_=uint16_t(pressure_-normal_pressure>change?pressure_-change:normal_pressure);
        if (absolute_slip(context_)<=SYNC_RPM && pressure_==normal_pressure) {
            if (!sync_seen_) {sync_seen_=true;sync_at_=now;}
            if (now-sync_at_>=SYNC_DWELL_MS) finish(now,Reason::RecoveryComplete);
        } else sync_seen_=false;
        // The timeout may disable future gliding, but never bypasses the slew
        // limit or snaps pressure to the normal map. Handoff requires equality.
        if (now-phase_start_>=RECOVERY_MAX_MS) {
            locked_out_=true;reason_=Reason::RecoveryTimeout;
            if (pressure_==normal_pressure) finish(now,Reason::RecoveryTimeout);
        }
        return pressure_;
    }

    void suspend() {
        show_g_=false;
        context_.feedback_valid=false;
        context_.can.driver_valid=false;
        context_.can.cruise=CruiseControlState::Unknown;
        if (managed()) pause(Reason::Signals);
        else clear_arm(Reason::Signals);
    }

    Status status(uint32_t now) const {
        Status s;
        s.state=state_;s.reason=reason_;s.cruise=context_.can.cruise;
        s.flags=(managed()?1:0)|(requests_open()?2:0)|(show_g_?4:0)|
            (locked_out_?8:0)|(context_.feedback_valid?16:0)|
            (context_.can.driver_valid?32:0)|(context_.can.dynamics_valid?64:0)|
            (context_.can.wheels_valid?128:0)|(context_.can.selector_d?256:0)|
            (context_.can.brake_released?512:0)|(context_.atf_valid?1024:0)|
            (context_.ratio_valid?2048:0)|(context_.automatic_profile?4096:0)|
            (context_.shifting?8192:0);
        s.pedal=context_.can.pedal;s.gear=context_.actual_gear;
        s.speed_x100=context_.speed_x100;
        const int32_t slip=int32_t(context_.engine_rpm)-context_.input_rpm;
        s.signed_slip_rpm=slip>INT16_MAX?INT16_MAX:slip<INT16_MIN?INT16_MIN:int16_t(slip);
        s.pressure_mbar=pressure_;s.normal_pressure_mbar=normal_pressure_;
        s.timestamp_ms=now;s.activations=activations_;s.exits=exits_;s.recoveries=recoveries_;
        s.speed_control_bits=context_.can.speed_control_bits;
        return s;
    }
private:
    static uint32_t absolute_slip(const Context& c) {
        const int32_t v=int32_t(c.engine_rpm)-c.input_rpm;
        return uint32_t(v<0?-v:v);
    }
    void clear_arm(Reason reason) {state_=State::Idle;positive_=release_seen_=false;reason_=reason;}
    void pause(Reason reason) {state_=State::Paused;reason_=reason;pressure_=0;positive_=release_seen_=sync_seen_=false;}
    void begin_reapply(uint32_t now,Reason reason) {
        if (state_==State::Opening || state_==State::Coast) ++exits_;
        state_=State::Reapply;reason_=reason;phase_start_=slew_at_=now;
        pressure_=0;slew_remainder_=0;sync_seen_=false;
        if(recoveries_<UINT16_MAX)++recoveries_;
    }
    void finish(uint32_t now,Reason reason) {
        state_=State::Idle;reason_=reason;completed_=true;completed_at_=now;
        positive_=release_seen_=sync_seen_=false;
    }
    State state_=State::Idle;
    Reason reason_=Reason::None;
    Context context_{};
    bool positive_=false,release_seen_=false,sampled_=false,show_g_=false;
    bool completed_=false,locked_out_=false,sync_seen_=false;
    uint32_t arm_start_=0,last_positive_=0,release_at_=0,coast_start_=0,phase_start_=0;
    uint32_t last_sample_=0,slew_at_=0,completed_at_=0,sync_at_=0;
    uint32_t activations_=0,exits_=0;
    uint16_t recoveries_=0,pressure_=0,normal_pressure_=0;
    uint32_t slew_remainder_=0;
};
}
