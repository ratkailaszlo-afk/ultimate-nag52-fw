#pragma once
#include <stdint.h>
#include <limits.h>
#include "canbus/gliding_inputs.h"
#include "tcc_reapply.h"

// Offline candidate constants. These are bounded engineering starting values,
// not vehicle-calibrated fill, pressure, temperature or comfort measurements.
namespace Gliding {
constexpr uint32_t CAN_EXPIRY_MS = 100;
constexpr uint16_t MIN_SPEED_X100 = 6000; // Latest user steering: strictly greater than 60 km/h.
constexpr int16_t MIN_ATF_C = 60;
constexpr int16_t MAX_ATF_C = 105;
constexpr uint8_t ARM_PEDAL_MIN = 5;     // 2% of 0..250.
constexpr uint8_t ARM_PEDAL_MAX = 62;    // Less than 25%.
constexpr uint32_t ARM_DWELL_MS = 100;
constexpr uint32_t ARM_MAX_MS = 1500;
constexpr uint32_t RELEASE_CONFIRM_MS = 40;
constexpr uint32_t MAX_COAST_MS = 12000;
constexpr uint32_t TIP_IN_WAIT_MS = 250;
constexpr uint32_t RECOVERY_MAX_MS = Reapply::TIMEOUT_MS;
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
    ClockGap, Cooldown, RecoveryComplete, NormalOpen
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
    uint32_t feedback_at_ms = 0;
    uint16_t engine_age_ms = UINT16_MAX, turbine_age_ms = UINT16_MAX, rpm_skew_ms = UINT16_MAX, torque_age_ms=UINT16_MAX;
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
    uint8_t recovery_phase=0, recovery_outcome=0, contact_evidence=0, normal_mode=0;
    uint16_t engine_age_ms=UINT16_MAX,turbine_age_ms=UINT16_MAX,rpm_skew_ms=UINT16_MAX,torque_age_ms=UINT16_MAX;
    uint8_t city_phase=0,city_outcome=0,city_contact=0,city_guarded=0;
    int16_t city_closure_rpm_per_s=0;uint16_t city_work_proxy_j=0;
    int16_t closure_rpm_per_s=0;
    uint32_t slip_work_proxy_j=0;
};

class Controller {
public:
    bool managed() const { return state_ >= State::Opening; }
    bool requests_open() const {
        return state_ == State::Opening || state_ == State::Coast || state_ == State::TipIn;
    }
    bool show_g() const { return show_g_; }
    bool pressure_guarded() const {return managed() || recovery_.guarded();}
    bool control_active() const {return managed() || recovery_.active();}

    // Called only by the gearbox/TCC owner, never by the CAN or diagnostic task.
    void observe(const Context* context, uint32_t now) {
        show_g_ = false;
        context_ = context ? *context : Context{};
        const uint32_t gap = sampled_ ? now - last_sample_ : 0;
        last_sample_ = now;
        sampled_ = true;
        if (!context) {
            if (managed()) pause(Reason::Signals,now);
            else clear_arm(Reason::Signals);
            return;
        }
        const Context& c = context_;
        const Reason blocked = veto(c);
        const bool safe_feedback = c.feedback_valid && c.atf_valid &&
            c.engine_rpm > 400 && c.input_rpm > 0;
        if (gap > MAX_SAMPLE_GAP_MS) {
            if (managed()) pause(Reason::ClockGap,now);
            else clear_arm(Reason::ClockGap);
            return;
        }
        if(recovery_.guarded() && (!safe_feedback || c.shifting || c.actual_gear!=c.target_gear)) {
            pause(c.shifting ? Reason::Shift : Reason::Signals,now);return;
        }
        if (managed()) {
            if (!safe_feedback || c.shifting || c.actual_gear != c.target_gear) {
                pause(c.shifting ? Reason::Shift : Reason::Signals,now);
                return;
            }
            if (state_ == State::Paused) begin_reapply(now, reason_);
            if (state_ == State::Opening || state_ == State::Coast) {
                if (blocked != Reason::None) begin_reapply(now, blocked);
                else if (c.can.pedal != 0) {
                    begin_reapply(now, Reason::Throttle);
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

    // All post-ATF normal targets pass through the retained common guard.
    uint16_t limit(uint16_t normal_pressure, uint32_t now,
                   Reapply::Mode mode=Reapply::Mode::Closed,uint16_t slip_target=0) {
        normal_pressure_=normal_pressure;normal_mode_=mode;
        if(requests_open() || state_==State::Paused) {
            recovery_.interrupt(now);pressure_=0;return 0;
        }
        Reapply::Sample sample;
        sample.valid=context_.feedback_valid && context_.atf_valid;
        sample.slip=int32_t(context_.engine_rpm)-context_.input_rpm;
        sample.torque=context_.torque_nm;sample.timestamp=context_.feedback_at_ms;
        pressure_=recovery_.update(normal_pressure,mode,slip_target,sample,now);
        if(state_==State::Reapply) {
            if(recovery_.failed()) {locked_out_=true;finish(now,Reason::RecoveryTimeout);}
            else if(recovery_.outcome()==Reapply::Outcome::NormalOpen) finish(now,Reason::NormalOpen);
            else if(recovery_.outcome()==Reapply::Outcome::Success) finish(now,Reason::RecoveryComplete);
        }
        return pressure_;
    }

    void suspend(uint32_t now) {
        recovery_.interrupt(now);
        show_g_=false;
        context_.feedback_valid=false;
        context_.can.driver_valid=false;
        context_.can.cruise=CruiseControlState::Unknown;
        if (managed()) pause(Reason::Signals,now);
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
            (context_.shifting?8192:0)|(pressure_guarded()?16384:0)|(control_active()?32768:0);
        s.pedal=context_.can.pedal;s.gear=context_.actual_gear;
        s.speed_x100=context_.speed_x100;
        const int32_t slip=int32_t(context_.engine_rpm)-context_.input_rpm;
        s.signed_slip_rpm=slip>INT16_MAX?INT16_MAX:slip<INT16_MIN?INT16_MIN:int16_t(slip);
        s.pressure_mbar=pressure_;s.normal_pressure_mbar=normal_pressure_;
        s.timestamp_ms=now;s.activations=activations_;s.exits=exits_;s.recoveries=recoveries_;
        s.speed_control_bits=context_.can.speed_control_bits;
        s.recovery_phase=uint8_t(recovery_.phase());s.recovery_outcome=uint8_t(recovery_.outcome());
        s.contact_evidence=uint8_t(recovery_.contact());s.normal_mode=uint8_t(normal_mode_);
        s.engine_age_ms=context_.engine_age_ms;s.turbine_age_ms=context_.turbine_age_ms;
        s.rpm_skew_ms=context_.rpm_skew_ms;s.torque_age_ms=context_.torque_age_ms;
        const int32_t closure=recovery_.closure_rate();
        s.closure_rpm_per_s=closure>INT16_MAX?INT16_MAX:closure<INT16_MIN?INT16_MIN:int16_t(closure);
        s.slip_work_proxy_j=recovery_.work_proxy_j();
        return s;
    }
private:
    static uint32_t absolute_slip(const Context& c) {
        const int32_t v=int32_t(c.engine_rpm)-c.input_rpm;
        return uint32_t(v<0?-v:v);
    }
    void clear_arm(Reason reason) {state_=State::Idle;positive_=release_seen_=false;reason_=reason;}
    void pause(Reason reason,uint32_t now) {recovery_.interrupt(now);state_=State::Paused;reason_=reason;pressure_=0;positive_=release_seen_=sync_seen_=false;}
    void begin_reapply(uint32_t now,Reason reason) {
        if (state_==State::Opening || state_==State::Coast) ++exits_;
        state_=State::Reapply;reason_=reason;phase_start_=slew_at_=now;
        pressure_=0;slew_remainder_=0;sync_seen_=false;recovery_.start(now);
        if(recoveries_<UINT16_MAX)++recoveries_;
    }
    void finish(uint32_t now,Reason reason) {
        state_=State::Idle;reason_=reason;completed_=true;completed_at_=now;
        positive_=release_seen_=sync_seen_=false;
    }
    Reapply::Control recovery_;
    Reapply::Mode normal_mode_=Reapply::Mode::Open;
    State state_=State::Idle;
    Reason reason_=Reason::None;
    Context context_{};
    bool positive_=false,release_seen_=false,sampled_=false,show_g_=false;
    bool completed_=false,locked_out_=false,sync_seen_=false;
    uint32_t arm_start_=0,last_positive_=0,release_at_=0,coast_start_=0,phase_start_=0;
    uint32_t last_sample_=0,slew_at_=0,completed_at_=0;
    uint32_t activations_=0,exits_=0;
    uint16_t recoveries_=0,pressure_=0,normal_pressure_=0;
    uint32_t slew_remainder_=0;
};
}
