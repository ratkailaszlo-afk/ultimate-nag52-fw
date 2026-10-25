#pragma once
#include <stdint.h>
#include <limits.h>

// Engineering limits for offline evaluation, not hydraulic calibration.
// Pressure is a post-temperature command. No pressure/contact sensor exists.
namespace Reapply {
constexpr uint32_t MAX_DT_MS=40, FILL_MS=180, TIMEOUT_MS=6000, DWELL_MS=300;
constexpr uint16_t FILL_CAP=600, MAX_RISE=2500, MAX_FALL=8000;
enum class Mode : uint8_t { Open, Slipping, Closed };
enum class Phase : uint8_t { Idle, Fill, Converge, Handoff, Complete, Fallback, Interrupted, NormalOpen };
enum class Outcome : uint8_t { None, Success, Timeout, Interrupted, NormalOpen };
enum class Contact : uint8_t { Unknown, Possible }; // Never a physical confirmation.
struct Sample {
    bool valid=false;
    int32_t slip=0;                 // Matched, aligned engine minus turbine RPM.
    uint32_t timestamp=0;
    int16_t torque=0;               // ECU estimate, not measured clutch torque.
};
inline int32_t magnitude(int32_t v) { return v<0 ? -v : v; }

class Control {
public:
    bool guarded() const { return guard_; }
    bool active() const { return phase_==Phase::Fill || phase_==Phase::Converge || phase_==Phase::Handoff; }
    bool failed() const { return fault_; }
    Phase phase() const { return phase_; }
    Outcome outcome() const { return outcome_; }
    Contact contact() const { return contact_; }
    uint16_t pressure() const { return pressure_; }
    uint16_t normal_pressure() const { return normal_; }
    int32_t closure_rate() const { return closure_; }
    uint32_t work_proxy_j() const { return uint32_t(work_mj_/1000); }
    void start(uint32_t now, uint16_t initial=0) {
        guard_=true;pressure_=initial;normal_=initial;at_=start_=phase_at_=now;
        phase_=initial>FILL_CAP?Phase::Converge:Phase::Fill;outcome_=Outcome::None;contact_=Contact::Unknown;
        sample_seen_=sync_seen_=false;remainder_=0;work_mj_=0;hold_until_=now;
        // A timeout is latched for the ignition session. It cannot become success.
        if(fault_) {phase_=initial>FILL_CAP?Phase::Fallback:Phase::Fill;outcome_=Outcome::Timeout;}
    }
    void interrupt(uint32_t now) {
        if(!guard_) return;
        pressure_=0;at_=now;remainder_=0;sample_seen_=sync_seen_=false;
        phase_=Phase::Interrupted;outcome_=Outcome::Interrupted;contact_=Contact::Unknown;
    }
    uint16_t update(uint16_t normal, Mode mode, uint16_t slip_target, const Sample& s, uint32_t now) {
        const bool target_changed=normal!=normal_ || mode!=last_mode_ || slip_target!=last_slip_target_;
        normal_=normal;last_mode_=mode;last_slip_target_=slip_target;
        if(target_changed) {
            sync_seen_=false;
            if(phase_==Phase::Complete) {phase_=Phase::Converge;start_=now;outcome_=Outcome::None;}
            else if(phase_==Phase::Handoff) phase_=Phase::Converge;
        }
        if(!guard_) {pressure_=normal;at_=now;return pressure_;}
        const uint32_t elapsed=now-at_;
        const uint32_t dt=elapsed>MAX_DT_MS ? MAX_DT_MS : elapsed;
        at_=now;
        // ECU/normal Open and invalid feedback are deliberate safety interruptions.
        if(mode==Mode::Open || normal==0) {
            pressure_=0;remainder_=0;phase_=Phase::NormalOpen;
            outcome_=Outcome::NormalOpen;sync_seen_=sample_seen_=false;contact_=Contact::Unknown;
            return 0;
        }
        if(elapsed==0) return pressure_; // Safety Open above still wins on duplicate ticks.
        if(!s.valid || now-s.timestamp>=100 || elapsed>100) {interrupt(now);return 0;}
        if(phase_==Phase::Interrupted || phase_==Phase::NormalOpen) start(now,0);
        const bool new_sample=!sample_seen_ || s.timestamp!=sample_at_;
        int32_t slope=previous_slope_;
        if(sample_seen_ && new_sample) {
            const uint32_t sample_dt=s.timestamp-sample_at_;
            if(sample_dt==0 || sample_dt>=100) {interrupt(now);return 0;}
            slope=(s.slip-previous_slip_)*1000/int32_t(sample_dt);
            closure_=(magnitude(previous_slip_)-magnitude(s.slip))*1000/int32_t(sample_dt);
            // A sign crossing is expected on some tip-ins; settle, do not label a fault.
            if((s.slip<0)!=(previous_slip_<0) || magnitude(int32_t(s.torque)-previous_torque_)>40 || (s.torque<0)!=(previous_torque_<0))
                hold_until_=now+120;
        } else if(!sample_seen_) {closure_=0;slope=0;}
        if(new_sample) {previous_slip_=s.slip;previous_torque_=s.torque;sample_at_=s.timestamp;previous_slope_=slope;sample_seen_=true;}
        work_mj_+=uint64_t(magnitude(s.slip))*magnitude(s.torque)*dt*105/1000;
        // This proxy includes hydrodynamic loss; it is not clutch energy or ATF heat.
        if(active() && (now-start_>=TIMEOUT_MS || work_mj_>=30000000ULL)) {
            fault_=true;phase_=Phase::Fallback;outcome_=Outcome::Timeout;sync_seen_=false;
        }
        if(phase_==Phase::Fill && now-phase_at_>=FILL_MS) {
            phase_=fault_?Phase::Fallback:Phase::Converge;phase_at_=now;
        }
        if(pressure_>0 && closure_>40 && now-start_>=FILL_MS) contact_=Contact::Possible;
        const int32_t target=mode==Mode::Closed ? 0 :
            (s.torque<0 ? -int32_t(slip_target) : int32_t(slip_target));
        const int32_t error=magnitude(s.slip-target);
        uint16_t demand=normal;
        uint32_t rise=2500;
        if(phase_==Phase::Fill) {demand=normal<FILL_CAP?normal:FILL_CAP;rise=1000;}
        else if(!fault_) {
            // Too rapid convergence: reduce the command slightly; near the target
            // or after a torque/sign transition: pause growth and observe settling.
            if(closure_>700 && magnitude(s.slip)<300) demand=pressure_>30?pressure_-30:0;
            else if((int32_t(now-hold_until_)<0) || (closure_>300 && error>20)) demand=pressure_<normal?pressure_:normal;
            else if(error<=40 && magnitude(slope)>200) demand=pressure_<normal?pressure_:normal;
            else if(error<=40) rise=2000;
        }
        // Completed recovery retains the same common limiter for future map changes.
        if(phase_==Phase::Complete) rise=MAX_RISE;
        if(demand>normal)demand=normal;
        slew(demand,rise,dt);
        if(active() && !fault_ && new_sample) {
            const bool aligned=error<=20 && magnitude(slope)<=200 && pressure_==normal &&
                phase_!=Phase::Fill && int32_t(now-hold_until_)>=0;
            if(aligned) {
                phase_=Phase::Handoff;
                if(!sync_seen_) {sync_seen_=true;sync_at_=now;}
                if(now-sync_at_>=DWELL_MS) {phase_=Phase::Complete;outcome_=Outcome::Success;}
            } else {
                sync_seen_=false;
                if(phase_==Phase::Handoff) phase_=Phase::Converge;
            }
        }
        return pressure_;
    }
private:
    void slew(uint16_t demand,uint32_t rise,uint32_t dt) {
        const bool up=demand>pressure_;
        if(up!=last_up_) remainder_=0;
        last_up_=up;
        const uint32_t budget=dt*(up?rise:MAX_FALL)+remainder_;
        const uint32_t delta=budget/1000;remainder_=budget%1000;
        if(up) pressure_=demand-pressure_>delta?uint16_t(pressure_+delta):demand;
        else pressure_=pressure_-demand>delta?uint16_t(pressure_-delta):demand;
    }
    bool guard_=false,fault_=false,sample_seen_=false,sync_seen_=false,last_up_=false;
    Phase phase_=Phase::Idle;Outcome outcome_=Outcome::None;Contact contact_=Contact::Unknown;
    uint16_t pressure_=0,normal_=0,last_slip_target_=0;
    Mode last_mode_=Mode::Open;
    uint32_t at_=0,start_=0,phase_at_=0,sync_at_=0,hold_until_=0,remainder_=0;
    uint32_t sample_at_=0;
    int32_t previous_slip_=0,closure_=0,previous_slope_=0;int16_t previous_torque_=0;
    uint64_t work_mj_=0;
};
}
