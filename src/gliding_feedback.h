#pragma once
#include <stdint.h>
#include <limits.h>

namespace Gliding {
// Small history aligns the polled raw turbine to CAN reception time. Reception
// time is not ECU measurement time; unknown sensor/source latency remains.
class Feedback {
public:
    bool update(bool valid,uint16_t engine,uint32_t engine_at,uint16_t turbine,
                uint32_t turbine_at,uint32_t now,uint16_t& out_engine,uint16_t& out_turbine) {
        if(!valid || now-engine_at>=100 || now-turbine_at>=100 ||
           engine==UINT16_MAX || turbine==UINT16_MAX) {reset();return false;}
        if(!have_poll_ || turbine_at!=last_poll_) {
            samples_[next_]={turbine_at,turbine};next_=(next_+1)%8;
            if(count_<8)++count_;
            last_poll_=turbine_at;have_poll_=true;
        }
        uint32_t best=UINT32_MAX;uint16_t input=0;
        for(unsigned i=0;i<count_;++i) {
            const int32_t d=int32_t(samples_[i].at-engine_at);
            const uint32_t distance=d<0?uint32_t(-d):uint32_t(d);
            if(distance<best) {best=distance;input=samples_[i].rpm;selected_at_=samples_[i].at;}
        }
        skew_=best>UINT16_MAX?UINT16_MAX:uint16_t(best);
        if(best>30 || now-selected_at_>=100) return false;
        if(!have_engine_ || engine_at!=last_engine_) {
            if(!have_engine_ || engine_at-last_engine_>=100) {engine_f_=int32_t(engine)*100;input_f_=int32_t(input)*100;}
            else {engine_f_+=(int32_t(engine)*100-engine_f_)/3;input_f_+=(int32_t(input)*100-input_f_)/3;}
            last_engine_=engine_at;have_engine_=true;
        }
        out_engine=uint16_t(engine_f_/100);out_turbine=uint16_t(input_f_/100);
        return true;
    }
    void reset() {count_=next_=0;have_poll_=have_engine_=false;skew_=UINT16_MAX;}
    uint16_t skew() const {return skew_;}
    uint32_t selected_at() const {return selected_at_;}
private:
    struct Point {uint32_t at=0;uint16_t rpm=0;} samples_[8];
    unsigned count_=0,next_=0;bool have_poll_=false,have_engine_=false;
    uint32_t last_poll_=0,last_engine_=0,selected_at_=0;int32_t engine_f_=0,input_f_=0;
    uint16_t skew_=UINT16_MAX;
};
}
