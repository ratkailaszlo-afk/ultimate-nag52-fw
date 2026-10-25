#include "gliding_status.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

namespace {
portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Gliding::Status latest;
void put16(uint8_t* p,uint16_t v) {p[0]=v;p[1]=v>>8;}
void put32(uint8_t* p,uint32_t v) {for(unsigned i=0;i<4;++i)p[i]=v>>(8*i);}
}
void GlidingStatus::publish(const Gliding::Status& status) {
    portENTER_CRITICAL(&status_mux);
    latest=status;
    portEXIT_CRITICAL(&status_mux);
}
bool GlidingStatus::read(uint8_t* out,size_t capacity,size_t* used) {
    if(!out || !used || capacity<WIRE_SIZE)return false;
    Gliding::Status s;
    portENTER_CRITICAL(&status_mux);
    s=latest;
    portEXIT_CRITICAL(&status_mux);
    memset(out,0,WIRE_SIZE);
    memcpy(out,"GLD2",4);out[4]=2;out[5]=uint8_t(s.state);out[6]=uint8_t(s.reason);
    out[7]=uint8_t(s.cruise);put16(out+8,s.flags);out[10]=s.pedal;out[11]=s.gear;
    put16(out+12,s.speed_x100);put16(out+14,uint16_t(s.signed_slip_rpm));
    put16(out+16,s.pressure_mbar);put16(out+18,s.normal_pressure_mbar);
    put32(out+20,s.timestamp_ms);put32(out+24,s.activations);put32(out+28,s.exits);
    put16(out+32,s.recoveries);out[34]=s.speed_control_bits;
    out[35]=s.recovery_phase;out[36]=s.recovery_outcome;out[37]=s.contact_evidence;out[38]=s.normal_mode;
    put16(out+40,s.engine_age_ms);put16(out+42,s.turbine_age_ms);put16(out+44,s.rpm_skew_ms);
    put16(out+46,uint16_t(s.closure_rpm_per_s));put32(out+48,s.slip_work_proxy_j);
    put16(out+52,Gliding::MIN_SPEED_X100);put16(out+54,s.torque_age_ms);
    out[56]=s.city_phase;out[57]=s.city_outcome;out[58]=s.city_contact;out[59]=s.city_guarded;
    put16(out+60,uint16_t(s.city_closure_rpm_per_s));put16(out+62,s.city_work_proxy_j);*used=WIRE_SIZE;return true;
}
