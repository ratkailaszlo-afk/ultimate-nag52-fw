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
    memcpy(out,"GLD1",4);out[4]=1;out[5]=uint8_t(s.state);out[6]=uint8_t(s.reason);
    out[7]=uint8_t(s.cruise);put16(out+8,s.flags);out[10]=s.pedal;out[11]=s.gear;
    put16(out+12,s.speed_x100);put16(out+14,uint16_t(s.signed_slip_rpm));
    put16(out+16,s.pressure_mbar);put16(out+18,s.normal_pressure_mbar);
    put32(out+20,s.timestamp_ms);put32(out+24,s.activations);put32(out+28,s.exits);
    put16(out+32,s.recoveries);out[34]=s.speed_control_bits;*used=WIRE_SIZE;return true;
}
