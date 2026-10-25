#pragma once
#include <stdint.h>

// Narrow S/A automatic 3->4 decision guard. Does not delay downshifts, manual
// requests, range restrictions or hydraulic shifts already in progress.
class City34Guard {
public:
    bool allow(bool eligible,bool requested,uint8_t pedal,bool kickdown,
               uint16_t predicted_fourth,uint16_t fourth_down_limit,uint32_t now) {
        const bool falling=seen_ && previous_pedal_>pedal+5;
        const bool gap=seen_ && now-last_>100;
        last_=now;seen_=true;previous_pedal_=pedal;
        if(!eligible || !requested || pedal==0 || pedal>250 || kickdown || falling || gap ||
           predicted_fourth<=uint32_t(fourth_down_limit)+80) {pending_=false;return false;}
        if(!pending_) {pending_=true;since_=now;}
        return now-since_>=150;
    }
    void reset(){pending_=seen_=false;}
private:
    bool pending_=false,seen_=false;uint8_t previous_pedal_=0;
    uint32_t since_=0,last_=0;
};
