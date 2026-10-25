#include "tcc_trace.h"
#include <string.h>
#include <limits.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_system.h"
#include "esp_random.h"

namespace {
constexpr uint32_t CAP = 512;
TccTraceRecord records[CAP] = {};
portMUX_TYPE trace_mux = portMUX_INITIALIZER_UNLOCKED;
uint64_t boot_id = 0;
std::atomic<bool> boot_ready{false};
uint32_t next_seq = 1;
uint32_t count = 0;
uint32_t overwritten = 0;
uint64_t last_timestamp_us = 0;
inline void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void put32(uint8_t* p, uint32_t v) {
    for (unsigned i = 0; i != 4; ++i) p[i] = v >> (i*8);
}
inline void put64(uint8_t* p, uint64_t v) {
    for (unsigned i = 0; i != 8; ++i) p[i] = v >> (i*8);
}
inline uint32_t get32(const uint8_t* p) {
    return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
inline uint64_t get64(const uint8_t* p) { return uint64_t(get32(p)) | uint64_t(get32(p+4))<<32; }
void ensure_boot() {
    if (boot_ready.load(std::memory_order_acquire)) return;
    uint64_t candidate = (uint64_t(esp_random())<<32) | esp_random();
    if (!candidate) candidate = 1;
    portENTER_CRITICAL(&trace_mux);
    if (!boot_id) {
        boot_id = candidate;
        boot_ready.store(true, std::memory_order_release);
    }
    portEXIT_CRITICAL(&trace_mux);
}
}

void TccTrace::init() { ensure_boot(); }

void TccTrace::append(TccTraceRecord r) {
    portENTER_CRITICAL(&trace_mux);
    r.seq = next_seq++;
    if (last_timestamp_us && r.timestamp_us >= last_timestamp_us) {
        uint64_t dt = r.timestamp_us - last_timestamp_us;
        r.dt_us = dt > UINT32_MAX ? UINT32_MAX : uint32_t(dt);
    } else {
        r.dt_us = 0;
    }
    last_timestamp_us = r.timestamp_us;
    memset(r.reserved_zero, 0, sizeof(r.reserved_zero));
    records[(r.seq-1) % CAP] = r;
    if (count < CAP) ++count;
    else if (overwritten != UINT32_MAX) ++overwritten;
    portEXIT_CRITICAL(&trace_mux);
}

bool TccTrace::read_page(const uint8_t* q, uint8_t* out, size_t capacity, size_t* used) {
    if (!q || !out || !used || capacity < 40 ||
        q[0] != 0x2D || q[1] != 1 || q[2] != 0 ||
        q[3] == 0 || q[3] > 16 || capacity < 40 + size_t(q[3])*72) return false;
    *used = 0;
    ensure_boot();
    const uint64_t wanted_boot = get64(q+4);
    const uint32_t wanted_seq = get32(q+12);
    if (wanted_boot == 0 && wanted_seq != 0) return false;
    portENTER_CRITICAL(&trace_mux);
    if (wanted_boot && wanted_boot != boot_id) {
        portEXIT_CRITICAL(&trace_mux);
        return false;
    }
    const uint32_t oldest = next_seq - count;
    uint32_t cursor = wanted_boot ? wanted_seq : oldest;
    uint8_t flags = 0;
    if (int32_t(cursor - oldest) < 0) { cursor = oldest; flags = 1; }
    if (int32_t(cursor - next_seq) > 0) {
        portEXIT_CRITICAL(&trace_mux);
        return false;
    }
    const uint32_t avail = next_seq - cursor;
    const uint16_t n = avail < q[3] ? uint16_t(avail) : q[3];
    memcpy(out, "TCCT", 4);
    out[4] = 1; out[5] = flags;
    put16(out+6, 40); put16(out+8, 72); put16(out+10, n);
    put64(out+12, boot_id);
    put32(out+20, cursor); put32(out+24, cursor+n);
    put32(out+28, oldest); put32(out+32, next_seq);
    put32(out+36, overwritten);
    for (uint16_t i=0; i<n; ++i) {
        memcpy(out+40+size_t(i)*72, &records[(cursor+i-1)%CAP], 72);
    }
    portEXIT_CRITICAL(&trace_mux);
    *used = 40 + size_t(n)*72;
    return true;
}





