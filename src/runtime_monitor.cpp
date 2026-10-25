#include "runtime_monitor.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <limits.h>
#include <string.h>

namespace {
struct StageStats {
    uint64_t previous_start_us;
    uint64_t completed_us;
    uint32_t count, last_duration_us, max_duration_us, max_period_us, execution_budget_overruns;
};
struct State {
    StageStats stages[RuntimeMonitor::kStageCount];
    uint64_t source_completed_us[RuntimeMonitor::kSourceCount];
    uint64_t boot_id;
    uint32_t counters[RuntimeMonitor::kCounterCount];
    uint32_t updates;
    uint16_t quality;
    uint8_t seen_sources;
    uint8_t flags;
};
static State state{};
static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
static_assert(sizeof(State) < 1024, "Monitor static state budget");

uint32_t bounded(uint64_t value) { return value > UINT32_MAX ? UINT32_MAX : uint32_t(value); }
void increment(uint32_t& value) {
    if (value < UINT32_MAX) ++value;
    else state.flags |= RuntimeMonitor::Saturated;
}
void put16(uint8_t*& p, uint16_t value) {
    *p++ = uint8_t(value); *p++ = uint8_t(value >> 8);
}
void put32(uint8_t*& p, uint32_t value) {
    for (unsigned n = 0; n < 4; ++n) *p++ = uint8_t(value >> (n * 8));
}
void put64(uint8_t*& p, uint64_t value) {
    for (unsigned n = 0; n < 8; ++n) *p++ = uint8_t(value >> (n * 8));
}
uint32_t age(uint64_t now, uint64_t timestamp) {
    return now >= timestamp ? bounded(now - timestamp) : UINT32_MAX;
}
}

void RuntimeMonitor::init() {
    // No lock needed: app_main calls before constructing/starting other tasks.
    memset(&state, 0, sizeof(state));
    state.boot_id = (uint64_t(esp_random()) << 32) | esp_random();
    if (state.boot_id == 0) state.boot_id = 1;
    state.flags = CpuUnknown | CompletionAges;
}
uint64_t RuntimeMonitor::now_us() { return uint64_t(esp_timer_get_time()); }

void RuntimeMonitor::record(Stage stage, uint64_t started, uint64_t completed, uint32_t budget) {
    const unsigned idx = unsigned(stage);
    if (idx >= kStageCount || completed < started) return;
    const uint32_t duration = bounded(completed - started);
    portENTER_CRITICAL(&mutex);
    StageStats& s = state.stages[idx];
    if (s.count && started >= s.previous_start_us) {
        const uint32_t period = bounded(started - s.previous_start_us);
        if (period > s.max_period_us) s.max_period_us = period;
    }
    s.previous_start_us = started;
    s.completed_us = completed;
    s.last_duration_us = duration;
    if (duration > s.max_duration_us) s.max_duration_us = duration;
    if (budget && duration > budget) increment(s.execution_budget_overruns);
    increment(s.count);
    increment(state.updates);
    portEXIT_CRITICAL(&mutex);
}
void RuntimeMonitor::source_complete(Source source, uint64_t completed) {
    const unsigned idx = unsigned(source);
    if (idx >= kSourceCount) return;
    portENTER_CRITICAL(&mutex);
    state.source_completed_us[idx] = completed;
    state.seen_sources |= uint8_t(1u << idx);
    increment(state.updates);
    portEXIT_CRITICAL(&mutex);
}
void RuntimeMonitor::fault(Counter counter) {
    const unsigned idx = unsigned(counter);
    if (idx >= kCounterCount) return;
    portENTER_CRITICAL(&mutex);
    increment(state.counters[idx]);
    increment(state.updates);
    portEXIT_CRITICAL(&mutex);
}
void RuntimeMonitor::quality(uint16_t flags) {
    portENTER_CRITICAL(&mutex);
    state.quality = flags;
    increment(state.updates);
    portEXIT_CRITICAL(&mutex);
}

bool RuntimeMonitor::read(uint8_t* output, size_t capacity, size_t* used) {
    if (!output || !used || capacity < kWireSize) return false;
    State copy;
    portENTER_CRITICAL(&mutex);
    copy = state;
    portEXIT_CRITICAL(&mutex);
    // Timestamp AFTER copying: updates racing the read cannot appear future-dated.
    const uint64_t now = now_us();
    uint8_t* p = output;
    put16(p, 0x4d52); // ASCII RM
    *p++ = 1;
    *p++ = kStageCount;
    put16(p, uint16_t(kWireSize));
    *p++ = kCounterCount;
    *p++ = kSourceCount;
    put64(p, copy.boot_id);
    put64(p, now);
    put32(p, copy.updates);
    *p++ = copy.seen_sources;
    *p++ = copy.flags;
    put16(p, copy.quality);
    for (const StageStats& s : copy.stages) {
        put32(p, s.count); put32(p, s.last_duration_us); put32(p, s.max_duration_us);
        put32(p, s.max_period_us); put32(p, s.execution_budget_overruns);
        put32(p, s.count ? age(now, s.completed_us) : UINT32_MAX);
    }
    for (uint32_t counter : copy.counters) put32(p, counter);
    for (unsigned n = 0; n < kSourceCount; ++n)
        put32(p, (copy.seen_sources & (1u << n)) ? age(now, copy.source_completed_us[n]) : UINT32_MAX);
    *used = size_t(p - output);
    return *used == kWireSize;
}
