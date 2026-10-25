#pragma once
#include <stddef.h>
#include <stdint.h>

// Fixed-size, task-context observation only. No actuator, reset or NVS policy.
namespace RuntimeMonitor {
enum class Stage : uint8_t { Gearbox, Tcc, Shift, Can, Adc, Io, Count };
enum class Source : uint8_t { IoCompletion, CanDequeue, AdcCompletion, Count };
enum class Counter : uint8_t {
    SensorInitError, SensorReadError, AtfCalibrationError, AdcInitError,
    AdcReadError, CanRxError, CanBusOff, AdaptationError, Count
};
enum Quality : uint16_t {
    AtfValid = 1u << 0, SpeedsValid = 1u << 1, EngineValid = 1u << 2,
    PedalValid = 1u << 3, Shifting = 1u << 4
};
enum : uint8_t { CpuUnknown = 1, CompletionAges = 2, Saturated = 4 };
constexpr uint8_t kStageCount = uint8_t(Stage::Count);
constexpr uint8_t kCounterCount = uint8_t(Counter::Count);
constexpr uint8_t kSourceCount = uint8_t(Source::Count);
constexpr size_t kWireSize = 32 + kStageCount * 24 + kCounterCount * 4 + kSourceCount * 4;
static_assert(kWireSize == 220, "Runtime monitor v1 ABI");

void init(); // Exactly once, before any producer task starts.
uint64_t now_us();
void record(Stage stage, uint64_t started_us, uint64_t completed_us, uint32_t budget_us = 0);
void source_complete(Source source, uint64_t completed_us);
void fault(Counter counter);
void quality(uint16_t flags);
// Read-only, non-consuming; capacity failure leaves destination and state unchanged.
bool read(uint8_t* output, size_t capacity, size_t* used);
class Scope {
public:
    explicit Scope(Stage stage, uint32_t budget_us = 0)
        : stage_(stage), started_(now_us()), budget_(budget_us) {}
    ~Scope() { finish(); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    void finish() {
        if (!finished_) { record(stage_, started_, now_us(), budget_); finished_ = true; }
    }
private:
    Stage stage_;
    uint64_t started_;
    uint32_t budget_;
    bool finished_ = false;
};
}
