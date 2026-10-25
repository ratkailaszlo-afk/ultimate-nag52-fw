#pragma once
#include <stdint.h>
#include <stddef.h>

// Versioned wire layout: see ../tcc_trace_protocol.md. No old diagnostic struct changes.
struct __attribute__((packed)) TccTraceRecord {
    uint32_t seq;
    uint64_t timestamp_us;
    uint32_t dt_us;
    uint32_t flags;
    uint8_t path, profile, actual_gear, target_gear;
    uint8_t state_before, state_after, target_before, target_after;
    uint8_t prefill_flags_before, prefill_flags_after;
    uint8_t prefill_cycles_before, prefill_cycles_after;
    uint16_t engine_rpm_at_tcc, input_rpm_at_tcc, output_rpm_at_tcc;
    int16_t converted_torque_at_tcc, atf_c_at_tcc;
    uint8_t pedal_at_tcc, selected_can_request_bits;
    int16_t signed_engine_minus_input_rpm, filtered_absolute_slip_rpm;
    uint16_t slip_target_before_rpm, slip_target_after_rpm;
    uint16_t command_before_atf_mbar, command_after_atf_mbar;
    uint16_t pm_requested_mbar, pm_final_pwm_raw;
    int32_t filtered_model_pressure_centimbar;
    uint8_t pm_path;
    uint8_t pedal_smoothed_at_tcc, coast_raw_pedal;
    uint16_t coast_engine_rpm;
    uint8_t quality_flags;
    uint8_t reserved_zero[2];
};
static_assert(sizeof(TccTraceRecord) == 72, "TCC trace ABI record size");
namespace TccTrace {
    enum : uint32_t {
        PREFILL_BRANCH = 1u<<0, SHIFTING = 1u<<1, RELEASE = 1u<<2,
        UPSHIFT = 1u<<3, WAS_SHIFTING = 1u<<4, INPUT_VALID = 1u<<5,
        PM_VALID = 1u<<6, UPDATE_SKIPPED = 1u<<7, CAN_FRESHNESS_UNKNOWN = 1u<<8,
        SIGNED_SLIP_SATURATED = 1u<<9, PRESSURE_SATURATED = 1u<<10,
        FILTERED_SLIP_SATURATED = 1u<<11
    };
    // The producer calls append exactly once per main gearbox loop; read_page never consumes.
    void init(); // Boot nonce generated once before controller loop.
    void append(TccTraceRecord record);
    bool read_page(const uint8_t* request16, uint8_t* response, size_t capacity, size_t* used);
}




