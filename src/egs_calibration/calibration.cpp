#include "calibration_structs.h"
#include "tcu_alloc.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "string.h"
#include <stddef.h>

CalibrationInfo* CAL_RAM_PTR = NULL;
HydraulicCalibration* HYDR_PTR = NULL;
MechanicalCalibration* MECH_PTR = NULL;
TorqueConverterCalibration* TCC_CFG_PTR = NULL;
ShiftAlgorithmPack* SHIFT_ALGO_CFG_PTR = NULL;

uint16_t crc(const uint8_t* buffer, uint16_t len) {
    uint16_t res = 0;
    for(uint16_t i = 0; i < len; i++) {
        res += i;
        res += buffer[i];
    }
    return res;
}

// Read serialized little-endian words by byte address so packed calibration
// fields never require an aligned uint16_t load.
static uint16_t packed_u16(const void* value) {
    const uint8_t* bytes = static_cast<const uint8_t*>(value);
    return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

static int16_t packed_i16(const void* value) {
    const uint16_t raw = packed_u16(value);
    int16_t result;
    memcpy(&result, &raw, sizeof(result));
    return result;
}

static bool ordered_u8_axis(const uint8_t* axis, size_t count) {
    for (size_t i = 1; i < count; ++i) {
        if (axis[i] <= axis[i - 1]) {
            return false;
        }
    }
    return true;
}

static bool ordered_u16_axis(const void* raw_axis, size_t count) {
    const uint8_t* axis = static_cast<const uint8_t*>(raw_axis);
    if (count == 0) {
        return false;
    }
    uint16_t previous = packed_u16(axis);
    for (size_t i = 1; i < count; ++i) {
        const uint16_t current = packed_u16(axis + i * sizeof(uint16_t));
        if (current <= previous) {
            return false;
        }
        previous = current;
    }
    return true;
}

static bool ordered_i16_axis(const void* raw_axis, size_t count) {
    const uint8_t* axis = static_cast<const uint8_t*>(raw_axis);
    if (count == 0) {
        return false;
    }
    int16_t previous = packed_i16(axis);
    for (size_t i = 1; i < count; ++i) {
        const int16_t current = packed_i16(axis + i * sizeof(int16_t));
        if (current <= previous) {
            return false;
        }
        previous = current;
    }
    return true;
}

static esp_err_t validate_calibration_candidate(const CalibrationInfo* candidate) {
    if (candidate == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (candidate->magic != 0xDEADBEEFu) {
        return ESP_ERR_INVALID_VERSION;
    }
    if (candidate->len != sizeof(CalibrationInfo)) {
        ESP_LOGE("CAL", "Calibration load failed. Length mismatch. Length at info is %d, calibration size is %d", (int)candidate->len, (int)sizeof(CalibrationInfo));
        return ESP_ERR_INVALID_SIZE;
    }
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(candidate);
    const uint16_t crc_calculated = crc(bytes + 8, sizeof(CalibrationInfo) - 8);
    if (crc_calculated != candidate->crc) {
        ESP_LOGE("CAL", "Calibration load failed. CRC error. Wanted %04X, got %04X", crc_calculated, candidate->crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Gearbox::Gearbox divides by all five forward and both reverse ratios.
    // ratio_table[0] is not read by the current runtime.
    for (size_t i = 1; i < 8; ++i) {
        if (packed_u16(&candidate->mech_cal.ratio_table[i]) == 0) {
            ESP_LOGE("CAL", "Calibration load failed. Used gear ratio %d is zero", (int)i);
            return ESP_ERR_INVALID_ARG;
        }
    }
    // Forward gear reductions must descend for the fixed 1..5 gear indexing.
    for (size_t i = 2; i <= 5; ++i) {
        if (packed_u16(&candidate->mech_cal.ratio_table[i - 1]) <= packed_u16(&candidate->mech_cal.ratio_table[i])) {
            ESP_LOGE("CAL", "Calibration load failed. Forward ratio order is invalid at gear %d", (int)i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Divisors used by ShiftingAlgorithm::correct_shift_shift_pressure.
    for (size_t i = 0; i < 8; ++i) {
        if (packed_u16(&candidate->hydr_cal.shift_spc_gain[i]) == 0) {
            ESP_LOGE("CAL", "Calibration load failed. Shift SPC gain %d is zero", (int)i);
            return ESP_ERR_INVALID_ARG;
        }
    }
    // ReleasingShift::calc_cycles_mod_phase1 does not use indices 4 or 5
    // as divisors on the reachable shift paths; those entries remain unchecked.
    const size_t turbine_drag_divisor_indices[] = {0, 1, 2, 3, 6, 7};
    for (size_t i = 0; i < sizeof(turbine_drag_divisor_indices) / sizeof(turbine_drag_divisor_indices[0]); ++i) {
        const size_t idx = turbine_drag_divisor_indices[i];
        if (packed_u16(&candidate->mech_cal.turbine_drag[idx]) == 0) {
            ESP_LOGE("CAL", "Calibration load failed. Turbine drag divisor %d is zero", (int)idx);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // The pressure manager divides by these multipliers in its active
    // line-pressure and factor calculations.
    if (packed_u16(&candidate->hydr_cal.p_multi_1) == 0 ||
        packed_u16(&candidate->hydr_cal.p_multi_other) == 0) {
        ESP_LOGE("CAL", "Calibration load failed. Pressure multiplier divisor is zero");
        return ESP_ERR_INVALID_ARG;
    }

    // The helper divides by every shift inertia factor. It is screened even
    // though the current source has no call site for this helper.
    for (size_t i = 0; i < 8; ++i) {
        if (packed_u16(&candidate->mech_cal.intertia_factor[i]) == 0) {
            ESP_LOGE("CAL", "Calibration load failed. Shift inertia factor %d is zero", (int)i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // calc_max_torque_for_clutch divides by these active apply-clutch map
    // cells for the eight fixed forward shifts. Other cells are intentionally
    // sparse: a zero marks a clutch that is not loaded in that gear.
    const uint8_t active_apply_friction_indices[8] = {12, 19, 26, 33, 9, 14, 22, 24};
    for (size_t i = 0; i < sizeof(active_apply_friction_indices) / sizeof(active_apply_friction_indices[0]); ++i) {
        const size_t idx = active_apply_friction_indices[i];
        if (packed_u16(&candidate->mech_cal.friction_map[idx]) == 0) {
            ESP_LOGE("CAL", "Calibration load failed. Active apply-clutch friction divisor %d is zero", (int)idx);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // strongest_loaded_clutch_idx indexes release_spring_pressure[6].
    // 0..5 are clutch indices; 0xFF is the explicit no-clutch sentinel.
    for (size_t i = 0; i < 8; ++i) {
        const uint8_t idx = candidate->mech_cal.strongest_loaded_clutch_idx[i];
        if (idx >= 6 && idx != 0xFF) {
            ESP_LOGE("CAL", "Calibration load failed. Loaded-clutch index %d is invalid", (int)idx);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Static MPC pressure selects the strongest clutch for the current gear and
    // calc_max_torque_for_clutch divides by that gear/clutch friction cell.
    for (size_t gear_idx = 1; gear_idx < 8; ++gear_idx) {
        const uint8_t clutch_idx = candidate->mech_cal.strongest_loaded_clutch_idx[gear_idx];
        if (clutch_idx < 6) {
            const size_t friction_idx = (gear_idx * 6) + clutch_idx;
            if (packed_u16(&candidate->mech_cal.friction_map[friction_idx]) == 0) {
                ESP_LOGE("CAL", "Calibration load failed. Static MPC friction divisor %d is zero", (int)friction_idx);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    // Every active Lookup*Map axis is required to be strictly increasing.
    // Payload values may legitimately be zero and are intentionally not screened.
    const TorqueConverterCalibration* tcc = &candidate->tcc_cal;
    if (!ordered_u16_axis(tcc->multiplier_map_x, 2) ||
        !ordered_u16_axis(tcc->pump_map_x, 11)) {
        ESP_LOGE("CAL", "Calibration load failed. Torque-converter axis is not ordered");
        return ESP_ERR_INVALID_ARG;
    }

    const HydraulicCalibration* hydr = &candidate->hydr_cal;
    // PressureManager constructs LookupRefMap with int16_t axis pointers.
    if (!ordered_i16_axis(hydr->pcs_map_x, 7) ||
        !ordered_i16_axis(hydr->pcs_map_y, 4)) {
        ESP_LOGE("CAL", "Calibration load failed. Hydraulic map axis is not ordered");
        return ESP_ERR_INVALID_ARG;
    }

    const ShiftAlgorithmPack* shift = &candidate->shift_algo_cal;
#define CAL_AXIS_OK(axis) ordered_u8_axis((axis), sizeof(axis) / sizeof((axis)[0]))
    if (!CAL_AXIS_OK(shift->momentum_1_2_x) || !CAL_AXIS_OK(shift->momentum_2_3_x) ||
        !CAL_AXIS_OK(shift->momentum_3_4_x) || !CAL_AXIS_OK(shift->momentum_4_5_x) ||
        !CAL_AXIS_OK(shift->momentum_1_2_y) || !CAL_AXIS_OK(shift->momentum_2_3_y) ||
        !CAL_AXIS_OK(shift->momentum_3_4_y) || !CAL_AXIS_OK(shift->momentum_4_5_y) ||
        !CAL_AXIS_OK(shift->momentum_2_1_x) || !CAL_AXIS_OK(shift->momentum_3_2_x) ||
        !CAL_AXIS_OK(shift->momentum_4_3_x) || !CAL_AXIS_OK(shift->momentum_5_4_x) ||
        !CAL_AXIS_OK(shift->momentum_2_1_y) || !CAL_AXIS_OK(shift->momentum_3_2_y) ||
        !CAL_AXIS_OK(shift->momentum_4_3_y) || !CAL_AXIS_OK(shift->momentum_5_4_y) ||
        !CAL_AXIS_OK(shift->trq_adder_1_2_x) || !CAL_AXIS_OK(shift->trq_adder_2_3_x) ||
        !CAL_AXIS_OK(shift->trq_adder_3_4_x) || !CAL_AXIS_OK(shift->trq_adder_4_5_x) ||
        !CAL_AXIS_OK(shift->trq_adder_1_2_y) || !CAL_AXIS_OK(shift->trq_adder_2_3_y) ||
        !CAL_AXIS_OK(shift->trq_adder_3_4_y) || !CAL_AXIS_OK(shift->trq_adder_4_5_y) ||
        !CAL_AXIS_OK(shift->trq_adder_2_1_x) || !CAL_AXIS_OK(shift->trq_adder_3_2_x) ||
        !CAL_AXIS_OK(shift->trq_adder_4_3_x) || !CAL_AXIS_OK(shift->trq_adder_5_4_x) ||
        !CAL_AXIS_OK(shift->trq_adder_2_1_y) || !CAL_AXIS_OK(shift->trq_adder_3_2_y) ||
        !CAL_AXIS_OK(shift->trq_adder_4_3_y) || !CAL_AXIS_OK(shift->trq_adder_5_4_y)) {
        ESP_LOGE("CAL", "Calibration load failed. Shift-algorithm axis is not ordered");
        return ESP_ERR_INVALID_ARG;
    }
#undef CAL_AXIS_OK

    return ESP_OK;
}

esp_err_t EGSCal::init_egs_calibration() {
    // Initialization is one-shot; do not replace any active calibration block.
    if (CAL_RAM_PTR != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    CalibrationInfo* candidate = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
    if (candidate == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_flash_read(nullptr, static_cast<void*>(candidate), CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
    if (ret == ESP_OK) {
        ret = validate_calibration_candidate(candidate);
    }
    if (ret != ESP_OK) {
        TCU_FREE(candidate);
        return ret;
    }

    // Publish the initial block only after the candidate passes every check.
    CAL_RAM_PTR = candidate;
    HYDR_PTR = &candidate->hydr_cal;
    MECH_PTR = &candidate->mech_cal;
    TCC_CFG_PTR = &candidate->tcc_cal;
    SHIFT_ALGO_CFG_PTR = &candidate->shift_algo_cal;
    return ESP_OK;
}

esp_err_t EGSCal::reload_egs_calibration() {
    if (CAL_RAM_PTR == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    CalibrationInfo* tmp = reinterpret_cast<CalibrationInfo*>(TCU_HEAP_ALLOC(sizeof(CalibrationInfo)));
    if (tmp == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_flash_read(nullptr, static_cast<void*>(tmp), CALIBRATION_START_ADDRESS, sizeof(CalibrationInfo));
    if (ret == ESP_OK) {
        ret = validate_calibration_candidate(tmp);
    }
    if (ret == ESP_OK) {
        // Preserve the active block and all published subobject pointers.
        memcpy(CAL_RAM_PTR, tmp, sizeof(CalibrationInfo));
    }
    TCU_FREE(tmp);
    return ret;
}