#include "shift_adaptation.h"
#include <esp_log.h>
#include "nvs/eeprom_config.h"
#include "esp_check.h"
#include "nvs/module_settings.h"
#include "common_structs_ops.h"
#include "maps.h"
#include "nvs/all_keys.h"
#include <limits>
#include <new>

namespace {
constexpr uint16_t SHIFT_ADAPTATION_MAP_ELEMENTS = 8;

bool shift_map_is_valid(StoredMap* map) {
    if (map == nullptr || map->init_status() != ESP_OK) {
        return false;
    }
    if (map->get_map_element_count() != SHIFT_ADAPTATION_MAP_ELEMENTS ||
        map->data_size() != SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !map->is_allocated() ||
        map->get_map_name() == nullptr) {
        return false;
    }
    uint16_t x_size = 0;
    uint16_t y_size = 0;
    int16_t* x_headers = nullptr;
    int16_t* y_headers = nullptr;
    map->get_x_headers(&x_size, &x_headers);
    map->get_y_headers(&y_size, &y_headers);
    if (x_size != SHIFT_ADAPTATION_MAP_ELEMENTS || y_size != 1 ||
        x_headers == nullptr || y_headers == nullptr) {
        return false;
    }
    return map->get_current_data() != nullptr;
}

int16_t clamp_i16(int64_t value) {
    if (value < std::numeric_limits<int16_t>::min()) {
        return std::numeric_limits<int16_t>::min();
    }
    if (value > std::numeric_limits<int16_t>::max()) {
        return std::numeric_limits<int16_t>::max();
    }
    return static_cast<int16_t>(value);
}
}

ShiftAdaptationSystem::ShiftAdaptationSystem() {
    const int16_t adpt_map_x[8] = {0,1,2,3,4,5,6,7};
    const int16_t adpt_map_y[1] = {1};
    this->prefill_time_map = new (std::nothrow) StoredMap(
        NVS_KEY_MAP_NAME_ADAPT_PREFILL_TIME, 8, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->applying_torque_offset = new (std::nothrow) StoredMap(
        NVS_KEY_MAP_NAME_ADAPT_APPLYING_TRQ, 8, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->freeing_torque_offset = new (std::nothrow) StoredMap(
        NVS_KEY_MAP_NAME_ADAPT_FREEING_TRQ, 8, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->spc_offset_map = new (std::nothrow) StoredMap(
        NVS_KEY_MAP_NAME_ADAPT_SPC_OFFSET, 8, adpt_map_x, adpt_map_y, 8, 1, GEAR_ADAPT_MAP);
    this->init_shift();
}

void ShiftAdaptationSystem::init_shift() {
    this->init_ok =
        shift_map_is_valid(this->prefill_time_map) &&
        shift_map_is_valid(this->applying_torque_offset) &&
        shift_map_is_valid(this->freeing_torque_offset) &&
        shift_map_is_valid(this->spc_offset_map);
    if (!this->init_ok) {
        ESP_LOGE("ADAPT", "Shift adaptation map initialization/shape/data check failed");
    }
}

esp_err_t ShiftAdaptationSystem::save(void) {
    this->init_shift();
    if (!this->init_ok) {
        return ESP_ERR_INVALID_STATE;
    }

    // Save order is retained. Stop and return the first map error; this is not
    // an atomic multi-map transaction.
    esp_err_t err = this->prefill_time_map->save_to_eeprom();
    if (err != ESP_OK) {
        return err;
    }
    err = this->applying_torque_offset->save_to_eeprom();
    if (err != ESP_OK) {
        return err;
    }
    err = this->freeing_torque_offset->save_to_eeprom();
    if (err != ESP_OK) {
        return err;
    }
    return this->spc_offset_map->save_to_eeprom();
}

int8_t ShiftAdaptationSystem::get_prefill_cycles_offset(uint8_t shift_idx) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->prefill_time_map)) {
        return 0;
    }
    const int16_t value = this->prefill_time_map->get_current_data()[shift_idx];
    if (value > std::numeric_limits<int8_t>::max()) {
        return std::numeric_limits<int8_t>::max();
    }
    if (value < std::numeric_limits<int8_t>::min()) {
        return std::numeric_limits<int8_t>::min();
    }
    return static_cast<int8_t>(value);
}

int16_t ShiftAdaptationSystem::get_adapt_spc_offset(uint8_t shift_idx) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->spc_offset_map)) {
        return 0;
    }
    return this->spc_offset_map->get_current_data()[shift_idx];
}

int16_t ShiftAdaptationSystem::get_freeing_torque_offset(uint8_t shift_idx) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->freeing_torque_offset)) {
        return 0;
    }
    return this->freeing_torque_offset->get_current_data()[shift_idx];
}

int16_t ShiftAdaptationSystem::get_applying_torque_offset(uint8_t shift_idx) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->applying_torque_offset)) {
        return 0;
    }
    return this->applying_torque_offset->get_current_data()[shift_idx];
}

void ShiftAdaptationSystem::offset_prefill_cycles(uint8_t shift_idx, int8_t offset) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->prefill_time_map)) {
        return;
    }
    int16_t* data = this->prefill_time_map->get_current_data();
    const int32_t configured_limit = ADP_CURRENT_SETTINGS.prefill_max_time_delta;
    const int32_t limit = configured_limit > std::numeric_limits<int16_t>::max()
        ? std::numeric_limits<int16_t>::max()
        : configured_limit;
    const int32_t proposed = static_cast<int32_t>(data[shift_idx]) + offset;
    const int32_t bounded = proposed > limit ? limit : (proposed < -limit ? -limit : proposed);
    data[shift_idx] = static_cast<int16_t>(bounded);
    ESP_LOGI("ADAPT", "Prefill cycles offset by %d to %d", offset, data[shift_idx]);
}

void ShiftAdaptationSystem::offset_spc_pressure(uint8_t shift_idx, int32_t offset) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->spc_offset_map)) {
        return;
    }
    int16_t* data = this->spc_offset_map->get_current_data();
    data[shift_idx] = clamp_i16(static_cast<int64_t>(data[shift_idx]) + offset);
    ESP_LOGI("ADAPT", "SPC pressure offset by %ld to %d", static_cast<long>(offset), data[shift_idx]);
}

void ShiftAdaptationSystem::offset_freeing_trq(uint8_t shift_idx, int32_t offset) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->freeing_torque_offset)) {
        return;
    }
    int16_t* data = this->freeing_torque_offset->get_current_data();
    data[shift_idx] = clamp_i16(static_cast<int64_t>(data[shift_idx]) + offset);
    ESP_LOGI("ADAPT", "Free. Trq offset by %ld to %d", static_cast<long>(offset), data[shift_idx]);
}

void ShiftAdaptationSystem::offset_applying_trq(uint8_t shift_idx, int32_t offset) {
    if (shift_idx >= SHIFT_ADAPTATION_MAP_ELEMENTS ||
        !shift_map_is_valid(this->applying_torque_offset)) {
        return;
    }
    int16_t* data = this->applying_torque_offset->get_current_data();
    data[shift_idx] = clamp_i16(static_cast<int64_t>(data[shift_idx]) + offset);
    ESP_LOGI("ADAPT", "Appl. Trq offset by %ld to %d", static_cast<long>(offset), data[shift_idx]);
}

esp_err_t ShiftAdaptationSystem::reset() {
    this->init_shift();
    if (!this->init_ok) {
        return ESP_ERR_INVALID_STATE;
    }

    // Reset order is retained. Stop and return the first map error; this is
    // not an atomic multi-map transaction.
    esp_err_t err = this->prefill_time_map->reset_from_flash();
    if (err != ESP_OK) {
        return err;
    }
    err = this->spc_offset_map->reset_from_flash();
    if (err != ESP_OK) {
        return err;
    }
    err = this->freeing_torque_offset->reset_from_flash();
    if (err != ESP_OK) {
        return err;
    }
    return this->applying_torque_offset->reset_from_flash();
}

ShiftAdaptationSystem* adaptation_manager = nullptr;
