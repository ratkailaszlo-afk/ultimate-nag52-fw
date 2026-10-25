#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <new>
#include "esp_err.h"

class StoredMap {
public:
    StoredMap(const char* name,
              uint16_t element_count,
              const int16_t* x_headers,
              const int16_t* y_headers,
              uint16_t x_size,
              uint16_t y_size,
              const int16_t* default_map)
        : map_name(name),
          element_count(element_count),
          data_element_count(element_count),
          x_size(x_size),
          y_size(y_size),
          x_headers_present(x_headers != nullptr),
          y_headers_present(y_headers != nullptr),
          size(element_count),
          data(element_count == 0 ? nullptr : new (std::nothrow) int16_t[element_count]),
          default_values(element_count == 0 ? nullptr : new (std::nothrow) int16_t[element_count]),
          default_map(default_values),
          init_result(ESP_OK),
          save_result(ESP_OK),
          allow_add_data(true),
          allocated(data != nullptr && default_values != nullptr && x_size * y_size == element_count),
          null_data(false),
          add_data_calls(0),
          save_calls(0) {
        if (element_count != 0 && (data == nullptr || default_values == nullptr || !allocated)) {
            init_result = ESP_ERR_INVALID_STATE;
        }
        if (data != nullptr && default_values != nullptr) {
            for (uint16_t i = 0; i < element_count; ++i) {
                const int16_t value = default_map == nullptr ? 0 : default_map[i];
                data[i] = value;
                default_values[i] = value;
            }
        }
    }

    ~StoredMap() {
        delete[] data;
        delete[] default_values;
    }

    esp_err_t init_status() const { return init_result; }
    uint16_t get_map_element_count() const { return element_count; }
    uint16_t data_size() { return size; }
    bool is_allocated() const { return allocated; }
    const char* get_map_name() const { return map_name; }
    int16_t* get_current_data() const { return null_data ? nullptr : data; }
    void get_x_headers(uint16_t* count, int16_t** headers) {
        if (count != nullptr) *count = x_size;
        if (headers != nullptr) *headers = x_headers_present ? &axis_placeholder : nullptr;
    }
    void get_y_headers(uint16_t* count, int16_t** headers) {
        if (count != nullptr) *count = y_size;
        if (headers != nullptr) *headers = y_headers_present ? &axis_placeholder : nullptr;
    }

    esp_err_t save_to_eeprom() {
        ++save_calls;
        return save_result;
    }

    bool add_data(const int16_t* new_data, uint16_t count) {
        ++add_data_calls;
        if (!allow_add_data || new_data == nullptr || data == nullptr || count != data_element_count) {
            return false;
        }
        std::memcpy(data, new_data, count * sizeof(int16_t));
        return true;
    }

    esp_err_t replace_data_content(const int16_t* new_data, uint16_t content_len);
    esp_err_t reset_from_flash();

    const char* map_name;
    uint16_t element_count;
    uint16_t data_element_count;
    uint16_t x_size;
    uint16_t y_size;
    bool x_headers_present;
    bool y_headers_present;
    uint16_t size;
    int16_t* data;
    int16_t* default_values;
    const int16_t* default_map;
    esp_err_t init_result;
    esp_err_t save_result;
    bool allow_add_data;
    bool allocated;
    bool null_data;
    int add_data_calls;
    int save_calls;
    int16_t axis_placeholder = 0;
};
