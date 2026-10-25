#include "solenoids.h"
#include "runtime_monitor.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "../sensors.h"
#include "soc/i2s_periph.h"
#include "string.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "../nvs/module_settings.h"
#include "clock.hpp"
#include "esp_timer.h"
#include "tcu_io/tcu_io.hpp"
#include <atomic>

extern adc_cali_handle_t adc1_cal;

OnOffSolenoid* sol_y3 = nullptr;
OnOffSolenoid* sol_y4 = nullptr;
OnOffSolenoid* sol_y5 = nullptr;

ConstantCurrentSolenoid* sol_mpc = nullptr;
ConstantCurrentSolenoid* sol_spc = nullptr;
InrushControlSolenoid* sol_tcc = nullptr;

#define NUM_SOLENOIDS 6
struct SolenoidOutputSummary {
    uint64_t peak_total[NUM_SOLENOIDS];
    uint16_t count_peak[NUM_SOLENOIDS];
};

/*
6 channels
each channel:
    200 samples per 'spike' (1000hz)
    record 2 spikes, and get average
    ~2400000sps = 400000sps per solenoid
*/
#define I2S_DMA_BUF_LEN 6 * 200 * SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2
uint8_t adc_read_buf[I2S_DMA_BUF_LEN];
bool first_read_complete = false;
uint64_t isr_done = 0;
uint8_t CHANNEL_ID_MAP[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
adc_continuous_handle_t solenoid_adc_handle = nullptr;
bool solenoid_adc_started = false;
std::atomic<bool> solenoid_workers_ready{false};
// The TCC GPTimer callback stores `this` as user_data and this class has no
// stop/unregister destructor. Once solenoid objects have been published, keep
// them alive until reset if startup later fails. A failed startup is terminal
// for this boot; callers must not retry against partially initialized drivers.
static bool solenoid_init_failed = false;

static esp_err_t latch_solenoid_init_failure(esp_err_t status) {
    solenoid_workers_ready.store(false, std::memory_order_release);
    solenoid_init_failed = true;
    ESP_LOGE("SOLENOID", "Initialization failed (%s); retaining published solenoid objects until reset and blocking retry",
             esp_err_to_name(status));
    return status;
}

static void cleanup_solenoid_calibration() {
    if (nullptr != adc1_cal) {
        const esp_err_t ret = adc_cali_delete_scheme_line_fitting(adc1_cal);
        if (ESP_OK != ret) {
            ESP_LOGE("SOLENOID", "Failed to release ADC calibration handle: %s", esp_err_to_name(ret));
        } else {
            adc1_cal = nullptr;
        }
    }
}

static esp_err_t cleanup_solenoid_adc_capture() {
    if (nullptr == solenoid_adc_handle) {
        solenoid_adc_started = false;
        return ESP_OK;
    }
    esp_err_t cleanup_status = ESP_OK;
    if (solenoid_adc_started) {
        const esp_err_t stop_status = adc_continuous_stop(solenoid_adc_handle);
        if (ESP_OK != stop_status) {
            ESP_LOGE("SOLENOID", "Failed to stop ADC capture during init cleanup: %s", esp_err_to_name(stop_status));
            cleanup_status = stop_status;
        } else {
            solenoid_adc_started = false;
        }
    }
    const esp_err_t deinit_status = adc_continuous_deinit(solenoid_adc_handle);
    if (ESP_OK != deinit_status) {
        ESP_LOGE("SOLENOID", "Failed to release ADC capture handle: %s", esp_err_to_name(deinit_status));
        if (ESP_OK == cleanup_status) cleanup_status = deinit_status;
    } else {
        solenoid_adc_handle = nullptr;
        solenoid_adc_started = false;
    }
    return cleanup_status;
}

static esp_err_t setup_solenoid_adc_capture() {
    if (nullptr != solenoid_adc_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    const PwmSolenoid* const sol_order[NUM_SOLENOIDS] = {
        sol_mpc, sol_spc, sol_y3, sol_y4, sol_y5, sol_tcc
    };
    const adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = I2S_DMA_BUF_LEN * 2,
        .conv_frame_size = I2S_DMA_BUF_LEN,
    };
    solenoid_adc_handle = nullptr;
    solenoid_adc_started = false;
    esp_err_t ret = adc_continuous_new_handle(&handle_cfg, &solenoid_adc_handle);
    if (ESP_OK != ret) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        ESP_LOGE("SOLENOID", "Failed to create ADC capture handle: %s", esp_err_to_name(ret));
        cleanup_solenoid_adc_capture();
        return ret;
    }

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = { 0 };
    memset(CHANNEL_ID_MAP, 0xFF, sizeof(CHANNEL_ID_MAP));
    for (int i = 0; i < NUM_SOLENOIDS; i++) {
        const uint8_t channel = static_cast<uint8_t>(sol_order[i]->get_adc_channel());
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
        CHANNEL_ID_MAP[channel & 0xF] = static_cast<uint8_t>(i);
    }
    const adc_continuous_config_t dig_cfg = {
        .pattern_num = NUM_SOLENOIDS,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 732000 * 2, // Real freq is 600000hz. (Bug with IDF 5.1) 2000000
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ret = adc_continuous_config(solenoid_adc_handle, &dig_cfg);
    if (ESP_OK != ret) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        ESP_LOGE("SOLENOID", "Failed to configure ADC capture: %s", esp_err_to_name(ret));
        cleanup_solenoid_adc_capture();
        return ret;
    }
    ret = adc_continuous_start(solenoid_adc_handle);
    if (ESP_OK != ret) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        ESP_LOGE("SOLENOID", "Failed to start ADC capture: %s", esp_err_to_name(ret));
        // A failing start may have partially activated the unit; try stop + deinit.
        solenoid_adc_started = true;
        cleanup_solenoid_adc_capture();
        return ret;
    }
    solenoid_adc_started = true;
    return ESP_OK;
}

void read_solenoids_i2s(void*) {
    PwmSolenoid* const sol_order[6] = { sol_mpc, sol_spc, sol_y3, sol_y4, sol_y5, sol_tcc };
    while (!solenoid_workers_ready.load(std::memory_order_acquire)) {
        vTaskDelay(1);
    }
    esp_err_t ret;
    uint32_t read_len;
    while (true) {
        // Process solenoid info
        SolenoidOutputSummary s = {
            .peak_total = {0,0,0,0,0,0},
            .count_peak = {0,0,0,0,0,0},
        };
        // Runs every 4ms or so
        ret = adc_continuous_read(solenoid_adc_handle, adc_read_buf, I2S_DMA_BUF_LEN, &read_len, portMAX_DELAY);
        if (ESP_OK != ret || read_len == 0 || read_len > I2S_DMA_BUF_LEN ||
            (read_len % SOC_ADC_DIGI_RESULT_BYTES) != 0) {
            RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcReadError);
            continue;
        }
        if (ESP_OK == ret) {
            RuntimeMonitor::Scope adc_work(RuntimeMonitor::Stage::Adc);
            for (int i = 0; i < read_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                // adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_read_buf[i];
                adc_digi_output_data_t* p = reinterpret_cast<adc_digi_output_data_t*>(&adc_read_buf[i]);
                uint8_t channel_idx = CHANNEL_ID_MAP[p->type1.channel];
                if (channel_idx != 0xFF) {
                    if (p->type1.data != 0) {
                        s.peak_total[channel_idx] += p->type1.data;
                        s.count_peak[channel_idx] += 1;
                    }
                }
            }

            for (int i = 0; i < 6; i++) {
                if (s.count_peak[i] > 0) {
                    sol_order[i]->__set_adc_reading((float)s.peak_total[i] / (float)s.count_peak[i]);
                }
                else {
                    sol_order[i]->__set_adc_reading(0);
                }
            }
            sol_mpc->update_when_reading(voltage);
            sol_spc->update_when_reading(voltage);
            RuntimeMonitor::source_complete(RuntimeMonitor::Source::AdcCompletion, RuntimeMonitor::now_us());
        }
    }
}

bool write_pwm = true;

void Solenoids::notify_diag_test_start() {
    sol_mpc->set_current_target(0);
    sol_spc->set_current_target(0);
    sol_tcc->set_duty(0);
    sol_y3->off();
    sol_y4->off();
    sol_y5->off();
    vTaskDelay(20);
    write_pwm = false;
}

void Solenoids::notify_diag_test_end(void) {
    write_pwm = true;
}

void update_solenoids(void*) {
    while (!solenoid_workers_ready.load(std::memory_order_acquire)) {
        vTaskDelay(1);
    }
    int16_t atf_temp = 250;
    float vref_compensation = 1.0;
    float temp_compensation = 1.0;
    uint16_t vbatt = TCUIO::battery_mv();
    int16_t atf = TCUIO::atf_temperature();
    while (true) {
        vbatt = TCUIO::battery_mv();
        atf = TCUIO::atf_temperature();
        if (UINT16_MAX != vbatt) {
            voltage = vbatt;
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        }
        else {
            vref_compensation = 1.0;
        }
        // Keep the last good compensation while the selected ATF source is invalid.
        if (TCUIO::atf_temperature_valid() && INT16_MAX != atf) {
            atf_temp = atf * 10.0;
            temp_compensation = (((atf_temp - (SOL_CURRENT_SETTINGS.cc_reference_temp * 10.0)) / 10.0) * SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires) / 10.0;
        }
        if (write_pwm) {
            // MOVED TO CURRENT READING TASK SO READINGS ARE SYNCED
            sol_tcc->__write_pwm(vref_compensation, temp_compensation);
            sol_y3->__write_pwm(vref_compensation, temp_compensation);
            sol_y4->__write_pwm(vref_compensation, temp_compensation);
            sol_y5->__write_pwm(vref_compensation, temp_compensation);
        }
        vTaskDelay(1); // Max we can do at 1000hz
    }
}

float resistance_mpc = 5.0;
float resistance_spc = 5.0;
bool temp_cal = false;
int16_t temp_at_test = 25;

bool routine = false;
bool startup_ok = false;


uint16_t Solenoids::get_solenoid_voltage(void) {
    return voltage;
}

bool Solenoids::init_routine_completed(void) {
    return routine;
}

// bool Solenoids::startup_test_ok() {
//     return startup_ok;
// }

void Solenoids::boot_solenoid_test(void*) {
    bool tcc_new_mode = VEHICLE_CONFIG.io_0_usage == 3 && BOARD_CONFIG.board_ver == 3;
    while (!first_read_complete) { vTaskDelay(1); }
    if (sol_spc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "SPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if (sol_mpc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "MPC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if (!tcc_new_mode && sol_tcc->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "TCC drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if (sol_y3->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y3 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if (sol_y4->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y4 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }

    if (sol_y5->get_current() > SOL_CURRENT_SETTINGS.current_threshold_error) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "SOLENOID", "Y5 drawing too much current when off!");
        routine = true;
        startup_ok = false;
        return;
    }
    startup_ok = true;
    routine = true;
    vTaskDelete(NULL);
}

esp_err_t Solenoids::init_all_solenoids()
{
    if (solenoid_init_failed || nullptr != solenoid_adc_handle || nullptr != adc1_cal ||
        nullptr != sol_tcc || nullptr != sol_mpc || nullptr != sol_spc ||
        nullptr != sol_y3 || nullptr != sol_y4 || nullptr != sol_y5) {
        return ESP_ERR_INVALID_STATE;
    }
    solenoid_workers_ready.store(false, std::memory_order_release);
    const esp_err_t adc_cal_status = SolenoidSetup::init_adc();
    if (ESP_OK != adc_cal_status) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        cleanup_solenoid_calibration();
        return latch_solenoid_init_failure(adc_cal_status);
    }
    // Read calibration for ADC1
    sol_y3 = new OnOffSolenoid("Y3", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y3_pwm, ledc_channel_t::LEDC_CHANNEL_0, ADC_CHANNEL_0, 250, 1524, 1);
    sol_y4 = new OnOffSolenoid("Y4", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y4_pwm, ledc_channel_t::LEDC_CHANNEL_1, ADC_CHANNEL_3, 250, 1524, 1);
    sol_y5 = new OnOffSolenoid("Y5", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->y5_pwm, ledc_channel_t::LEDC_CHANNEL_2, ADC_CHANNEL_7, 250, 1524, 1);
    sol_mpc = new ConstantCurrentSolenoid("MPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 1);
    sol_spc = new ConstantCurrentSolenoid("SPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->spc_pwm, ledc_channel_t::LEDC_CHANNEL_4, ADC_CHANNEL_4, 1);

    // ~700mA for TCC solenoid when holding
    gpio_num_t gpio_zener = GPIO_NUM_NC;
    if (VEHICLE_CONFIG.io_0_usage == 3 && BOARD_CONFIG.board_ver == 3) {
        gpio_zener = pcb_gpio_matrix->io_pin;
    }
    sol_tcc = new InrushControlSolenoid("TCC", ledc_timer_t::LEDC_TIMER_1, pcb_gpio_matrix->tcc_pwm, gpio_zener, ledc_channel_t::LEDC_CHANNEL_5, ADC_CHANNEL_5, 100, 700, 10);
    const struct {
        PwmSolenoid* solenoid;
        const char* name;
    } init_checks[] = {
        {sol_tcc, "TCC"}, {sol_mpc, "MPC"}, {sol_spc, "SPC"},
        {sol_y3, "Y3"}, {sol_y4, "Y4"}, {sol_y5, "Y5"}
    };
    for (const auto& check : init_checks) {
        const esp_err_t status = check.solenoid->init_ok();
        if (ESP_OK != status) {
            ESP_LOGE("SOLENOID", "%s init failed: %s", check.name, esp_err_to_name(status));
            // TCC may already have registered an active GPTimer callback with
            // user_data=this. Keep every globally published object and its ADC
            // calibration alive; there is no verified timer teardown API here.
            return latch_solenoid_init_failure(status);
        }
    }

    esp_err_t ret = setup_solenoid_adc_capture();
    if (ESP_OK != ret) {
        return latch_solenoid_init_failure(ret);
    }

    TaskHandle_t update_task = nullptr;
    if (pdPASS != xTaskCreate(update_solenoids, "LEDC-Update", 8192, nullptr, 10, &update_task) ||
        nullptr == update_task) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        cleanup_solenoid_adc_capture();
        return latch_solenoid_init_failure(ESP_ERR_NO_MEM);
    }
    TaskHandle_t read_task = nullptr;
    if (pdPASS != xTaskCreatePinnedToCore(read_solenoids_i2s, "I2S-Reader", 2048, nullptr, 3, &read_task, 1) ||
        nullptr == read_task) {
        RuntimeMonitor::fault(RuntimeMonitor::Counter::AdcInitError);
        vTaskDelete(update_task);
        cleanup_solenoid_adc_capture();
        return latch_solenoid_init_failure(ESP_ERR_NO_MEM);
    }
    // Release both workers only after the ADC and both task creations succeeded.
    solenoid_workers_ready.store(true, std::memory_order_release);
    return ESP_OK;
}
