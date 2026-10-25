#include "gearbox.h"
#include "tcc_trace.h"
#include "s_city_tcc_coast.h"
#include "runtime_monitor.h"
#include "runtime_signal_policy.h"
#include "gliding_status.h"
#include "esp_timer.h"
#include "common_structs_ops.h"
#include "nvs/eeprom_config.h"
#include "adv_opts.h"
#include <tcu_maths.h>
#include "speaker.h"
#include "clock.hpp"
#include "nvs/device_mode.h"
#include "egs_calibration/calibration_structs.h"
#include "shifting_algo/s_algo.h"
#include "shifting_algo/shift_crossover.h"
#include "shifting_algo/shift_release.h"
#include "tcu_io/tcu_io.hpp"

#define SBS SBS_CURRENT_SETTINGS

const uint8_t AVG_SAMPLES_500MS = 500 / 20;

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(const int output_rpm, const GearboxGear req_gear, const GearboxConfiguration* gb_config)
{
    int calculated = output_rpm;
    switch (req_gear)
    {
    case GearboxGear::First:
        calculated *= gb_config->bounds[0].ratio;
        break;
    case GearboxGear::Second:
        calculated *= gb_config->bounds[1].ratio;
        break;
    case GearboxGear::Third:
        calculated *= gb_config->bounds[2].ratio;
        break;
    case GearboxGear::Fourth:
        calculated *= gb_config->bounds[3].ratio;
        break;
    case GearboxGear::Fifth:
        calculated *= gb_config->bounds[4].ratio;
        break;
    default:
        break;
    }
    return calculated;
}

Gearbox::Gearbox(Shifter* shifter) : shifter(shifter), kickdown(), brake_pedal()
{
    TccTrace::init();
    this->current_profile = nullptr;
    egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
    this->sensor_data = SensorData{
        .input_rpm = 0,
        .engine_rpm = 0,
        .output_rpm = 0,
        .pedal_pos = 0,
        .pedal_pos_smoothed = 0,
        .atf_temp = 0,
        .input_torque = 0,
        .converted_torque = 0,
        .converted_driver_torque = 0,
        .indicated_torque = 0,
        .max_torque = 0,
        .min_torque = 0,
        .last_shift_time = 0,
        .gear_ratio = 0.0F,
        .kickdown_pressed = false,
        .brake_pressed = false,
    };
    this->output_data = OutputData{
        .torque_req_amount = 0,
        .ctrl_type = TorqueRequestControlType::None,
        .bounds = TorqueRequestBounds::LessThan,
    };

    float r1 = ((float)(MECH_PTR->ratio_table[1])) / 1000.0;
    float r2 = ((float)(MECH_PTR->ratio_table[2])) / 1000.0;
    float r3 = ((float)(MECH_PTR->ratio_table[3])) / 1000.0;
    float r4 = ((float)(MECH_PTR->ratio_table[4])) / 1000.0;
    float r5 = ((float)(MECH_PTR->ratio_table[5])) / 1000.0;
    float rr1 = ((float)(MECH_PTR->ratio_table[6]) * -1) / 1000.0;
    float rr2 = ((float)(MECH_PTR->ratio_table[7]) * -1) / 1000.0;

    this->gearboxConfig.max_torque = 330;
    if (MECH_PTR->gb_ty == 0) {
        this->gearboxConfig.max_torque = 580;
    }
    this->gearboxConfig.bounds[0] = GearRatioInfo{ // 1st 
        .ratio_max_drift = r1 * (float)1.1,
        .ratio = r1,
        .ratio_min_drift = r1 * (float)0.9,
    };
    this->gearboxConfig.bounds[1] = GearRatioInfo{ // 2nd 
        .ratio_max_drift = r2 * (float)1.1,
        .ratio = r2,
        .ratio_min_drift = r2 * (float)0.9,
    };
    this->gearboxConfig.bounds[2] = GearRatioInfo{ // 3rd 
        .ratio_max_drift = r3 * (float)1.1,
        .ratio = r3,
        .ratio_min_drift = r3 * (float)0.9,
    };
    this->gearboxConfig.bounds[3] = GearRatioInfo{ // 4th 
        .ratio_max_drift = r4 * (float)1.1,
        .ratio = r4,
        .ratio_min_drift = r4 * (float)0.9,
    };
    this->gearboxConfig.bounds[4] = GearRatioInfo{ // 5th 
        .ratio_max_drift = r5 * (float)1.1,
        .ratio = r5,
        .ratio_min_drift = r5 * (float)0.9,
    };
    this->gearboxConfig.bounds[5] = GearRatioInfo{ // R1 
        .ratio_max_drift = rr1 * (float)1.1,
        .ratio = rr1,
        .ratio_min_drift = rr1 * (float)0.9,
    };
    this->gearboxConfig.bounds[6] = GearRatioInfo{ // R2 
        .ratio_max_drift = rr2 * (float)1.1,
        .ratio = rr2,
        .ratio_min_drift = rr2 * (float)0.9,
    };
    // IMPORTANT - Set the Ratio2/Ratio1 multiplier for the sensor RPM reading algorithm!
    TCUIO::set_2_1_ratio(r1 / r2);

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    this->tcc = new TorqueConverter(this->gearboxConfig.max_torque);
    this->shift_adapter = new ShiftAdaptationSystem();
    pressure_manager = this->pressure_mgr;
    adaptation_manager = this->shift_adapter;
    // Wait for solenoid routine to complete
    if (!Solenoids::init_routine_completed())
    {
        vTaskDelay(1);
    }
    if (VEHICLE_CONFIG.engine_type == 1)
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_petrol;
    }
    else
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_diesel;
    }
    if (this->redline_rpm < 4000)
    {
        this->redline_rpm = 4000; // just in case
    }
    this->diff_ratio_f = (float)VEHICLE_CONFIG.diff_ratio / 1000.0;
    this->input_rpm_delta = new DeltaTracker(25);
    this->pedal_delta = new DeltaTracker(25);
}

bool Gearbox::is_stationary() {
    return this->sensor_data.input_rpm < 100 && this->sensor_data.output_rpm < 100;
}

void Gearbox::set_profile(AbstractProfile* prof)
{
    if ((nullptr != prof) && ((nullptr == current_profile) || (prof != current_profile)))
    {
        // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
        portEXIT_CRITICAL(&this->profile_mutex);
    }
}

esp_err_t Gearbox::start_controller()
{
    xTaskCreatePinnedToCore(Gearbox::start_controller_internal, "GEARBOX", 32768, static_cast<void*>(this), 10, nullptr, 1);
    return ESP_OK;
}

GearboxGear gear_from_idx(uint8_t idx)
{
    // Only for drivable gears. P/R/SNV is never used
    GearboxGear ret = GearboxGear::SignalNotAvailable;
    if (likely(idx >= 1 && idx <= 5)) {
        ret = (GearboxGear)(idx); // Direct cast for gears 1-5
    }
    else if (idx == 6) {
        ret = GearboxGear::Reverse_First;
    }
    else if (idx == 7) {
        ret = GearboxGear::Reverse_Second;
    }
    return ret;
}

bool is_controllable_gear(GearboxGear g)
{
    bool controllable = true;
    if (unlikely(g == GearboxGear::Park || g == GearboxGear::Neutral || g == GearboxGear::SignalNotAvailable)) {
        controllable = false;
    }
    return controllable;
}

float ratio_absolute(GearboxGear g, GearboxConfiguration* cfg) {
    switch (g) {
    case GearboxGear::First:
        return cfg->bounds[0].ratio;
    case GearboxGear::Second:
        return cfg->bounds[1].ratio;
    case GearboxGear::Third:
        return cfg->bounds[2].ratio;
    case GearboxGear::Fourth:
        return cfg->bounds[3].ratio;
    case GearboxGear::Fifth:
        return cfg->bounds[4].ratio;
    case GearboxGear::Reverse_First:
        return abs(cfg->bounds[5].ratio);
    case GearboxGear::Reverse_Second:
        return abs(cfg->bounds[6].ratio);
    default:
        return 0;
    }
}

bool is_fwd_gear(GearboxGear g)
{
    bool is_fwd = false;
    if (likely((uint8_t)g >= 1 && (uint8_t)g <= 5)) {
        is_fwd = true;
    }
    return is_fwd;
}

const char* gear_to_text(GearboxGear g)
{
    switch (g)
    {
    case GearboxGear::First:
        return "D1";
    case GearboxGear::Second:
        return "D2";
    case GearboxGear::Third:
        return "D3";
    case GearboxGear::Fourth:
        return "D4";
    case GearboxGear::Fifth:
        return "D5";
    case GearboxGear::Reverse_First:
        return "R1";
    case GearboxGear::Reverse_Second:
        return "R2";
    case GearboxGear::Park:
        return "P";
    case GearboxGear::SignalNotAvailable:
        return "SNA";
    case GearboxGear::Neutral:
        return "N";
    default:
        return "";
    }
}

void Gearbox::inc_gear_request()
{
    this->ask_upshift = true;
    this->ask_downshift = false;
    this->manual_shift = true;
}

void Gearbox::dec_gear_request()
{
    this->ask_upshift = false;
    this->ask_downshift = true;
    this->manual_shift = true;
}

void Gearbox::set_torque_request(TorqueRequestControlType ctrl_type, TorqueRequestBounds bounds, float amount) {
    this->output_data.torque_req_amount = amount;
    this->output_data.ctrl_type = ctrl_type;
    this->output_data.bounds = bounds;
    egs_can_hal->set_torque_request(ctrl_type, bounds, amount);
}

GearboxGear next_gear(GearboxGear g)
{
    GearboxGear next = g;
    uint8_t idx = (uint8_t)g;
    if (idx >= 1 && idx < 5) { // 1-4
        next = (GearboxGear)(idx + 1);
    }
    return next;
}

GearboxGear prev_gear(GearboxGear g)
{
    GearboxGear prev = g;
    uint8_t idx = (uint8_t)g;
    if (idx > 1 && idx <= 5) { // 2-5
        prev = (GearboxGear)(idx - 1);
    }
    return prev;
}

#define SHIFT_DELAY_MS 20     // 20ms steps
#define NUM_SCD_ENTRIES 100 / SHIFT_DELAY_MS // 100ms moving average window

ClutchSpeeds Gearbox::diag_get_clutch_speeds()
{

    return ClutchSpeedModel::get_clutch_speeds_debug(
        this->speed_sensors,
        this->last_motion_gear,
        this->actual_gear,
        this->target_gear,
        this->gearboxConfig.bounds
    );
}

ShiftReportSegment Gearbox::collect_report_segment(uint64_t start_time) {
    return ShiftReportSegment{
        .static_torque = sensor_data.converted_torque,
        .driver_torque = sensor_data.converted_driver_torque,
        .egs_req_torque = (int16_t)((this->output_data.ctrl_type == TorqueRequestControlType::None) ? INT16_MAX : (int16_t)(this->output_data.torque_req_amount)),
        .engine_rpm = sensor_data.engine_rpm,
        .input_rpm = sensor_data.input_rpm,
        .output_rpm = sensor_data.output_rpm,
        .mpc_pressure = 0, //this->pressure_mgr->get_targ_mpc_clutch_pressure(),
        .spc_pressure = 0, //this->pressure_mgr->get_targ_spc_clutch_pressure(),
        .timestamp = (uint16_t)(GET_CLOCK_TIME() - start_time)
    };
}

/**
 * @brief Used to shift between forward gears
 *
 * @return uint16_t - The actual time taken to shift gears. This is fed back into the adaptation network so it can better meet 'target_shift_duration_ms'
 */

bool Gearbox::elapse_shift(GearChange req_lookup, AbstractProfile* profile, bool manually_requested)
{
    bool result = false;
    // This is important for all EGS compatibility
    uint8_t egs_map_idx_lookup = fwd_gearchange_egs_map_lookup_idx(req_lookup);

    if (nullptr != profile && req_lookup != GearChange::_IDLE && 0xFF != egs_map_idx_lookup)
    {
        ShiftCharacteristics chars = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
        chars.target_shift_time = MAX(100, chars.target_shift_time);
        CircuitInfo sd = pressure_mgr->get_basic_shift_data(&this->gearboxConfig, req_lookup, chars);
        sd.map_idx = egs_map_idx_lookup;
        if (this->last_shift_circuit == sd.shift_circuit) { // Same shift solenoid
            while (GET_CLOCK_TIME() - sensor_data.last_shift_time < 500) {
                vTaskDelay(10);
            }
        }
        this->last_shift_circuit = sd.shift_circuit;
        bool process_shift = true;

        ShiftPressures p_now = {};
        memset(&p_now, 0, sizeof(ShiftPressures));

        uint32_t total_elapsed = 0;
        uint32_t phase_elapsed = 0;

        pressure_manager->register_shift_pressure_data(&p_now);

        ShiftClutchData now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, req_lookup, this->gearboxConfig.bounds);
        Clutch applying = get_clutch_to_apply(req_lookup);
        Clutch releasing = get_clutch_to_release(req_lookup);
        PrefillData prefill_data = pressure_mgr->make_fill_data(applying);

        int MOD_MAX = this->pressure_mgr->get_max_solenoid_pressure();
        int SPC_MAX = pressure_manager->get_max_shift_pressure(egs_map_idx_lookup);

        TorqueRequstData trd = {
            .ty = TorqueRequestControlType::None,
            .bounds = TorqueRequestBounds::LessThan,
            .amount = 0
        };

        bool en_trq_req = true;
        if (GearChange::_1_2 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_1_2;
        } else if (GearChange::_2_3 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_2_3;
        } else if (GearChange::_3_4 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_3_4;
        } else if (GearChange::_4_5 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_4_5;
        } else if (GearChange::_2_1 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_2_1;
        } else if (GearChange::_3_2 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_3_2;
        } else if (GearChange::_4_3 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_4_3;
        } else if (GearChange::_5_4 == req_lookup) {
            en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_5_4;
        }

        ShiftInterfaceData sid = {
            .profile = profile,
            .MOD_MAX = MOD_MAX,
            .SPC_MAX = SPC_MAX,
            .shift_flags = 0,
            .change = req_lookup,
            .applying = applying,
            .releasing = releasing,
            .curr_g = this->actual_gear,
            .targ_g = this->target_gear,
            .inf = sd,
            .release_spring_on_clutch = pressure_manager->get_spring_pressure(applying),
            .release_spring_off_clutch = pressure_manager->get_spring_pressure(releasing),
            .prefill_info = prefill_data,
            .chars = chars,
            .ptr_r_clutch_speeds = &now_cs,
            .ptr_w_pressures = &p_now,
            .ptr_w_trq_req = &trd,
            .tcc = this->tcc,
            .adaptation_mgr = this->shift_adapter,
            .manual_shift = manually_requested,
            .trq_req_en = en_trq_req
        };
        // To set the flag values initially
        ShiftHelpers::calc_shift_flags(&sid, &this->sensor_data);

        float threshold_torque = VEHICLE_CONFIG.engine_drag_torque/10.0;
        ShiftingAlgorithm* algo;
        if (is_upshift) {
            if (sensor_data.converted_torque <= -threshold_torque/2) {
                algo = new ReleasingShift(&sid);
            }
            else {
                algo = new CrossoverShift(&sid);
            }
        }
        else {
            bool is_release = true;
            if (
                (sensor_data.converted_torque < threshold_torque && (sid.shift_flags & SHIFT_FLAG_COAST) != 0) ||
                ((sid.shift_flags & SHIFT_FLAG_COAST_54_43) != 0)
            ) {
                is_release = false;
            }
            if (is_release) {
                algo = new ReleasingShift(&sid);
            } else {
                algo = new CrossoverShift(&sid);
            }
        }

        uint8_t algo_phase_id = 0;
        while (process_shift) {
            uint32_t start_time = GET_CLOCK_TIME();
            RuntimeMonitor::Scope shift_time(RuntimeMonitor::Stage::Shift, 20000);
            bool stationary_shift = this->is_stationary();
            // Shifter moved mid shift!
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                process_shift = false;
                result = false;
                break;
            }

            int abs_input_torque = abs(sensor_data.input_torque);
            now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, req_lookup, this->gearboxConfig.bounds);

            // Shift reporting
            if (!stationary_shift) {
                if (now_cs.off_clutch_speed < -50 || now_cs.on_clutch_speed < -50) {
                    flaring = true;
                }
                else {
                    flaring = false;
                }

                this->set_torque_request(trd.ty, trd.bounds, trd.amount);
            }
            else {
                // If input speed is too low, use the overlap time as a way of measuring shift progress
                this->flaring = false;
                this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0); // And also torque requests
            }

            // Algorithm has control
            uint8_t step_result = algo->step(
                algo_phase_id,
                abs_input_torque,
                stationary_shift,
                is_upshift,
                phase_elapsed,
                total_elapsed,
                this->pressure_mgr,
                &this->sensor_data
            );
            this->algo_feedback = algo->get_diag_feedback(algo_phase_id);

            // Update pressures
            pressure_mgr->set_target_modulating_pressure(p_now.mod_sol_req);
            pressure_mgr->set_target_shift_pressure(p_now.shift_sol_req);
            pressure_mgr->update_pressures(
                sid.targ_g,
                sid.change
            );

            if (step_result == 0) {
                // Continue
                phase_elapsed += SHIFT_DELAY_MS;
            }
            else if (step_result == STEP_RES_END_SHIFT) {
                // Shift completed OK!
                result = true;
                break;
            }
            else if (step_result == STEP_RES_FAILURE) {
                // Shift failure!
                break;
            }
            else {
                // Phase has completed, update our data
                phase_elapsed = 0;
                if (step_result == STEP_RES_NEXT) {
                    algo_phase_id += 1;
                }
                else {
                    algo_phase_id = step_result;
                }
            }
            shift_time.finish();
            uint32_t elapsed = GET_CLOCK_TIME() - start_time;
            if (elapsed < SHIFT_DELAY_MS) {
                vTaskDelay((SHIFT_DELAY_MS - elapsed) / portTICK_PERIOD_MS);
            }
            total_elapsed += SHIFT_DELAY_MS;
        }
        if (result) { // Only set gear on conformation!
            this->actual_gear = gear_from_idx(sd.targ_g);
        }
        else {
            if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
                ESP_LOGE("SHIFT", "Shift failed due to selector moving");
                this->target_gear = GearboxGear::Neutral;
                this->actual_gear = GearboxGear::Neutral;
            }
            else {
                ESP_LOGE("SHIFT", "Shift failed! End ratio is %.2f", (float)sensor_data.gear_ratio);
                this->target_gear = this->actual_gear;
            }
        }
        pressure_manager->set_spc_p_max();
        pressure_manager->set_shift_circuit(sd.shift_circuit, false);
        pressure_manager->notify_shift_end();
        this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0);
        this->abort_shift = false;
        this->sensor_data.last_shift_time = GET_CLOCK_TIME();
        this->flaring = false;
        memset(&this->algo_feedback, 0x00, sizeof(ShiftAlgoFeedback));
        delete algo;
    }
    else if (GearChange::_IDLE == req_lookup) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! GearChange is IDLE");
    }
    else if (0xFF == egs_map_idx_lookup) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! GearChange is INVALID for FwdShift");
    }
    else if (nullptr == profile) {
        ESP_LOGE("ELAPSE_SHIFT", "BUG! Profile is null");
    }
    this->shift_idx = GearChange::_IDLE;
    return result;
}

void Gearbox::shift_thread()
{
    this->shifting = true;
    GearboxGear curr_target = this->target_gear;
    GearboxGear curr_actual = this->actual_gear;
    if (curr_actual == curr_target)
    {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "SHIFTER", "Gears are the same????");
        goto cleanup;
    }
    if (!is_controllable_gear(curr_actual) && !is_controllable_gear(curr_target))
    { // N->P or P->N
        this->pressure_mgr->set_target_shift_pressure(4000);
        //sol_y4->write_pwm_12_bit(800); // 3-4 is pulsed at 20%
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "No need to shift");
        this->actual_gear = curr_target; // Set on startup
        goto cleanup;
    }
    else if (is_controllable_gear(curr_actual) != is_controllable_gear(curr_target))
    { // This would be a garage shift, either in or out
        ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Garage shift");
        if (is_controllable_gear(curr_target))
        {
            bool into_reverse = GearboxGear::Reverse_First == curr_target || GearboxGear::Reverse_Second == curr_target;
            pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            float prefill;
            float spring;
            if (into_reverse) {
                prefill = pressure_manager->get_b3_prefill_pressure();
                spring = pressure_manager->get_spring_pressure(Clutch::B3);
            }
            else {
                prefill = pressure_manager->make_fill_data(Clutch::B2).fill_pressure_on_clutch;
                spring = pressure_manager->get_spring_pressure(Clutch::B2);
            }
            pressure_mgr->set_target_shift_pressure(((spring + (prefill)) / 1.993) + HYDR_PTR->shift_reg_spring_pressure); // TODO - 1.993 = spc multi [0]
            pressure_mgr->set_target_modulating_pressure(3000);
            this->pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
            // N/P -> R/D
            // Defaults (Start in 2nd)
            egs_can_hal->set_garage_shift_state(true, !into_reverse);
            uint16_t cycle_count = 0;
            bool completed_ok = false;

            uint16_t ramp = 0;
            uint16_t spc_step;
            uint16_t prefill_cycles;
            if (into_reverse) {
                prefill_cycles = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.prefill_time_b3, InterpType::Linear);
                spc_step = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.p_ramp_b3, InterpType::Linear);
            }
            else {
                prefill_cycles = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.prefill_time_b2, InterpType::Linear);
                spc_step = interpolate_float(sensor_data.atf_temp, &GAR_CURRENT_SETTINGS.p_ramp_b2, InterpType::Linear);
            }

            int p_apply_clutch = 0;
            while (true) {
                if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N) {
                    completed_ok = false;
                    break;
                }
                if (into_reverse) {
                    if (cycle_count > prefill_cycles) {
                        prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Reverse_Second, Clutch::B3, abs(sensor_data.input_torque), CoefficientTy::Sliding);
                        ramp += spc_step;
                    }
                }
                else {
                    if (cycle_count > prefill_cycles) {
                        prefill = pressure_manager->p_clutch_with_coef(GearboxGear::Second, Clutch::B2, abs(sensor_data.input_torque), CoefficientTy::Sliding);
                        ramp += spc_step;
                    }
                }
                p_apply_clutch = prefill + spring + ramp + HYDR_PTR->shift_reg_spring_pressure;
                pressure_mgr->set_target_modulating_pressure(3000);
                pressure_mgr->set_target_shift_pressure(p_apply_clutch);
                // Because EGS uses 1-2 when garage shifting...
                this->pressure_mgr->update_pressures(this->actual_gear, GearChange::_1_2);

                int turbine = this->speed_sensors.turbine;
                if (cycle_count > 50 && turbine <= 100 + calc_input_rpm_from_req_gear(sensor_data.output_rpm, curr_target, &this->gearboxConfig)) {
                    completed_ok = true;
                    break;
                }
                if (cycle_count > GAR_CURRENT_SETTINGS.timeout_cycles && sensor_data.engine_rpm - sensor_data.input_rpm < 200) {
                    completed_ok = false;
                    break;
                }
                vTaskDelay(20);
                cycle_count += 1;
            }
            if (!completed_ok) {
                ESP_LOGW("SHIFT", "Garage shift aborted");
                curr_target = this->shifter_pos == ShifterPosition::P ? GearboxGear::Park : GearboxGear::Neutral;
                curr_actual = this->shifter_pos == ShifterPosition::P ? GearboxGear::Park : GearboxGear::Neutral;
                pressure_mgr->set_target_shift_pressure(4000);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
            }
            else {
                ESP_LOGI("SHIFT", "Garage shift completed OK after %d ms", cycle_count * 20);
                int ramp_cycles = 5;
                while (ramp_cycles != 0) {
                    p_apply_clutch = linear_ramp_with_timer(p_apply_clutch, pressure_manager->get_max_solenoid_pressure(), ramp_cycles);
                    pressure_manager->set_target_modulating_pressure(5100);
                    this->pressure_mgr->update_pressures(this->actual_gear, GearChange::_1_2);
                    ramp_cycles--;
                    vTaskDelay(20);
                }
                pressure_mgr->set_spc_p_max();
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            }
            egs_can_hal->set_garage_shift_state(false, !into_reverse);
        }
        else
        {
            // Garage shifting to N or P, we can just set the pressure back to idle
            pressure_mgr->set_target_shift_pressure(4000);
        }
        if (is_fwd_gear(curr_target))
        {
            // Last forward known gear
            // This way we better handle shifting from N->D at speed!
            this->actual_gear = GearboxGear::Second;
        }
        else
        {
            this->actual_gear = curr_target; // R1/R2
        }
        goto cleanup;
    }
    else
    { // Both gears are controllable
        if (is_fwd_gear(curr_target) != is_fwd_gear(curr_actual))
        {
            // In this case, we set the current gear to neutral, then thread will re-spawn
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shifter got stuck in R-D. Returning and trying again");
            this->actual_gear = GearboxGear::Neutral;
            goto cleanup;
        }
        else if (is_fwd_gear(curr_target) && is_fwd_gear(curr_actual))
        {
            // Forward shift logic
            if (curr_target > curr_actual)
            { // Upshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Upshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                // this->show_upshift = true;
                GearChange pgc;
                if (curr_target == GearboxGear::Second)
                { // 1-2
                    pgc = GearChange::_1_2;
                }
                else if (curr_target == GearboxGear::Third)
                { // 2-3
                    pgc = GearChange::_2_3;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 3-4
                    pgc = GearChange::_3_4;
                }
                else if (curr_target == GearboxGear::Fifth)
                { // 4-5
                    pgc = GearChange::_4_5;
                }
                else
                { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                this->shift_idx = pgc;
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile* prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                this->is_upshift = true;
                this->fwd_gear_shift = true;
                elapse_shift(pgc, prof, this->shift_req_was_manual);
                this->start_second = true;
                goto cleanup;
            }
            else
            { // Downshifting
                ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Downshift request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
                // this->show_downshift = true;
                GearChange pgc;
                if (curr_target == GearboxGear::First)
                { // 2-1
                    pgc = GearChange::_2_1;
                    this->start_second = false;
                }
                else if (curr_target == GearboxGear::Second)
                { // 3-2
                    pgc = GearChange::_3_2;
                    this->start_second = true;
                }
                else if (curr_target == GearboxGear::Third)
                { // 4-3
                    pgc = GearChange::_4_3;
                }
                else if (curr_target == GearboxGear::Fourth)
                { // 5-4
                    pgc = GearChange::_5_4;
                }
                else
                { // WTF
                    this->target_gear = this->actual_gear;
                    goto cleanup;
                }
                this->shift_idx = pgc;
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile* prof = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                this->is_upshift = false;
                this->fwd_gear_shift = true;
                elapse_shift(pgc, prof, this->shift_req_was_manual);
                goto cleanup;
            }
        }
        else
        {
            ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Ignoring request to change between %s and %s!", gear_to_text(curr_actual), gear_to_text(curr_target));
        }
        goto cleanup;
    }
cleanup:
    ESP_LOG_LEVEL(ESP_LOG_INFO, "SHIFTER", "Shift complete");
    this->set_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 0);
    // The shift thread owns this lifecycle, including selector abort and
    // algorithm failure before the phase-specific normal shift_end calls.
    if (this->tcc != nullptr) this->tcc->shift_end();
    this->shifting = false;
    this->fwd_gear_shift = false;
    this->is_upshift = false;
    vTaskDelete(nullptr);
}

void Gearbox::controller_loop()
{
    ShifterPosition last_position = ShifterPosition::SignalNotAvailable;
    ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "GEARBOX START!");
    uint32_t expire_check = GET_CLOCK_TIME() + 100; // 100ms
    egs_can_hal->set_safe_start(true);
    while (GET_CLOCK_TIME() < expire_check)
    {
        // Step 1. Aquire ALL Sensors
        {
            RuntimeMonitor::Scope io_time(RuntimeMonitor::Stage::Io);
            TCUIO::update_io_layer();
            RuntimeMonitor::source_complete(RuntimeMonitor::Source::IoCompletion, RuntimeMonitor::now_us());
        }

        this->shifter_pos = egs_can_hal->get_shifter_position(250);
        last_position = this->shifter_pos;
        if (this->shifter_pos == ShifterPosition::P || this->shifter_pos == ShifterPosition::N)
        {
            egs_can_hal->set_safe_start(true);
            break; // Default startup, OK
        }
        else if (this->shifter_pos == ShifterPosition::D)
        { // Car is in motion forwards!
            this->actual_gear = GearboxGear::Fifth;
            this->target_gear = GearboxGear::Fifth;
            this->gear_disagree_count = 20; // Set disagree counter to non 0. This way gearbox must calculate ratio
            egs_can_hal->set_safe_start(false);
            break;
        }
        else if (this->shifter_pos == ShifterPosition::R)
        { // Car is in motion backwards!
            this->actual_gear = GearboxGear::Reverse_Second;
            this->target_gear = GearboxGear::Reverse_Second;
            egs_can_hal->set_safe_start(false);
            break;
        }
        else {
            egs_can_hal->set_safe_start(true); // Unknown position, keep polling until we don't know
        }
        vTaskDelay(5);
    }
    bool previous_speeds_valid = true;
    bool previous_engine_valid = true;
    bool previous_pedal_valid = true;
    while (1)
    {
        uint32_t start = GET_CLOCK_TIME();
        RuntimeMonitor::Scope monitor_loop(RuntimeMonitor::Stage::Gearbox, 20000);
        TccTraceRecord cycle_trace = {};
        cycle_trace.timestamp_us = uint64_t(esp_timer_get_time());
        cycle_trace.flags = TccTrace::UPDATE_SKIPPED | TccTrace::CAN_FRESHNESS_UNKNOWN;
        cycle_trace.actual_gear = uint8_t(this->actual_gear);
        cycle_trace.target_gear = uint8_t(this->target_gear);
        portENTER_CRITICAL(&this->profile_mutex);
        AbstractProfile* trace_profile_at_start = this->current_profile;
        portEXIT_CRITICAL(&this->profile_mutex);
        cycle_trace.profile = trace_profile_at_start ? uint8_t(trace_profile_at_start->get_profile()) : 0xFF;
        {
            RuntimeMonitor::Scope io_time(RuntimeMonitor::Stage::Io);
            TCUIO::update_io_layer();
            RuntimeMonitor::source_complete(RuntimeMonitor::Source::IoCompletion, RuntimeMonitor::now_us());
        }
        if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {
            if (this->tcc) {
                this->tcc->suspend(this->pressure_mgr);
                GlidingStatus::publish(this->tcc->gliding_status(GET_CLOCK_TIME()));
            }
            previous_speeds_valid = previous_engine_valid = previous_pedal_valid = false;
            SOLENOID_CONTROL_EGS_SLAVE slave_rq = egs_can_hal->get_tester_req();
            sol_mpc->set_current_target(__builtin_bswap16(slave_rq.MPC_REQ));
            sol_spc->set_current_target(__builtin_bswap16(slave_rq.SPC_REQ));
            sol_tcc->set_duty(slave_rq.TCC_REQ * 16); // x16 to go from 8 bit (0-255) to 12bit (0-4096)
            if (slave_rq.Y3_EN) {
                sol_y3->on();
            }
            else {
                sol_y3->off();
            }
            if (slave_rq.Y4_EN) {
                sol_y4->on();
            }
            else {
                sol_y4->off();
            }
            if (slave_rq.Y5_EN) {
                sol_y5->on();
            }
            else {
                sol_y5->off();
            }
            SENSOR_REPORT_EGS_SLAVE sensor_rpt;

            this->process_speed_sensors();


            uint8_t pll = TCUIO::parking_lock();
            int16_t tft = TCUIO::atf_temperature();
            uint16_t vbatt = TCUIO::battery_mv();

            sensor_rpt.N2_RAW = __builtin_bswap16(this->speed_sensors.n2);
            sensor_rpt.N3_RAW = __builtin_bswap16(this->speed_sensors.n3);
            sensor_rpt.TFT = pll ? 0xFF : tft + 50;
            sensor_rpt.VBATT = (vbatt / 100) & 0xFF;

            SOLENOID_REPORT_EGS_SLAVE sol_rpt;
            sol_rpt.MPC_CURR = __builtin_bswap16(sol_mpc->get_current());
            sol_rpt.SPC_CURR = __builtin_bswap16(sol_spc->get_current());
            sol_rpt.TCC_PWM = (sol_tcc->get_pwm_raw() / 16) & 0xFF;

            UN52_REPORT_EGS_SLAVE un52_rpt;
            un52_rpt.Y3_CURR = __builtin_bswap16(sol_y3->get_current());
            un52_rpt.Y4_CURR = __builtin_bswap16(sol_y4->get_current());
            un52_rpt.Y5_CURR = __builtin_bswap16(sol_y5->get_current());
            un52_rpt.TCC_CURR = __builtin_bswap16(sol_tcc->get_current());

            egs_can_hal->set_slave_mode_reports(sol_rpt, sensor_rpt, un52_rpt);
            cycle_trace.path = 0; // Slave mode bypasses normal TCC/PM control.
            TccTrace::append(cycle_trace);
            RuntimeMonitor::quality(0);
            monitor_loop.finish();
            vTaskDelay(20);
            continue;
        }
        if (this->diag_stop_control)
        {
            if (this->tcc) {
                this->tcc->suspend(this->pressure_mgr);
                GlidingStatus::publish(this->tcc->gliding_status(GET_CLOCK_TIME()));
            }
            previous_speeds_valid = previous_engine_valid = previous_pedal_valid = false;
            cycle_trace.path = 0; // Diagnostic stop skips the controller update.
            TccTrace::append(cycle_trace);
            RuntimeMonitor::quality(0);
            monitor_loop.finish();
            vTaskDelay(50);
            continue;
        }

        // Use this cycle's valid ATF before any pressure/TCC decisions. On a
        // source fault retain only the last numerical value for an active shift.
        const bool atf_valid = TCUIO::atf_temperature_valid();
        if (atf_valid) this->sensor_data.atf_temp = TCUIO::atf_temperature();

        // Set sensors Motor temperature (Always ran)
        int16_t coolant_temp = egs_can_hal->get_engine_coolant_temp(50);

        bool coast_raw_speeds_valid = false;
        bool speeds_valid = this->process_speed_sensors(&coast_raw_speeds_valid);
        if (speeds_valid)
        {
            if (!previous_speeds_valid) {
                this->cached_input_rpm = speed_sensors.turbine * 100;
                this->cached_output_rpm = speed_sensors.output * 100;
            }
            this->cached_input_rpm = first_order_filter(3, speed_sensors.turbine * 100, this->cached_input_rpm);
            this->sensor_data.input_rpm = this->cached_input_rpm / 100;
            this->cached_output_rpm = first_order_filter(3, speed_sensors.output * 100, this->cached_output_rpm);
            this->sensor_data.output_rpm = this->cached_output_rpm / 100;
            bool stationary = this->is_stationary();
            if (!stationary)
            {
                // Store our ratio
                this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
                this->sensor_data.targ_gear_ratio = ratio_absolute(this->actual_gear, &this->gearboxConfig);

            }
            else {
                // Stationary so no ratios
                this->sensor_data.gear_ratio = 0.0;
                this->sensor_data.targ_gear_ratio = 0.0;
            }
            if (!shifting && !stationary && sensor_data.output_rpm > 250)
            {
                if (is_fwd_gear(this->actual_gear))
                {
                    if (calcGearFromRatio(false) && this->est_gear_idx != 0)
                    {
                        // Compare gears
                        GearboxGear estimate = gear_from_idx(this->est_gear_idx);
                        if (estimate != this->actual_gear)
                        {
                            gear_disagree_count++;
                            if (gear_disagree_count >= 50)
                            {
                                this->actual_gear = estimate; // DID NOT SHIFT!
                                this->target_gear = estimate;
                                this->last_fwd_gear = estimate;
                            }
                        }
                        else
                        {
                            gear_disagree_count = 0;
                        }
                    }
                }
                else
                {
                    gear_disagree_count = 0;
                }
            }
            else
            {
                gear_disagree_count = 0;
            }
        }
        else
        {
            speeds_valid = false;
            gear_disagree_count = 0;
        }
        previous_speeds_valid = speeds_valid;
        if (speeds_valid && !this->is_stationary())
        {
            bool rev = !is_fwd_gear(this->target_gear);
            if (!this->calcGearFromRatio(rev))
            {
                // ESP_LOG_LEVEL(ESP_LOG_ERROR, "GEARBOX", "GEAR RATIO IMPLAUSIBLE");
            }
        }
        const GlidingCanInputs gliding_can = egs_can_hal->get_gliding_inputs(Gliding::CAN_EXPIRY_MS);
        const uint8_t p_tmp = gliding_can.driver_valid ? gliding_can.pedal : egs_can_hal->get_pedal_value(100);
        const bool pedal_valid = p_tmp != UINT8_MAX;
        this->pedal_last = this->sensor_data.pedal_pos;
        if (pedal_valid)
        {
            this->sensor_data.pedal_pos = p_tmp;
            if (!previous_pedal_valid) this->sensor_data.pedal_pos_smoothed = p_tmp;
            this->sensor_data.pedal_pos_smoothed = linear_interp_with_percentage(80, p_tmp, this->sensor_data.pedal_pos_smoothed);
        }
        // Missing pedal does not synthesize 25% demand. Both caches are held;
        // the separate current validity gates all new forward control below.
        previous_pedal_valid = pedal_valid;

        if (GET_CLOCK_TIME() - start > 100) {
            // Update every 100ms, not every EGS cycle, values multiplied by 10
            // to get them in terms of 1 second (1s/100ms = 10)
            if (this->pedal_delta) {
                this->pedal_delta->update(this->sensor_data.pedal_pos * 10);
            }
            if (this->input_rpm_delta) {
                this->input_rpm_delta->update(this->sensor_data.input_rpm * 10);
            }
            this->last_delta_time = start;
        }

        sensor_data.brake_pressed = brake_pedal.is_brake_pedal_pressed(egs_can_hal, 250);
        sensor_data.kickdown_pressed = kickdown.is_kickdown_newly_pressed(egs_can_hal, 250);
        const uint16_t tmp_rpm = egs_can_hal->get_engine_rpm(100);
        const bool engine_valid = tmp_rpm != UINT16_MAX;
        if (engine_valid) {
            if (!previous_engine_valid) this->cached_engine_rpm = tmp_rpm * 100;
            this->cached_engine_rpm = first_order_filter(3, tmp_rpm * 100, this->cached_engine_rpm);
            this->sensor_data.engine_rpm = this->cached_engine_rpm / 100;
        }
        previous_engine_valid = engine_valid;
        const RuntimeSignalPolicy signals{engine_valid, pedal_valid, speeds_valid, atf_valid, tmp_rpm};
        RuntimeMonitor::quality((atf_valid ? RuntimeMonitor::AtfValid : 0) |
            (speeds_valid ? RuntimeMonitor::SpeedsValid : 0) |
            (engine_valid ? RuntimeMonitor::EngineValid : 0) |
            (pedal_valid ? RuntimeMonitor::PedalValid : 0) |
            (this->shifting ? RuntimeMonitor::Shifting : 0));
        const bool forward_control_valid = signals.forward_control_allowed();
        // The coast filter observes the same current CAN samples.
        const uint8_t coast_pedal = p_tmp;
        const uint16_t coast_engine_rpm = tmp_rpm;
        const bool coast_engine_valid = coast_engine_rpm != UINT16_MAX && coast_engine_rpm > 100 &&
            this->redline_rpm > 500 && coast_engine_rpm < this->redline_rpm - 500;
        portENTER_CRITICAL(&this->profile_mutex);
        AbstractProfile* coast_profile = this->current_profile;
        portEXIT_CRITICAL(&this->profile_mutex);
        if (comfort != nullptr) {
            comfort->update_coast_context(this->actual_gear, this->target_gear, &this->sensor_data,
                coast_profile == comfort && !this->shifting && gear_disagree_count == 0 &&
                    this->sensor_data.engine_rpm > 100,
                speeds_valid && coast_raw_speeds_valid && coast_engine_valid && coast_pedal != UINT8_MAX,
                coast_pedal, GET_CLOCK_TIME());
        }

        // Update solenoids, only if engine RPM is OK
        if (engine_valid && atf_valid && tmp_rpm > 400)
        {
            if (!shifting)
            {
                this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, true);
                this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
            }
        }
        uint8_t pll = TCUIO::parking_lock();
        if (UINT8_MAX != pll)
        {
            bool lock_state = pll != 0;
            if (lock_state && engine_valid && atf_valid) {
                this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, true);
                this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
                this->pressure_mgr->set_target_shift_pressure(4000);
            }
            egs_can_hal->set_safe_start(lock_state);
            this->shifter_pos = egs_can_hal->get_shifter_position(1000);
            if (
                this->shifter_pos == ShifterPosition::P ||
                this->shifter_pos == ShifterPosition::P_R ||
                this->shifter_pos == ShifterPosition::R ||
                this->shifter_pos == ShifterPosition::R_N ||
                this->shifter_pos == ShifterPosition::N ||
                this->shifter_pos == ShifterPosition::N_D ||
                this->shifter_pos == ShifterPosition::D ||
                this->shifter_pos == ShifterPosition::FOUR ||
                this->shifter_pos == ShifterPosition::THREE ||
                this->shifter_pos == ShifterPosition::TWO ||
                this->shifter_pos == ShifterPosition::ONE)
            {
                if (this->shifter_pos != last_position)
                {
                    if (lock_state)
                    {
                        if (this->shifter_pos == ShifterPosition::P)
                        {
                            this->target_gear = GearboxGear::Park;
                            last_position = this->shifter_pos;
                            if (this->shift_adapter != nullptr)
                            {
                                if (this->shift_adapter->save() != ESP_OK)
                                    RuntimeMonitor::fault(RuntimeMonitor::Counter::AdaptationError);
                            }
                            if (this->tcc && this->tcc->save() != ESP_OK)
                                RuntimeMonitor::fault(RuntimeMonitor::Counter::AdaptationError);
                            // Save profile
                            if (ShifterStyle::EWM == shifter->get_shifter_type()) {
                                if (ETS_CURRENT_SETTINGS.ewm_save_profile) {
                                    // We know that the profile is valid based on
                                    // use selection (EWM button code) - So we don't need to check this
                                    uint8_t tag = this->current_profile->get_profile_id();
                                    // By default, we can save, but just check if manual profile without
                                    // the user wanting to save manual profiles
                                    bool can_save = true;
                                    if (tag == PROFILE_IDX_M || tag == PROFILE_IDX_R || tag == PROFILE_IDX_W) {
                                        can_save = ETS_CURRENT_SETTINGS.ewm_save_profile_manual;
                                    }
                                    if (can_save) {
                                        esp_err_t res = EEPROM::ewm_btn_save_profile(tag);
                                        if (ESP_OK != res) {
                                            ESP_LOGW("EWM SAVE", "Profile could not be saved to NVS");
                                        }
                                    }
                                }
                            }
                        }
                        else if (this->shifter_pos == ShifterPosition::N)
                        {
                            this->target_gear = GearboxGear::Neutral;
                            last_position = this->shifter_pos;
                        }
                    }
                    else
                    {
                        // Drive or R!
                        if (this->shifter_pos == ShifterPosition::R)
                        {
                            this->target_gear = this->start_second ? GearboxGear::Reverse_Second : GearboxGear::Reverse_First;
                            last_position = this->shifter_pos;
                        }
                        else if (this->shifter_pos == ShifterPosition::D || this->shifter_pos == ShifterPosition::N_D)
                        {
                            // Some shift levers can be semi broken
                            // and can switch between ND and D randomly whilst in motion
                            // handle that case here by checking current fwd gear first
                            // If current gear is also fwd, ignore!
                            if (!is_fwd_gear(this->actual_gear) && !is_fwd_gear(this->target_gear))
                            {
                                this->target_gear = this->start_second ? GearboxGear::Second : GearboxGear::First;
                            }
                            last_position = this->shifter_pos;
                        }
                    }
                }
            }
        }
        if (!forward_control_valid) {
            this->ask_downshift = false;
            this->ask_upshift = false;
            this->manual_shift = false;
        }
        if (engine_valid && tmp_rpm > 100)
        {
            if (forward_control_valid && is_fwd_gear(this->actual_gear))
            {
                // Check our range restict (Only for TRRS)
                switch (egs_can_hal->get_shifter_position(250)) { // Don't use shifter_pos, as that only registers D. Query raw selector pos
                case ShifterPosition::FOUR:
                    this->restrict_target = GearboxGear::Fourth;
                    break;
                case ShifterPosition::THREE:
                    this->restrict_target = GearboxGear::Third;
                    break;
                case ShifterPosition::TWO:
                    this->restrict_target = GearboxGear::Second;
                    break;
                case ShifterPosition::ONE:
                    this->restrict_target = GearboxGear::First;
                    break;
                default:
                    this->restrict_target = GearboxGear::Fifth;
                    break;
                }
                // Seek up the restriction target if the RPM is too high for the current gear!
                // Seek up to Fifth
                while (this->restrict_target != GearboxGear::Fifth && calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, this->restrict_target, &this->gearboxConfig) > this->redline_rpm)
                {
                    this->restrict_target = next_gear(this->restrict_target);
                }

                // In gear, not shifting, and no ratio mismatch
                if (!shifting && this->actual_gear == this->target_gear && gear_disagree_count == 0)
                {
                    // Enter critical ISR section
                    portENTER_CRITICAL(&this->profile_mutex);
                    AbstractProfile* p = this->current_profile;
                    // Exit critical
                    portEXIT_CRITICAL(&this->profile_mutex);
                    // Check if profile is loaded
                    if (p != nullptr)
                    {
                        p->update(&this->sensor_data);
                        // Ask the current drive profile if it thinks, given the current
                        // data, if the car should up/downshift
                        if (this->restrict_target > this->actual_gear && p->should_upshift(this->actual_gear, &this->sensor_data))
                        {
                            this->ask_upshift = true; // Upshift takes priority
                            this->manual_shift = false;
                        }
                        else if (this->restrict_target < this->actual_gear || p->should_downshift(this->actual_gear, &this->sensor_data)) {
                            this->ask_downshift = true; // Downshift is secondary
                            this->manual_shift = false;
                        }
                    }
                    if (this->ask_upshift && this->actual_gear < GearboxGear::Fifth)
                    {
                        // Check RPMs
                        GearboxGear next = next_gear(this->actual_gear);
                        // Second gear shift defaults to OK as we can safely start in second (For C/W mode)
                        if (next == GearboxGear::Second || calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, &this->gearboxConfig) > 900)
                        {
                            this->target_gear = next;
                        }
                    }
                    else if ((this->ask_downshift || sensor_data.kickdown_pressed) && this->actual_gear > GearboxGear::First)
                    {
                        // Check RPMs
                        GearboxGear prev = prev_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev, &this->gearboxConfig) < this->redline_rpm - 500)
                        {
                            this->target_gear = prev;
                        }
                    }
                }
                // Request processed. Cancel the requests. Put this outside here so that if there is a ratio mismatch, paddles are ignored
                this->ask_downshift = false;
                this->ask_upshift = false;
                this->shift_req_was_manual = this->manual_shift;
                this->manual_shift = false;

            }
        }
        else if (signals.engine_stopped() && !shifting)
        {
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
        }

        // A skipped update is a real suspension, not just a zero PM target.
        // This covers P/N/R (including a non-forward target), signal faults and
        // observed engine stop before the later PM output can reapply old state.
        // Exactly one torque decoder call per loop: its EGS52 implementation
        // maintains demand-correction state. Share the same fresh sample.
        const CanTorqueData loop_torque = egs_can_hal->get_torque_data(100);
        if (forward_control_valid && is_fwd_gear(this->actual_gear) && is_fwd_gear(this->target_gear)) {
            if (this->tcc != nullptr)
            {
                // Profiles live for the lifetime of the TCU; only snapshot the pointer under lock.
                portENTER_CRITICAL(&this->profile_mutex);
                AbstractProfile* tcc_profile = this->current_profile;
                portEXIT_CRITICAL(&this->profile_mutex);
                cycle_trace.path = 1;
                cycle_trace.flags = (cycle_trace.flags & ~TccTrace::UPDATE_SKIPPED) | TccTrace::INPUT_VALID;
                cycle_trace.profile = tcc_profile ? uint8_t(tcc_profile->get_profile()) : 0xFF;
                cycle_trace.actual_gear = uint8_t(this->actual_gear);
                cycle_trace.target_gear = uint8_t(this->target_gear);
                cycle_trace.engine_rpm_at_tcc = this->sensor_data.engine_rpm;
                cycle_trace.input_rpm_at_tcc = this->sensor_data.input_rpm;
                cycle_trace.output_rpm_at_tcc = this->sensor_data.output_rpm;
                cycle_trace.converted_torque_at_tcc = this->sensor_data.converted_torque;
                cycle_trace.atf_c_at_tcc = this->sensor_data.atf_temp;
                cycle_trace.pedal_at_tcc = this->sensor_data.pedal_pos;
                cycle_trace.pedal_smoothed_at_tcc = this->sensor_data.pedal_pos_smoothed;
                cycle_trace.coast_raw_pedal = coast_pedal;
                cycle_trace.coast_engine_rpm = coast_engine_rpm;
                cycle_trace.quality_flags = (speeds_valid ? 1 : 0) | (coast_raw_speeds_valid ? 2 : 0) |
                    (coast_engine_valid ? 4 : 0) | (coast_pedal != UINT8_MAX ? 8 : 0);
                int32_t signed_slip = int32_t(cycle_trace.engine_rpm_at_tcc) - int32_t(cycle_trace.input_rpm_at_tcc);
                if (signed_slip > INT16_MAX) {
                    cycle_trace.signed_engine_minus_input_rpm = INT16_MAX;
                    cycle_trace.flags |= TccTrace::SIGNED_SLIP_SATURATED;
                } else if (signed_slip < INT16_MIN) {
                    cycle_trace.signed_engine_minus_input_rpm = INT16_MIN;
                    cycle_trace.flags |= TccTrace::SIGNED_SLIP_SATURATED;
                } else cycle_trace.signed_engine_minus_input_rpm = int16_t(signed_slip);
                this->tcc->trace_before(cycle_trace);
                {
                    RuntimeMonitor::Scope tcc_time(RuntimeMonitor::Stage::Tcc);
                    const bool fresh_brake_released = coast_pedal == 0 &&
                        tcc_profile != nullptr && tcc_profile->get_profile() == GearboxProfile::Standard &&
                        this->actual_gear == GearboxGear::Third && this->target_gear == GearboxGear::Third &&
                        !this->shifting && egs_can_hal->get_brake_released_for_coast(
                            SCityTccCoast::BRAKE_RELEASE_EXPIRY_MS);
                    const SCityTccCoast::Context third_coast_context{
                        .standard_profile = tcc_profile != nullptr &&
                            tcc_profile->get_profile() == GearboxProfile::Standard,
                        .stable_third = this->actual_gear == GearboxGear::Third &&
                            this->target_gear == GearboxGear::Third,
                        .gearbox_shifting = this->shifting,
                        .ratio_agrees = gear_disagree_count == 0 && SCityTccCoast::ratio_is_third(
                            this->sensor_data.input_rpm, this->sensor_data.output_rpm,
                            this->gearboxConfig.bounds[2].ratio_min_drift,
                            this->gearboxConfig.bounds[2].ratio_max_drift),
                        .forward_signals_valid = forward_control_valid,
                        .raw_speeds_valid = coast_raw_speeds_valid,
                        .safe_engine_rpm = coast_engine_valid,
                        .brake_released_valid = fresh_brake_released,
                        .brake_pressed = this->sensor_data.brake_pressed,
                        .raw_pedal = coast_pedal,
                        .cached_pedal = this->sensor_data.pedal_pos,
                    };
                    const CanTorqueData& glide_torque = loop_torque;
                    const bool automatic_glide_profile = tcc_profile != nullptr &&
                        (tcc_profile->get_profile() == GearboxProfile::Standard ||
                         tcc_profile->get_profile() == GearboxProfile::Comfort ||
                         tcc_profile->get_profile() == GearboxProfile::Agility ||
                         tcc_profile->get_profile() == GearboxProfile::Winter);
                    Gliding::Context glide_context;
                    glide_context.can = gliding_can;
                    // Keep the physical brake input as an additional veto.
                    glide_context.can.brake_released &= !sensor_data.brake_pressed;
                    glide_context.feedback_valid = forward_control_valid && coast_raw_speeds_valid &&
                        coast_engine_valid && glide_torque.m_converted_static != INT16_MAX;
                    glide_context.atf_valid = atf_valid;
                    glide_context.automatic_profile = automatic_glide_profile;
                    glide_context.shifting = this->shifting;
                    glide_context.actual_gear = uint8_t(this->actual_gear);
                    glide_context.target_gear = uint8_t(this->target_gear);
                    const float fifth_ratio = sensor_data.output_rpm > 250 ?
                        float(sensor_data.input_rpm)/sensor_data.output_rpm : 0;
                    glide_context.ratio_valid = gear_disagree_count == 0 &&
                        fifth_ratio >= gearboxConfig.bounds[4].ratio_min_drift &&
                        fifth_ratio <= gearboxConfig.bounds[4].ratio_max_drift;
                    glide_context.speed_x100 = Gliding::speed_x100(gliding_can, VEHICLE_CONFIG.wheel_circumference);
                    glide_context.engine_rpm = coast_engine_rpm;
                    glide_context.input_rpm = sensor_data.input_rpm;
                    glide_context.torque_nm = glide_torque.m_converted_static;
                    glide_context.atf_c = sensor_data.atf_temp;
                    this->tcc->update(this->actual_gear, this->target_gear, this->pressure_mgr,
                        tcc_profile, &this->sensor_data, SCityTccCoast::allow(third_coast_context), &glide_context);
                }
                this->tcc->trace_after(cycle_trace);
                egs_can_hal->set_clutch_status(this->tcc->get_clutch_state());
            }
        } else {
            this->tcc_percent = 0;
            if (this->tcc) this->tcc->suspend(this->pressure_mgr);
            else this->pressure_mgr->set_target_tcc_pressure(0);
            cycle_trace.path = 3;
            cycle_trace.actual_gear = uint8_t(this->actual_gear);
            cycle_trace.target_gear = uint8_t(this->target_gear);
            egs_can_hal->set_clutch_status(TccClutchStatus::Open);
        }

        // Keep shift-thread creation after the TCC decision, as in the stable base.
        if (this->target_gear != this->actual_gear && !this->shifting &&
            signals.new_shift_allowed(is_fwd_gear(this->actual_gear), is_fwd_gear(this->target_gear),
                this->target_gear == GearboxGear::Park || this->target_gear == GearboxGear::Neutral))
        {
            xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
        }

        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(this->shifter_pos);
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        egs_can_hal->set_tcc_trq_multiplier(this->sensor_data.tcc_trq_multiplier);
        if (this->aborting)
        {
            egs_can_hal->set_abort_shift(true);
        }
        else
        {
            egs_can_hal->set_target_gear(this->target_gear);
        }
        egs_can_hal->set_actual_gear(this->actual_gear);
        egs_can_hal->set_wheel_torque(0); // Nm

        const CanTorqueData& trqs = loop_torque;
        // CALC TORQUES
        if (INT16_MAX != trqs.m_min) { sensor_data.min_torque = trqs.m_min; }
        if (INT16_MAX != trqs.m_max) { sensor_data.max_torque = trqs.m_max; }
        if (INT16_MAX != trqs.m_ind) { sensor_data.indicated_torque = trqs.m_ind; }
        if (INT16_MAX != trqs.m_converted_static) { sensor_data.converted_torque = trqs.m_converted_static; }
        if (INT16_MAX != trqs.m_converted_driver) {
            int input_trq = InputTorqueModel::get_input_torque(
                sensor_data.engine_rpm,
                sensor_data.input_rpm,
                trqs.m_converted_driver
            );
            sensor_data.input_torque = input_trq;
            sensor_data.converted_driver_torque = trqs.m_converted_driver;
        }
        sensor_data.pump_torque = InputTorqueModel::get_pump_torque(sensor_data.engine_rpm, sensor_data.input_rpm);

        if (this->shifting && is_controllable_gear(this->target_gear) && !is_controllable_gear(this->actual_gear)) {
            if (INT16_MAX != sensor_data.pump_torque) {
                sensor_data.input_torque = sensor_data.pump_torque * sensor_data.tcc_trq_multiplier;
            }
        }

        // Wheel torque
        /*
        if (this->sensor_data.gear_ratio == 0)
        {
            // Fallback ratio for when gear ratio is actually 0
            float f;
            switch (this->target_gear)
            {
            case GearboxGear::First:
                f = gearboxConfig.ratios[0];
                break;
            case GearboxGear::Second:
                f = gearboxConfig.ratios[1];
                break;
            case GearboxGear::Third:
                f = gearboxConfig.ratios[2];
                break;
            case GearboxGear::Fourth:
                f = gearboxConfig.ratios[3];
                break;
            case GearboxGear::Reverse_First:
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Reverse_Second:
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Park:
            case GearboxGear::SignalNotAvailable:
            case GearboxGear::Neutral:
            default:
                f = 0.0;
                break;
            }
            egs_can_hal->set_wheel_torque_multi_factor(f);
        }
        else
        {
            egs_can_hal->set_wheel_torque_multi_factor(this->sensor_data.gear_ratio);
        }
        */

        // ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Torque: MIN: %3d, MAX: %3d, STAT: %3d", min_torque, max_torque, static_torque);
        //  Show debug symbols on IC
        float ratio_from_c_gear = ratio_absolute(this->actual_gear, &this->gearboxConfig);
        float ratio_from_t_gear = ratio_absolute(this->target_gear, &this->gearboxConfig);
        float tcc_multipler = InputTorqueModel::get_input_torque_factor(sensor_data.engine_rpm, sensor_data.input_rpm);
        this->sensor_data.tcc_trq_multiplier = tcc_multipler;
        float torque_ratio = 0; // Implausible
        if (
            ratio_from_c_gear != 0 && // Valid ratio
            sensor_data.engine_rpm != 0 // Engine is turning
            ) {
            torque_ratio = ratio_from_c_gear;
            if (ratio_from_t_gear > ratio_from_c_gear) {
                torque_ratio = ratio_from_t_gear;
            }
            torque_ratio *= tcc_multipler;
            torque_ratio *= diff_ratio_f;
            if (torque_ratio < 1) {
                torque_ratio = 1; // HOW!? (diff ratio is always > 2.0)
            }
        }
        else if (sensor_data.engine_rpm == 0) {
            torque_ratio = -1; // Cannot calculate
        }
        egs_can_hal->set_wheel_torque_multi_factor(torque_ratio);
        if (this->show_upshift && this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::RequestGearAgain);
        }
        else if (this->show_upshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Upshift);
        }
        else if (this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Downshift);
        }
        else
        {
            egs_can_hal->set_display_msg(GearboxMessage::None);
        }

        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr)
        {
            egs_can_hal->set_drive_profile(this->current_profile->get_profile());
            if (this->flaring && SBS.f_shown_if_flare)
            {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Failure, false);
            }
            else if (this->tcc != nullptr && this->tcc->gliding_display_active())
            {
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Gliding, false);
            }
            else
            {
                if (this->current_profile == race && this->fwd_gear_shift && SBS.debug_show_up_down_arrows_in_r) {
                    egs_can_hal->set_display_msg(this->is_upshift ? GearboxMessage::Upshift : GearboxMessage::Downshift);
                }
                else if ((this->current_profile == manual || this->current_profile == race) &&
                    sensor_data.engine_rpm > this->redline_rpm - 1000
                    ) {
                    egs_can_hal->set_display_msg(GearboxMessage::Upshift);
                }
                else {
                    egs_can_hal->set_display_msg(GearboxMessage::None);
                }
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear), this->current_profile == manual);
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
        // Apply the confirmed engine-stop electrical state after PM so its
        // retained targets cannot immediately undo it. Lost CAN is not STOP.
        if (signals.engine_stopped() && !this->shifting) {
            sol_mpc->set_current_target(0);
            sol_spc->set_current_target(0);
            sol_tcc->set_duty(0);
        }
        // This is the later PM output point; input fields above remain the exact TCC snapshot.
        cycle_trace.pm_requested_mbar = pressure_mgr->get_targ_tcc_pressure();
        cycle_trace.pm_final_pwm_raw = sol_tcc->get_pwm_raw();
        cycle_trace.pm_path = cycle_trace.path == 3 ? 2 : 1;
        cycle_trace.flags |= TccTrace::PM_VALID;
        if (!(cycle_trace.flags & TccTrace::INPUT_VALID) && this->tcc) {
            this->tcc->trace_before(cycle_trace);
            this->tcc->trace_after(cycle_trace);
            cycle_trace.selected_can_request_bits = this->tcc->get_can_req_bits();
        }
        if (this->tcc != nullptr)
            GlidingStatus::publish(this->tcc->gliding_status(GET_CLOCK_TIME()));
        TccTrace::append(cycle_trace);
        monitor_loop.finish();
        uint32_t time = GET_CLOCK_TIME() - start;
        if (time < 20) {
            vTaskDelay((20 - time) / portTICK_PERIOD_MS); // 50 updates/sec!
        }
    }
}

bool Gearbox::process_speed_sensors(bool* raw_signals_valid)
{
    bool ok = true;
    bool conduct_sanity_check = gear_disagree_count == 0 &&
        (this->actual_gear == this->target_gear) && (                                                 // Same gear (Not shifting)
            (this->actual_gear == GearboxGear::Second) || // And in 2..
            (this->actual_gear == GearboxGear::Third) ||  // .. or 3 ..
            (this->actual_gear == GearboxGear::Fourth)    // .. or 4
            );
    uint16_t n2 = TCUIO::n2_rpm();
    uint16_t n3 = TCUIO::n3_rpm();
    uint16_t output = TCUIO::output_rpm();
    if (raw_signals_valid != nullptr) {
        // Export validity from the same sample as the control decision.
        *raw_signals_valid = n2 != UINT16_MAX && n3 != UINT16_MAX && output != UINT16_MAX;
    }

    if (UINT16_MAX != n2 && UINT16_MAX != n3) {
        uint16_t turbine = TCUIO::calc_turbine_rpm(n2, n3);
        if (conduct_sanity_check) {
            if (abs(n2 - n3) > 100) {
                ok = false;
            }
        }
        if (ok) {
            this->speed_sensors.turbine = turbine;
        }
        this->speed_sensors.n2 = n2;
        this->speed_sensors.n3 = n3;
    }

    if (UINT16_MAX == n2 || UINT16_MAX == n3) {
        ok = false; // A cached turbine value is not a current input measurement.
    }

    if (UINT16_MAX != output) {
        speed_sensors.output = output;
    }
    else {
        ok = false; // Output RPM failed
    }

    return ok;
}

bool Gearbox::calcGearFromRatio(bool is_reverse)
{
    float ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
    //ESP_LOGI("CGFR", "R %.3f", ratio);
    if (is_reverse)
    {
        ratio *= -1;
        for (uint8_t i = 0; i < 2; i++)
        { // Scan the 2 reverse gears
            GearRatioInfo limits = gearboxConfig.bounds[i + 5];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                //ESP_LOGI("CGFR", "G %d", i+1);
                this->est_gear_idx = i + 1;
                return true;
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 5; i++)
        { // Scan the 5 forwards gears
            GearRatioInfo limits = gearboxConfig.bounds[i];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                this->est_gear_idx = i + 1;
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}

Gearbox* gearbox = nullptr;


