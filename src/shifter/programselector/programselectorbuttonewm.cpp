#include "programselectorbuttonewm.h"
#include "nvs/eeprom_config.h"

namespace {
// With only Comfort and Agility enabled, use the requested C/A-only cycle.
// Other profile configurations keep the upstream selection behavior.
bool comfort_agility_only() {
    return ETS_CURRENT_SETTINGS.ewm_enable_c &&
           ETS_CURRENT_SETTINGS.ewm_enable_a &&
           !ETS_CURRENT_SETTINGS.ewm_enable_w &&
           !ETS_CURRENT_SETTINGS.ewm_enable_m &&
           !ETS_CURRENT_SETTINGS.ewm_enable_r &&
           !ETS_CURRENT_SETTINGS.ewm_custom_profile_btn;
}
}



ProgramSelectorButtonEwm::ProgramSelectorButtonEwm(TCM_CORE_CONFIG *vehicle_config): vehicle_config(vehicle_config)
{
	if (comfort_agility_only()) {
		profile_id = PROFILE_IDX_C;
		return;
	}

	// Read profile ID on startup based on TCM config
	profile_id = VEHICLE_CONFIG.default_profile;
	// Try to load from NVS
	if (ETS_CURRENT_SETTINGS.ewm_save_profile) {
		esp_err_t load_res = EEPROM::ewm_btn_get_saved_profile(&profile_id);
		if (ESP_OK != load_res) {
			ESP_LOGW("EWM PROFILE", "Last saved profile couldn't be loaded from NVS");
		}
	}

	bool modified_default = false;
	if (profile_id == PROFILE_IDX_C && !ETS_CURRENT_SETTINGS.ewm_enable_c) {
		profile_id ++;
		modified_default = true;
	}
	if (profile_id == PROFILE_IDX_W && !ETS_CURRENT_SETTINGS.ewm_enable_w) {
		profile_id ++;
		modified_default = true;
	}
	if (profile_id == PROFILE_IDX_A && !ETS_CURRENT_SETTINGS.ewm_enable_a) {
		profile_id ++;
		modified_default = true;
	}
	if (profile_id == PROFILE_IDX_M && !ETS_CURRENT_SETTINGS.ewm_enable_m) {
		profile_id ++;
		modified_default = true;
	}
	if (profile_id == PROFILE_IDX_R && !ETS_CURRENT_SETTINGS.ewm_enable_r) {
		profile_id ++;
		modified_default = true;
	}
	if (profile_id >= NUM_PROFILES)
	{
		profile_id = 0u;
	}
	ESP_LOGW("EWM PROFILE", "Startup profile is disabled in TCU Program settings, switching to next available profile");
}

void ProgramSelectorButtonEwm::set_button_pressed(const bool is_pressed)
{
	if (comfort_agility_only()) {
        // Ignore missing CAN frames and require a released button after boot.
        // A button already held at startup must not change the initial C mode.
        if (egs_can_hal == nullptr ||
            egs_can_hal->get_profile_switch_pos(500) == ProfileSwitchPos::SNV) {
            return;
        }
        static bool released_after_startup = false;
        if (!released_after_startup) {
            is_pressed_last_call = is_pressed;
            released_after_startup = !is_pressed;
            return;
        }
		if (is_pressed && !is_pressed_last_call) {
			profile_id = profile_id == PROFILE_IDX_C ? PROFILE_IDX_A : PROFILE_IDX_C;
		}
		is_pressed_last_call = is_pressed;
		return;
	}

	if(is_pressed && (is_pressed != is_pressed_last_call)) {
		// button is pressed and was not pressed before
		profile_id++;
		// Check for additional profiles
		if (profile_id == PROFILE_IDX_C && !ETS_CURRENT_SETTINGS.ewm_enable_c) {
			profile_id ++;
		}
		if (profile_id == PROFILE_IDX_W && !ETS_CURRENT_SETTINGS.ewm_enable_w) {
			profile_id ++;
		}
		if (profile_id == PROFILE_IDX_A && !ETS_CURRENT_SETTINGS.ewm_enable_a) {
			profile_id ++;
		}
		if (profile_id == PROFILE_IDX_M && !ETS_CURRENT_SETTINGS.ewm_enable_m) {
			profile_id ++;
		}
		if (profile_id == PROFILE_IDX_R && !ETS_CURRENT_SETTINGS.ewm_enable_r) {
			profile_id ++;
		}
		if (profile_id >= NUM_PROFILES)
		{
			profile_id = 0u;
		}
	}
	is_pressed_last_call = is_pressed;
}

AbstractProfile *ProgramSelectorButtonEwm::get_profile(const uint32_t expire_time_ms)
{
	return profiles[profile_id];
}

ProgramSelectorType ProgramSelectorButtonEwm::get_type() const {
	return ProgramSelectorType::EWMButton;
}

DiagProfileInputState ProgramSelectorButtonEwm::get_input_raw() const {
	DiagProfileInputState pos = DiagProfileInputState::SNV;
	if (this->is_pressed_last_call) {
		pos = DiagProfileInputState::ButtonPressed;
	} else {
		pos = DiagProfileInputState::ButtonReleased;
	}
	return pos;
}
