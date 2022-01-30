/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Generator_IE_PPM.h"

#if GENERATOR_ENABLED

extern const AP_HAL::HAL& hal;

AP_Generator_IE_PPM *AP_Generator_IE_PPM::_singleton;


AP_Generator_IE_PPM::AP_Generator_IE_PPM(AP_Generator& frontend)
    : AP_Generator_IE_FuelCell(frontend)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Generator_IE_PPM must be singleton");
    }
    _singleton = this;
}

// Update fuel cell, expected to be called at 20hz
void AP_Generator_IE_PPM::assign_measurements(const uint32_t now)
{
    _PPM_last = _PPM_parsed;
    _err_code = _PPM_last.fcpm_1.err_code | _PPM_last.fcpm_2.err_code;

    gcs().send_named_float("TANK_PRESS", _PPM_last.tank_pressure);

    _last_time_ms = now;
}

// Process characters received and extract terms for IE 650/800 W
void AP_Generator_IE_PPM::decode_latest_term()
{
    // Null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;

    switch (_term_number) {
        case 1:
            // timestamp - not used
            break;

        // FCPM 1
        case 2:
            _PPM_parsed.fcpm_1.tank_pct = atof(_term);
            // Out of range values
            if (_PPM_parsed.fcpm_1.tank_pct > 100.0f || _PPM_parsed.fcpm_1.tank_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 3:
            _PPM_parsed.fcpm_1.battery_pct = atof(_term);
            // Out of range values
            if (_PPM_parsed.fcpm_1.battery_pct > 100.0f || _PPM_parsed.fcpm_1.battery_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 4:
            _PPM_parsed.fcpm_1.state = (State)strtoul(_term, nullptr, 10);
            break;
        
        case 5:
            _PPM_parsed.fcpm_1.err_code = strtoul(_term, nullptr, 16);
            break;


        // FCPM 2
        case 6:
            _PPM_parsed.fcpm_2.tank_pct = atof(_term);
            // Out of range values
            if (_PPM_parsed.fcpm_2.tank_pct > 100.0f || _PPM_parsed.fcpm_2.tank_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 7:
            _PPM_parsed.fcpm_2.battery_pct = atof(_term);
            // Out of range values
            if (_PPM_parsed.fcpm_2.battery_pct > 100.0f || _PPM_parsed.fcpm_2.battery_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 8:
            _PPM_parsed.fcpm_2.state = (State)strtoul(_term, nullptr, 10);
            break;
        
        case 9:
            _PPM_parsed.fcpm_2.err_code = strtoul(_term, nullptr, 16);
            break;

        // PPM
        case 10:
            _PPM_parsed.tank_pressure = atof(_term);
            break;

        case 11:
            _PPM_parsed.output_power = atof(_term);
            break;

        case 12:
            _PPM_parsed.output_voltage = atof(_term);
            break;

        case 13:
            _PPM_parsed.output_current = atof(_term);
            break;

        case 14:
            _PPM_parsed.bit_statuses = strtoul(_term, nullptr, 16);
            _sentence_valid = _data_valid;
            break;

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }
}

// Check for failsafes
AP_BattMonitor::Failsafe AP_Generator_IE_PPM::update_failsafes() const
{
    if ((_PPM_last.fcpm_1.state != State::RUNNING) || (_PPM_last.fcpm_2.state != State::RUNNING)) {
        return  AP_BattMonitor::Failsafe::Critical;
    }

    // Check if we are in a critical failsafe
    if ((_err_code & fs_crit_mask) != 0) {
        return  AP_BattMonitor::Failsafe::Critical;
    }

    // Check if we are in a low failsafe
    if ((_err_code & fs_low_mask) != 0) {
        return  AP_BattMonitor::Failsafe::Low;
    }

    return AP_BattMonitor::Failsafe::None;
}

// Check for arming
bool AP_Generator_IE_PPM::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // Refuse arming if not healthy
    if (!healthy()) {
        strncpy(failmsg, "Not healthy", failmsg_len);
        return false;
    }

    if (_PPM_last.fcpm_1.state != State::RUNNING) {
        strncpy(failmsg, "Status 1 not running", failmsg_len);
        return false;
    }
    if (_PPM_last.fcpm_2.state != State::RUNNING) {
        strncpy(failmsg, "Status 2 not running", failmsg_len);
        return false;
    }

    // Check for error codes
    if (check_for_err_code(failmsg, failmsg_len)) {
        return false;
    }

    return true;
}

// Lookup table for running state.  State code is the same for all IE units.
const AP_Generator_IE_PPM::Lookup_State AP_Generator_IE_PPM::lookup_state[] = {
    { State::STARTING,"Starting"},
    { State::READY,"Ready"},
    { State::RUNNING,"Running"},
    { State::FAULT,"Fault"},
    { State::BATTERY_ONLY,"Battery Only"},
};

// Check for any change in error state or status and report to gcs
void AP_Generator_IE_PPM::check_status()
{
    // Check driver health
    if (!healthy() && !_health_warn_sent) {
        // Don't spam GCS with unhealthy message
        _health_warn_sent = true;
        gcs().send_text(MAV_SEVERITY_ALERT, "Generator: Driver Not healthy");

    } else if (healthy()) {
        _health_warn_sent = false;
    }

    if (_PPM_last.fcpm_1.state != _last_state_1) {
        for (const struct Lookup_State entry : lookup_state) {
            if (_PPM_last.fcpm_1.state == entry.option) {
                gcs().send_text(MAV_SEVERITY_INFO, "Generator PPM 1: %s", entry.msg_txt);
                break;
            }
        }
        _last_state_1 = _PPM_last.fcpm_1.state;
    }

    if (_PPM_last.fcpm_2.state != _last_state_2) {
        for (const struct Lookup_State entry : lookup_state) {
            if (_PPM_last.fcpm_2.state == entry.option) {
                gcs().send_text(MAV_SEVERITY_INFO, "Generator PPM 2: %s", entry.msg_txt);
                break;
            }
        }
        _last_state_2 = _PPM_last.fcpm_2.state;
    }

    // Check error codes
    char msg_txt[32];
    if (check_for_err_code_if_changed(msg_txt, sizeof(msg_txt))) {
        gcs().send_text(MAV_SEVERITY_ALERT, "%s", msg_txt);
    }
}

// Check error codes and populate message with error code
bool AP_Generator_IE_PPM::check_for_err_code(char* msg_txt, uint8_t msg_len) const
{
    if ((_PPM_last.fcpm_1.err_code & (fs_crit_mask | fs_low_mask)) != 0) {
        hal.util->snprintf(msg_txt, msg_len, "Fuel cell 1 err code <0x%x>", (unsigned)_PPM_last.fcpm_1.err_code);
        return true;
    }

    if ((_PPM_last.fcpm_2.err_code & (fs_crit_mask | fs_low_mask)) != 0) {
        hal.util->snprintf(msg_txt, msg_len, "Fuel cell 2 err code <0x%x>", (unsigned)_PPM_last.fcpm_2.err_code);
        return true;
    }

    return false;
}

#endif
