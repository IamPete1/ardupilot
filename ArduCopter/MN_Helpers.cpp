#include "Copter.h"

#define TIME_TO_ALTHOLD 350 //Time it takes before changing automatically from Loiter to altHold when the conditions are met
#define TIME_TO_LOITER 30   //Time it takes before changing automatically from ALtHold to Loiter when the conditions are met

// This function checks at 10 hz the conditions of flight mode
// and viso status for automatic flight mode and ekf source change
void Copter::auto_flight_mode_check_loop(void) {

    // if not auto mode switch flag return inmediately
    if (!copter.mn_auto_mode_switch || !copter.g2.auto_mode_switch_enabled) {
        mn_auto_mode_switch_engaged = false;
        return;
    }

    bool confidence_ok = copter.visual_odom.confidence_ok();
    bool in_loiter = copter.flightmode == &copter.mode_loiter;
    bool in_althold = copter.flightmode == &copter.mode_althold;

    if (in_loiter && confidence_ok) {
        mn_auto_mode_switch_engaged = false;
        return;
    }
    
    if (in_althold && !confidence_ok) {
        mn_auto_mode_switch_engaged = false;
        return;
    }

    if (!mn_auto_mode_switch_engaged) {
        mn_auto_mode_switch_engaged = true;
        mn_auto_mode_switch_time = millis();
        return;
    }

    // If we are here it means we are waiting for the timer to change flight mode
    // we initialize them to change to alt hold from loiter
    uint32_t time_to_change_flight_mode = TIME_TO_ALTHOLD;
    Mode::Number mode_to_change = Mode::Number::ALT_HOLD;

    // Set up the proper thresholds for each flight mode transition
    if (in_althold) {
        if (change_to_loiter_requested) { // don't proceed if check_request_change_to_loiter is already waiting for the timer to expire
            return;
        }
        time_to_change_flight_mode = TIME_TO_LOITER;
        mode_to_change = Mode::Number::LOITER;

    } else if (!in_loiter) { // sanity check, we should never be here
        mn_auto_mode_switch_engaged = false;
        return;
    }
        
    // check if timer has expired and command flight mode change
    if (millis() - mn_auto_mode_switch_time >= time_to_change_flight_mode) {
        copter.set_mode(mode_to_change, ModeReason::SCRIPTING);
    }  
}

// This function checks at 20 hz if we requested a change to loiter 
// mode manually. In order to start a timer
void Copter::check_request_change_to_loiter(void) {
    
    if (!g2.auto_mode_switch_enabled) {
        return;
    }


    if (!change_to_loiter_requested) {
        return;
    }
    
    // Sanity check. We should never be here. If we are in loiter reset state and return inmediately
    if (copter.flightmode == &copter.mode_loiter) {
        change_to_loiter_allowed = false;
        change_to_loiter_requested = false;           
        loiter_timer_running = false; 
        return;
    }

    if (!loiter_timer_running) {
        loiter_timer_running = true;
        mn_loiter_mode_switch_time = millis();
        return;
    }
    
    if (millis() - mn_loiter_mode_switch_time >= copter.g2.auto_mode_switch_time_to_loiter ) {
        change_to_loiter_allowed = true;
        copter.set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
    } 
}