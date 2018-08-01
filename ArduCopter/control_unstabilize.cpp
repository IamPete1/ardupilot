#include "Copter.h"

/*
 * Init and run calls for sport flight mode
 */

// Custom direct passthrough throttle mode 
bool Copter::unstabilize_init(bool ignore_checks)
{
 // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
           (get_pilot_desired_throttle(channel_throttle->get_control_in(), g2.acro_thr_mid) > get_non_takeoff_throttle())) {
       return false;
   }
   // set target altitude to zero for reporting
   pos_control->set_alt_target(0);

   return true;
}

// Custom direct passthrough throttle mode 
void Copter::unstabilize_run()
{
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in(), g2.acro_thr_mid);

    // output pilot's throttle direct to motors, no stabalisation
    attitude_control->set_throttle_out_unstabilized(pilot_throttle_scaled, true, g.throttle_filt);
}
