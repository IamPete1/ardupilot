#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_EFI/AP_EFI.h>

class AP_BattMonitor_EFI : public AP_BattMonitor_Backend
{
public:

    // Constructor
    AP_BattMonitor_EFI(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    // Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    // returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    // returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    // dont allow reset for fuel cell
    bool reset_remaining(float percentage) override { return false;};

    void init(void) override {}

protected:

};
