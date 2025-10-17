#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_INA260_ENABLED

class AP_BattMonitor_INA260 : public AP_BattMonitor_Backend
{
public:

    AP_BattMonitor_INA260(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_current() const override { return true; }

    void init() override;
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_HAL::I2CDevice *dev;

    void configure(void);
    bool read_word(const uint8_t reg, uint16_t& data) const;
    bool write_word(const uint8_t reg, const uint16_t data) const;
    void timer(void);

    bool configured;
    uint8_t failed_reads;
    uint8_t configure_counter;

    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;

    AP_Int8 i2c_bus;
    AP_Int8 i2c_address;
};

#endif // AP_BATTERY_INA260_ENABLED
