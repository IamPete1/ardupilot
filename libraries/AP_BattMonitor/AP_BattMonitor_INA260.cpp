#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA260_ENABLED

#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_INA260.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_INA260::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number.
    // @Description: Battery monitor I2C bus number.
    // @Range: 0 3
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 7, AP_BattMonitor_INA260, i2c_bus, 0),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address.
    // @Description: Battery monitor I2C address.
    // @Range: 0 127
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 8, AP_BattMonitor_INA260, i2c_address, 0x40),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_BattMonitor_INA260::AP_BattMonitor_INA260(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_INA260::init(void)
{
    dev = hal.i2c_mgr->get_device_ptr(i2c_bus, i2c_address, 100000, false, 20);
    if (!dev) {
        return;
    }
    // Run at 25Hz
    dev->register_periodic_callback(40000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA260::timer, void));
}

/*
  read 16 bit word from register
  returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA260::read_word(const uint8_t reg, uint16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = be16toh(data);

    return true;
}

/*
  write word to a register, byte swapped
  returns true if write was successful, false if failed
*/
bool AP_BattMonitor_INA260::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

void AP_BattMonitor_INA260::configure(void)
{

    // Check manufacturer ID
    uint16_t mfg_id;
    if (!read_word(0xfe, mfg_id) ||
        mfg_id != 0x5449) {
        return;
    }

    // Check device ID
    uint16_t device_id;
    if (!read_word(0xff, device_id) ||
        (device_id >> 4) != 0x227) {
        return;
    }

    // Reset device
    if (!write_word(0x00, 0x8000)) {
        return;
    }

    // Set longest conversion time and no averaging
    // This is 8.244ms for both voltage and current
    // So both should have new readings after 16.488ms
    // This is a new reading at 60Hz, we read at 25Hz
    // Continuous voltage and current measurement
    if (!write_word(0x00, 0x01ff)) {
        return;
    }

    failed_reads = 0;
    configured = true;
}

void AP_BattMonitor_INA260::timer(void)
{
    // Re-configure
    if (!configured) {
        configure_counter++;
        if (configure_counter > 5) {
            // Try configuring every 5th loop (at 5Hz)
            configure();
            configure_counter = 0;
        }
        if (!configured) {
            // waiting for the device to respond
            return;
        }
    }

    uint16_t voltage, current;
    if (!read_word(0x02, voltage) ||
        !read_word(0x01, current)) {
        failed_reads++;
        if (failed_reads > 10) {
            // device has disconnected, we need to reconfigure it
            configured = false;
        }
        return;
    }
    failed_reads = 0;

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.volt_sum += int16_t(voltage) * 0.00125;
    accumulate.current_sum += int16_t(current) * 0.00125;
    accumulate.count++;
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA260::read(void)
{
    WITH_SEMAPHORE(accumulate.sem);
    _state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.volt_sum / accumulate.count;
    _state.current_amps = accumulate.current_sum / accumulate.count;
    accumulate.volt_sum = 0;
    accumulate.current_sum = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;

    // update total current drawn since startup
    update_consumed(_state, dt_us);

    _state.last_time_micros = tnow;
}


#endif // AP_BATTERY_INA260_ENABLED
