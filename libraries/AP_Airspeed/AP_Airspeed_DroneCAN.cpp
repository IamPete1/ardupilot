#include "AP_Airspeed_DroneCAN.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_Airspeed_DroneCAN::DetectedModules AP_Airspeed_DroneCAN::_detected_modules[];
HAL_Semaphore AP_Airspeed_DroneCAN::_sem_registry;

bool AP_Airspeed_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_airspeed, driver_index) != nullptr)
#if AP_AIRSPEED_HYGROMETER_ENABLE
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_hygrometer, driver_index) != nullptr)
#endif
    ;
}

AP_Airspeed_Backend* AP_Airspeed_DroneCAN::probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* backend = nullptr;

    // If an override is set use the slot the node has already been detected in,
    // otherwise take an empty slot so no other detected module is lost,
    // failing that take the slot of a detected module no instance has reserved
    const uint8_t override_node_id = _frontend.get_can_override_node_id(_instance);
    if (override_node_id > 0) {
        int8_t slot = -1;
        int8_t unreserved_slot = -1;
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if (_detected_modules[i].driver != nullptr) {
                continue;
            }
            if (_detected_modules[i].ap_dronecan != nullptr && _detected_modules[i].node_id == override_node_id) {
                slot = i;
                break;
            }
            if (_detected_modules[i].ap_dronecan == nullptr) {
                if (slot < 0) {
                    slot = i;
                }
            } else if (unreserved_slot < 0 && !node_id_overridden(_frontend, _detected_modules[i].node_id)) {
                unreserved_slot = i;
            }
        }
        if (slot < 0 && unreserved_slot < 0) {
            return nullptr;
        }
        backend = NEW_NOTHROW AP_Airspeed_DroneCAN(_frontend, _instance);
        if (backend == nullptr) {
            return nullptr;
        }
        if (slot < 0) {
            // no empty slot, drop a detected module that no instance has reserved
            slot = unreserved_slot;
            _detected_modules[slot].ap_dronecan = nullptr;
        }
        _detected_modules[slot].driver = backend;
        if (_detected_modules[slot].ap_dronecan != nullptr) {
            // already detected, bus is known
            backend->set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                            _detected_modules[slot].ap_dronecan->get_driver_index(),
                                                            override_node_id, 0));
        } else {
            // Not yet detected, the bus number and devid are filled in at run time
            _detected_modules[slot].node_id = override_node_id;
            backend->clear_bus_id();
        }
        return backend;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_dronecan != nullptr) {
            if (node_id_overridden(_frontend, _detected_modules[i].node_id)) {
                // reserved for an instance with a node id override
                continue;
            }
            const auto bus_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                            _detected_modules[i].ap_dronecan->get_driver_index(),
                                                            _detected_modules[i].node_id, 0);
            if (previous_devid != 0 && previous_devid != bus_id) {
                // match with previous ID only
                continue;
            }
            backend = NEW_NOTHROW AP_Airspeed_DroneCAN(_frontend, _instance);
            if (backend != nullptr) {
                _detected_modules[i].driver = backend;
                backend->set_bus_id(bus_id);
            }
            break;
        }
    }

    return backend;
}

// return true if any DroneCAN airspeed instance has its node id override set to this node
bool AP_Airspeed_DroneCAN::node_id_overridden(const AP_Airspeed &_frontend, uint8_t node_id)
{
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_frontend.get_can_override_node_id(i) == node_id) {
            return true;
        }
    }
    return false;
}

AP_Airspeed_DroneCAN* AP_Airspeed_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if ((_detected_modules[i].driver != nullptr) && (_detected_modules[i].node_id == node_id)) {
            // A driver without a bus is waiting for its override node id, bind it to this bus
            if (_detected_modules[i].ap_dronecan == nullptr) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                // Set the correct devid now the bus is known
                _detected_modules[i].driver->set_bus_id(
                    AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                ap_dronecan->get_driver_index(),
                                                node_id,
                                                0)
                );
            }

            if (_detected_modules[i].ap_dronecan == ap_dronecan) {
                return _detected_modules[i].driver;
            }
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_dronecan == ap_dronecan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if ((_detected_modules[i].ap_dronecan == nullptr) && (_detected_modules[i].driver == nullptr)) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_Airspeed_DroneCAN::handle_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_pressure = msg.differential_pressure;
        if (!isnan(msg.static_air_temperature) &&
            msg.static_air_temperature > 0) {
            driver->_temperature = KELVIN_TO_C(msg.static_air_temperature);
            driver->_have_temperature = true;
        }
        driver->_last_sample_time_ms = AP_HAL::millis();
    }
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
void AP_Airspeed_DroneCAN::handle_hygrometer(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_hygrometer.temperature = KELVIN_TO_C(msg.temperature);
        driver->_hygrometer.humidity = msg.humidity;
        driver->_hygrometer.last_sample_ms = AP_HAL::millis();
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

bool AP_Airspeed_DroneCAN::init()
{
    // always returns true
    return true;
}

bool AP_Airspeed_DroneCAN::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 250) {
        return false;
    }

    pressure = _pressure;

    return true;
}

bool AP_Airspeed_DroneCAN::get_temperature(float &temperature)
{
    if (!_have_temperature) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    temperature = _temperature;

    return true;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
/*
  return hygrometer data if available
 */
bool AP_Airspeed_DroneCAN::get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity)
{
    if (_hygrometer.last_sample_ms == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);
    last_sample_ms = _hygrometer.last_sample_ms;
    temperature = _hygrometer.temperature;
    humidity = _hygrometer.humidity;
    return true;
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE
#endif // AP_AIRSPEED_DRONECAN_ENABLED
