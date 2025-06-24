#include "AP_Airspeed_DroneCAN.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "AirSpeed"

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

    bool haveDetected = false;
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_dronecan != nullptr) {
            haveDetected = true;
            const auto bus_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                            _detected_modules[i].ap_dronecan->get_driver_index(),
                                                            _detected_modules[i].node_id, 0);
            if (previous_devid != 0 && previous_devid != bus_id) {
                // match with previous ID only
                continue;
            }
            AP_Airspeed_DroneCAN* backend = NEW_NOTHROW AP_Airspeed_DroneCAN(_frontend, _instance);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Failed register DroneCAN Airspeed Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_dronecan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Registered DroneCAN Airspeed Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_dronecan->get_driver_index());
                backend->set_bus_id(bus_id);
            }
            return backend;
        }
    }

    // If some sensors have been detected the user might be tying to swap to a new device
    if (haveDetected) {
        return nullptr;
    }

    // No detected sensors but a valid ID, declare the driver and hope the sensor turns up later
    if ((previous_devid == 0) || (AP_HAL::Device::devid_get_bus_type(previous_devid) != AP_HAL::Device::BUS_TYPE_UAVCAN)) {
        return nullptr;
    }

    // See if the driver is valid
    const uint8_t driver_index = AP_HAL::Device::devid_get_bus(previous_devid);
    AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(driver_index);
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    // Extract node ID
    const uint8_t node_id = AP_HAL::Device::devid_get_address(previous_devid);

    // Declare a new driver
    AP_Airspeed_DroneCAN* backend = NEW_NOTHROW AP_Airspeed_DroneCAN(_frontend, _instance);
    if (backend == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Failed register undetected DroneCAN Airspeed Node %d on Bus %d\n",
                            node_id,
                            driver_index);
        return nullptr;
    }

    // Set the detected variables as they are used at runtime to identify drivers
    _detected_modules[0] = {
        ap_dronecan: ap_dronecan,
        node_id: node_id,
        driver: backend
    };

    // Make sure the backend bus ID is set correctly
    backend->set_bus_id(previous_devid);

    AP::can().log_text(AP_CANManager::LOG_INFO,
                        LOG_TAG,
                        "Registered undetected DroneCAN Airspeed Node %d on Bus %d\n",
                        node_id,
                        driver_index);

    return backend;
}

AP_Airspeed_DroneCAN* AP_Airspeed_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
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
            if (_detected_modules[i].ap_dronecan == nullptr) {
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
