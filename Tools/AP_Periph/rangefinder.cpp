#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER

/*
  rangefinder support
 */

#include <dronecan_msgs.h>

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

/*
  update CAN rangefinder
 */
void AP_Periph_FW::can_rangefinder_update(void)
{

    uint32_t now = AP_HAL::millis();
    if (g.rangefinder_max_rate > 0 &&
        now - last_rangefinder_update_ms < uint32_t(1000/(g.rangefinder_max_rate*rangefinder.num_sensors()))) {
        // limit to max rate per instance
        return;
    }


#if AP_PERIPH_PROBE_CONTINUOUS
    if (rangefinder.num_sensors() < RANGEFINDER_MAX_INSTANCES) {
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif

    // update all rangefinder instances
    rangefinder.update();

    // cycle through each rangefinder instance to find one to send
    for (uint8_t i = 0; i < rangefinder.num_sensors(); i++) {

        // Send each sensor in turn
        const uint8_t instance = (rangefinder_last_sent_index + 1 + i) % rangefinder.num_sensors();

        if (rangefinder.get_type(instance) == RangeFinder::Type::NONE) {
            continue;
        }

        RangeFinder::Status status = rangefinder.status_instance(instance);
        if (status <= RangeFinder::Status::NoData) {
            // don't send any data for this instance
            continue;
        }
        const uint32_t sample_ms = rangefinder.last_reading_ms_instance(instance);
        if (last_rangefinder_sample_ms == sample_ms) {
            continue;
        }
        last_rangefinder_sample_ms = sample_ms;

        uint16_t dist_cm = rangefinder.distance_cm_instance(instance);
        uavcan_equipment_range_sensor_Measurement pkt {};
        pkt.sensor_id = rangefinder.get_address(instance);
        switch (status) {
        case RangeFinder::Status::OutOfRangeLow:
            pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
            break;
        case RangeFinder::Status::OutOfRangeHigh:
            pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
            break;
        case RangeFinder::Status::Good:
            pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
            break;
        default:
            pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
            break;
        }
        switch (rangefinder.get_mav_distance_sensor_type_instance(instance)) {
        case MAV_DISTANCE_SENSOR_LASER:
            pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
            break;
        case MAV_DISTANCE_SENSOR_ULTRASOUND:
            pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR;
            break;
        case MAV_DISTANCE_SENSOR_RADAR:
            pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR;
            break;
        default:
            pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED;
            break;
        }

        pkt.range = dist_cm * 0.01;

        uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                        UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);

        rangefinder_last_sent_index = instance;
        last_rangefinder_update_ms = now;
        break;
    }
}

#endif // HAL_PERIPH_ENABLE_RANGEFINDER
