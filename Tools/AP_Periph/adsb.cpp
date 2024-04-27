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

// Support for full ADSB lib. This allows the tracking to be done on locally minimizing CAN bus bandwidth

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#ifdef HAL_ADSB_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  init ADSB library
 */
void AP_Periph_FW::adsb_init()
{
    if (g.adsb_baudrate <= 0) {
        // Invalid baud rate
        return;
    }

    auto *uart = hal.serial(g.adsb_port);
    if (uart == nullptr) {
        // Invalid port
        return;
    }

    // Begin serial port and set protocol so the ADSB library can find it
    uart->begin(AP_SerialManager::map_baudrate(g.adsb_baudrate));
    serial_manager.set_protocol_and_baud(g.adsb_port, AP_SerialManager::SerialProtocol_ADSB, g.adsb_baudrate);

    // healthy calls "check_startup" which in turn runs init
    adsb.init_success = adsb_lib.healthy();
}

/*
  update ADSB library
 */
void AP_Periph_FW::adsb_update()
{
    if (!adsb.init_success) {
        // Lib not running
        return;
    }

    // Rate limit to 10Hz, this is the same update rate that vehicles use
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - adsb.last_update_ms) < 100) {
        return;
    }
    adsb.last_update_ms = now_ms;

    adsb_lib.update();
}

// Get next ADSB vehicle from the list and send it at the configured rate
void AP_Periph_FW::can_adsb_update()
{
    if (!adsb.init_success) {
        // Lib not running
        return;
    }

    if (g.adsb_msg_rate <= 0) {
        // No configured to send
        return;
    }

    // Rate limit given by parameter
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - adsb.last_send_ms < (1000U / g.adsb_msg_rate)) {
        return;
    }
    adsb.last_send_ms = now_ms;

    // Note that because of the way the ADSB list works it is possible to miss vehicles using this simple approach.
    // When items are removed from the list the last item is copied from the end to the index that is to be removed, the list length is then decremented by 1.
    // If were scanning in the middle of the list at this time the item could move to a point earlier in the list, thus is its missed in this list scan.
    // This could happen several times until the "hidden" item is at the start of the list.

    const uint16_t list_len = adsb_lib.get_vehicle_count();
    for (uint8_t i = 0; i < list_len; i++) {
        // Send each item in turn
        const uint8_t index = (adsb.last_sent_index + 1 + i) % list_len;

        AP_ADSB::adsb_vehicle_t vehicle;
        if (adsb_lib.get_vehicle(index, vehicle)) {
            // Found a vehicle, send it
            can_send_adsb_vehicle(vehicle);
            adsb.last_sent_index = index;
            break;
        }
    }

}

// Convert from adsb_vehicle_t to CAN message and send
// This is the inverse of the `handle_traffic_report` method found here:
// https://github.com/ArduPilot/ardupilot/blob/54bfaa4438a8e7112b0f2a539d446093ea7abcc6/libraries/AP_DroneCAN/AP_DroneCAN.cpp#L1293C19-L1293C40
void AP_Periph_FW::can_send_adsb_vehicle(const AP_ADSB::adsb_vehicle_t& vehicle)
{
    ardupilot_equipment_trafficmonitor_TrafficReport msg {};

    // Helper to get at info more easily
    const mavlink_adsb_vehicle_t &pkt = vehicle.info;

    msg.icao_address = pkt.ICAO_address;

    // Update the time since last contact for the current time
    // The other end converts this back into a "last update" timestamp.
    // Going via tslc removes any difference between clocks.
    // The bus transport time is lost.
    msg.tslc = (AP_HAL::millis() - vehicle.last_update_ms) * 0.001;

    msg.latitude_deg_1e7 = pkt.lat;
    msg.longitude_deg_1e7 = pkt.lon;
    msg.alt_m = pkt.altitude * 0.001;
    msg.heading = radians(pkt.heading * 0.01);

    // This assumes ground course and heading are the same.
    // The receive function just uses the length, so we get away with it.
    Vector2f vel_xy {};
    vel_xy.offset_bearing(pkt.heading * 0.01, pkt.hor_velocity * 0.01);
    msg.velocity[0] = vel_xy.x;
    msg.velocity[1] = vel_xy.y;

    msg.velocity[2] = pkt.ver_velocity * -0.01;
    msg.squawk = pkt.squawk;
    for (uint8_t i=0; i<MIN(ARRAY_SIZE(msg.callsign), ARRAY_SIZE(pkt.callsign)); i++) {
        msg.callsign[i] = pkt.callsign[i];
    }
    msg.traffic_type = pkt.emitter_type;

    if ((pkt.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0) {
        // Have valid altitude
        if (pkt.altitude_type == ADSB_ALTITUDE_TYPE_PRESSURE_QNH) {
            msg.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL;

        } else if (pkt.altitude_type == ADSB_ALTITUDE_TYPE_GEOMETRIC) {
            msg.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84;
        }
    }

    msg.lat_lon_valid = (pkt.flags & ADSB_FLAGS_VALID_COORDS) != 0;
    msg.heading_valid = (pkt.flags & ADSB_FLAGS_VALID_HEADING) != 0;
    msg.velocity_valid = (pkt.flags & ADSB_FLAGS_VALID_VELOCITY) != 0;
    msg.callsign_valid = (pkt.flags & ADSB_FLAGS_VALID_CALLSIGN) != 0;
    msg.ident_valid = (pkt.flags & ADSB_FLAGS_VALID_SQUAWK) != 0;
    msg.simulated_report = (pkt.flags & ADSB_FLAGS_SIMULATED) != 0;
    msg.vertical_velocity_valid = (pkt.flags & ADSB_FLAGS_VERTICAL_VELOCITY_VALID) != 0;
    msg.baro_valid = (pkt.flags & ADSB_FLAGS_BARO_VALID) != 0;

    uint8_t buffer[ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_MAX_SIZE] {};
    uint16_t total_size = ardupilot_equipment_trafficmonitor_TrafficReport_encode(&msg, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SIGNATURE,
                    ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ID,
                    CANARD_TRANSFER_PRIORITY_LOWEST,
                    &buffer[0],
                    total_size);
}

#endif // HAL_ADSB_ENABLED
