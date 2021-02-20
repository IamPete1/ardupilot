# auto generated readme

uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
int32_t|MAX(-INT32_MAX,INT32_MIN)|MIN(INT32_MAX,INT32_MAX)
int32_t|MAX(-INT32_MAX,INT32_MIN)|MIN(INT32_MAX,INT32_MAX)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
bool||
bool||
bool||
bool||
int32_t|MAX((-LOCATION_ALT_MAX_M*100+1),INT32_MIN)|MIN((LOCATION_ALT_MAX_M*100-1),INT32_MAX)
int32_t|MAX(-1800000000,INT32_MIN)|MIN(1800000000,INT32_MAX)
int32_t|MAX(-900000000,INT32_MIN)|MIN(900000000,INT32_MAX)
## mavlink_mission_item_int_t


## Parameter

### Parameter:set_and_save
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Parameter:set
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Parameter:get
argument|min|max
:---:|:---:|:---:

### Parameter:init
argument|min|max
:---:|:---:|:---:
string||


## Vector2f

### Vector2f:rotate
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Vector2f:is_zero
argument|min|max
:---:|:---:|:---:

### Vector2f:is_inf
argument|min|max
:---:|:---:|:---:

### Vector2f:is_nan
argument|min|max
:---:|:---:|:---:

### Vector2f:normalize
argument|min|max
:---:|:---:|:---:

### Vector2f:length
argument|min|max
:---:|:---:|:---:


## Vector3f

### Vector3f:scale
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Vector3f:cross
argument|min|max
:---:|:---:|:---:
Vector3f||

### Vector3f:dot
argument|min|max
:---:|:---:|:---:
Vector3f||

### Vector3f:is_zero
argument|min|max
:---:|:---:|:---:

### Vector3f:is_inf
argument|min|max
:---:|:---:|:---:

### Vector3f:is_nan
argument|min|max
:---:|:---:|:---:

### Vector3f:normalize
argument|min|max
:---:|:---:|:---:

### Vector3f:length
argument|min|max
:---:|:---:|:---:


## Location

### Location:get_distance_NE
argument|min|max
:---:|:---:|:---:
Location||

### Location:get_distance_NED
argument|min|max
:---:|:---:|:---:
Location||

### Location:get_bearing
argument|min|max
:---:|:---:|:---:
Location||

### Location:get_vector_from_origin_NEU
argument|min|max
:---:|:---:|:---:

### Location:offset_bearing
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Location:offset
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### Location:get_distance
argument|min|max
:---:|:---:|:---:
Location||


## Motors_6DoF
depends on `APM_BUILD_TYPE(APM_BUILD_ArduCopter)`

### Motors_6DoF:add_motor
argument|min|max
:---:|:---:|:---:
int8_t|MAX(0,INT8_MIN)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),INT8_MAX)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
bool||
uint8_t|MAX(0,0)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),UINT8_MAX)

### Motors_6DoF:init
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),UINT8_MAX)


## attitude_control
depends on `APM_BUILD_TYPE(APM_BUILD_ArduCopter)`

### attitude_control:set_offset_roll_pitch
argument|min|max
:---:|:---:|:---:
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### attitude_control:set_forward_enable
argument|min|max
:---:|:---:|:---:
bool||

### attitude_control:set_lateral_enable
argument|min|max
:---:|:---:|:---:
bool||


## frsky_sport

### frsky_sport:prep_number
argument|min|max
:---:|:---:|:---:
int32_t|MAX(INT32_MIN,INT32_MIN)|MIN(INT32_MAX,INT32_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### frsky_sport:sport_telemetry_push
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)
int32_t|MAX(-INT32_MAX,INT32_MIN)|MIN(INT32_MAX,INT32_MAX)


## MotorsMatrix
depends on `APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_TYPE(APM_BUILD_ArduCopter)`

### MotorsMatrix:add_motor_raw
argument|min|max
:---:|:---:|:---:
int8_t|MAX(0,INT8_MIN)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),INT8_MAX)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
uint8_t|MAX(0,0)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),UINT8_MAX)

### MotorsMatrix:init
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN((AP_MOTORS_MAX_NUM_MOTORS-1),UINT8_MAX)


## quadplane
depends on `APM_BUILD_TYPE(APM_BUILD_ArduPlane)`

### quadplane:in_vtol_mode
argument|min|max
:---:|:---:|:---:


## LED

### LED:get_rgb
argument|min|max
:---:|:---:|:---:


## button

### button:get_button_state
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(AP_BUTTON_NUM_PINS,UINT8_MAX)


## RPM

### RPM:get_rpm
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(RPM_MAX_INSTANCES,UINT8_MAX)


## mission

### mission:clear
argument|min|max
:---:|:---:|:---:

### mission:set_item
argument|min|max
:---:|:---:|:---:
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)
mavlink_mission_item_int_t||

### mission:get_item
argument|min|max
:---:|:---:|:---:
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### mission:num_commands
argument|min|max
:---:|:---:|:---:

### mission:get_current_do_cmd_id
argument|min|max
:---:|:---:|:---:

### mission:get_current_nav_id
argument|min|max
:---:|:---:|:---:

### mission:get_prev_nav_cmd_id
argument|min|max
:---:|:---:|:---:

### mission:set_current_cmd
argument|min|max
:---:|:---:|:---:
uint16_t|MAX(0,0)|MIN((ud->num_commands()-1),UINT16_MAX)

### mission:get_current_nav_index
argument|min|max
:---:|:---:|:---:

### mission:state
argument|min|max
:---:|:---:|:---:


## param

### param:set_and_save
argument|min|max
:---:|:---:|:---:
string||
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### param:set
argument|min|max
:---:|:---:|:---:
string||
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### param:get
argument|min|max
:---:|:---:|:---:
string||


## esc_telem

### esc_telem:get_usage_seconds
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(NUM_SERVO_CHANNELS,UINT8_MAX)


## baro

### baro:get_external_temperature
argument|min|max
:---:|:---:|:---:

### baro:get_temperature
argument|min|max
:---:|:---:|:---:

### baro:get_pressure
argument|min|max
:---:|:---:|:---:


## serial

### serial:find_serial
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)


## rc

### rc:find_channel_for_option
argument|min|max
:---:|:---:|:---:
enum||

### rc:get_pwm
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(NUM_RC_CHANNELS,UINT8_MAX)


## SRV_Channels

### SRV_Channels:set_range
argument|min|max
:---:|:---:|:---:
enum||
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### SRV_Channels:set_angle
argument|min|max
:---:|:---:|:---:
enum||
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### SRV_Channels:set_output_norm
argument|min|max
:---:|:---:|:---:
enum||
float|MAX(-1,-INFINITY)|MIN(1,INFINITY)

### SRV_Channels:get_output_scaled
argument|min|max
:---:|:---:|:---:
enum||

### SRV_Channels:get_output_pwm
argument|min|max
:---:|:---:|:---:
enum||

### SRV_Channels:set_output_scaled
argument|min|max
:---:|:---:|:---:
enum||
int16_t|MAX(INT16_MIN,INT16_MIN)|MIN(INT16_MAX,INT16_MAX)

### SRV_Channels:set_output_pwm_chan_timeout
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(NUM_SERVO_CHANNELS-1,UINT8_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### SRV_Channels:set_output_pwm_chan
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(NUM_SERVO_CHANNELS-1,UINT8_MAX)
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### SRV_Channels:set_output_pwm
argument|min|max
:---:|:---:|:---:
enum||
uint16_t|MAX(0,0)|MIN(UINT16_MAX,UINT16_MAX)

### SRV_Channels:find_channel
argument|min|max
:---:|:---:|:---:
enum||


## serialLED

### serialLED:send
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(16,UINT8_MAX)

### serialLED:set_RGB
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(16,UINT8_MAX)
int8_t|MAX(-1,INT8_MIN)|MIN(INT8_MAX,INT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### serialLED:set_num_profiled
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(16,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(AP_SERIALLED_MAX_LEDS,UINT8_MAX)

### serialLED:set_num_neopixel
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(1,0)|MIN(16,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(AP_SERIALLED_MAX_LEDS,UINT8_MAX)


## vehicle

### vehicle:get_wp_crosstrack_error_m
argument|min|max
:---:|:---:|:---:

### vehicle:get_wp_bearing_deg
argument|min|max
:---:|:---:|:---:

### vehicle:get_wp_distance_m
argument|min|max
:---:|:---:|:---:

### vehicle:set_steering_and_throttle
argument|min|max
:---:|:---:|:---:
float|MAX(-1,-INFINITY)|MIN(1,INFINITY)
float|MAX(-1,-INFINITY)|MIN(1,INFINITY)

### vehicle:set_target_angle_and_climbrate
argument|min|max
:---:|:---:|:---:
float|MAX(-180,-INFINITY)|MIN(180,INFINITY)
float|MAX(-90,-INFINITY)|MIN(90,INFINITY)
float|MAX(-360,-INFINITY)|MIN(360,INFINITY)
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)
bool||
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### vehicle:set_target_velocity_NED
argument|min|max
:---:|:---:|:---:
Vector3f||

### vehicle:get_target_location
argument|min|max
:---:|:---:|:---:

### vehicle:get_control_output
argument|min|max
:---:|:---:|:---:
enum||

### vehicle:set_target_location
argument|min|max
:---:|:---:|:---:
Location||

### vehicle:start_takeoff
argument|min|max
:---:|:---:|:---:
float|MAX((-LOCATION_ALT_MAX_M*100+1),-INFINITY)|MIN((LOCATION_ALT_MAX_M*100-1),INFINITY)

### vehicle:get_time_flying_ms
argument|min|max
:---:|:---:|:---:

### vehicle:get_likely_flying
argument|min|max
:---:|:---:|:---:

### vehicle:get_control_mode_reason
argument|min|max
:---:|:---:|:---:

### vehicle:get_mode
argument|min|max
:---:|:---:|:---:

### vehicle:set_mode
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)


## gcs

### gcs:send_named_float
argument|min|max
:---:|:---:|:---:
string||
float|MAX(-FLT_MAX,-INFINITY)|MIN(FLT_MAX,INFINITY)

### gcs:set_message_interval
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(MAVLINK_COMM_NUM_BUFFERS,UINT8_MAX)
uint32_t|MAX(0U,0U)|MIN(UINT32_MAX,UINT32_MAX)
int32_t|MAX(-1,INT32_MIN)|MIN(INT32_MAX,INT32_MAX)

### gcs:send_text
argument|min|max
:---:|:---:|:---:
enum||
string||


## relay

### relay:toggle
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(AP_RELAY_NUM_RELAYS,UINT8_MAX)

### relay:enabled
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(AP_RELAY_NUM_RELAYS,UINT8_MAX)

### relay:off
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(AP_RELAY_NUM_RELAYS,UINT8_MAX)

### relay:on
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(AP_RELAY_NUM_RELAYS,UINT8_MAX)


## terrain
depends on `defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1`

### terrain:height_above_terrain
argument|min|max
:---:|:---:|:---:
bool||

### terrain:height_terrain_difference_home
argument|min|max
:---:|:---:|:---:
bool||

### terrain:height_amsl
argument|min|max
:---:|:---:|:---:
Location||
bool||

### terrain:status
argument|min|max
:---:|:---:|:---:

### terrain:enabled
argument|min|max
:---:|:---:|:---:


## rangefinder

### rangefinder:get_pos_offset_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:has_data_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:status_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:ground_clearance_cm_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:min_distance_cm_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:max_distance_cm_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:distance_cm_orient
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:has_orientation
argument|min|max
:---:|:---:|:---:
enum||

### rangefinder:num_sensors
argument|min|max
:---:|:---:|:---:


## proximity

### proximity:get_object_angle_and_distance
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### proximity:get_closest_object
argument|min|max
:---:|:---:|:---:

### proximity:get_object_count
argument|min|max
:---:|:---:|:---:

### proximity:num_sensors
argument|min|max
:---:|:---:|:---:


## notify

### notify:handle_rgb
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### notify:play_tune
argument|min|max
:---:|:---:|:---:
string||


## gps

### gps:first_unconfigured_gps
argument|min|max
:---:|:---:|:---:

### gps:get_antenna_offset
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:have_vertical_velocity
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:last_message_time_ms
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:last_fix_time_ms
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:get_vdop
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:get_hdop
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:time_week_ms
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:time_week
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:num_sats
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:ground_course
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:ground_speed
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:velocity
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:vertical_accuracy
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:horizontal_accuracy
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:speed_accuracy
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:location
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:status
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_sensors(),UINT8_MAX)

### gps:primary_sensor
argument|min|max
:---:|:---:|:---:

### gps:num_sensors
argument|min|max
:---:|:---:|:---:


## battery

### battery:get_cycle_count
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:get_temperature
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:overpower_detected
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:has_failsafed
argument|min|max
:---:|:---:|:---:

### battery:pack_capacity_mah
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:capacity_remaining_pct
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:consumed_wh
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:consumed_mah
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:current_amps
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:voltage_resting_estimate
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:voltage
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:healthy
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(ud->num_instances(),UINT8_MAX)

### battery:num_instances
argument|min|max
:---:|:---:|:---:


## arming

### arming:set_aux_auth_failed
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)
string||

### arming:set_aux_auth_passed
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### arming:get_aux_auth_id
argument|min|max
:---:|:---:|:---:

### arming:arm
argument|min|max
:---:|:---:|:---:

### arming:is_armed
argument|min|max
:---:|:---:|:---:

### arming:disarm
argument|min|max
:---:|:---:|:---:


## ahrs

### ahrs:get_vel_innovations_and_variances_for_source
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(3,0)|MIN(6,UINT8_MAX)

### ahrs:set_posvelyaw_source_set
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(2,UINT8_MAX)

### ahrs:get_variances
argument|min|max
:---:|:---:|:---:

### ahrs:get_EAS2TAS
argument|min|max
:---:|:---:|:---:

### ahrs:body_to_earth
argument|min|max
:---:|:---:|:---:
Vector3f||

### ahrs:earth_to_body
argument|min|max
:---:|:---:|:---:
Vector3f||

### ahrs:get_vibration
argument|min|max
:---:|:---:|:---:

### ahrs:airspeed_estimate
argument|min|max
:---:|:---:|:---:

### ahrs:healthy
argument|min|max
:---:|:---:|:---:

### ahrs:home_is_set
argument|min|max
:---:|:---:|:---:

### ahrs:get_relative_position_NED_home
argument|min|max
:---:|:---:|:---:

### ahrs:get_velocity_NED
argument|min|max
:---:|:---:|:---:

### ahrs:groundspeed_vector
argument|min|max
:---:|:---:|:---:

### ahrs:wind_estimate
argument|min|max
:---:|:---:|:---:

### ahrs:get_hagl
argument|min|max
:---:|:---:|:---:

### ahrs:get_accel
argument|min|max
:---:|:---:|:---:

### ahrs:get_gyro
argument|min|max
:---:|:---:|:---:

### ahrs:get_home
argument|min|max
:---:|:---:|:---:

### ahrs:get_position
argument|min|max
:---:|:---:|:---:

### ahrs:get_yaw
argument|min|max
:---:|:---:|:---:

### ahrs:get_pitch
argument|min|max
:---:|:---:|:---:

### ahrs:get_roll
argument|min|max
:---:|:---:|:---:


## AP_HAL::UARTDriver

### AP_HAL::UARTDriver:set_flow_control
argument|min|max
:---:|:---:|:---:
enum||

### AP_HAL::UARTDriver:available
argument|min|max
:---:|:---:|:---:

### AP_HAL::UARTDriver:write
argument|min|max
:---:|:---:|:---:
uint8_t|MAX(0,0)|MIN(UINT8_MAX,UINT8_MAX)

### AP_HAL::UARTDriver:read
argument|min|max
:---:|:---:|:---:

### AP_HAL::UARTDriver:begin
argument|min|max
:---:|:---:|:---:
uint32_t|MAX(1U,0U)|MIN(UINT32_MAX,UINT32_MAX)


## RC_Channel

### RC_Channel:norm_input_ignore_trim
argument|min|max
:---:|:---:|:---:

### RC_Channel:get_aux_switch_pos
argument|min|max
:---:|:---:|:---:

### RC_Channel:norm_input
argument|min|max
:---:|:---:|:---:


