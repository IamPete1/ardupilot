-- Control a ODrive based actuator and home with a analog pot feedback

local STATE = {
   UNDEFINED = 0,
   IDLE = 1,
   ENCODER_INDEX_SEARCH = 6,
   CLOSED_LOOP_CONTROL = 8,
}

local CMD = {
   HEARTBEAT = 0x1,
   RXSDO = 0x04,
   SET_AXIS_STATE = 0x7,
   GET_ENCODER_ESTIMATES = 0x9,
   SET_CONTROLLER_MODE = 0x0b,
   SET_INPUT_POS = 0x0C,
   SET_INPUT_TORQUE = 0x0e,
   GET_TEMPERATURE = 0x15,
   GET_BUS_VOLTAGE_CURRENT = 0x17,
}

local LOCAL_STATE = {
   SETTINGS_REQUIRED = 0,
   NOT_CALIBRATED = 1,
   MOVE_FOIL_BACK = 2,
   RETURN_TO_CENTER = 3,
   MOVE_FOIL_FORWARD = 4,
   DISARMED = 5,
   ARMED = 6,
}

local CONTROL_MODE = {
   TORQUE_CONTROL = 1,
   POSITION_CONTROL = 3,
}

-- init to not calibrated state
local state = LOCAL_STATE.SETTINGS_REQUIRED

local odrive_status = {
   axis_errors = uint32_t(0),
   axis_state = STATE.UNDEFINED,
}

local target_node_id = 10

local last_heartbeat_ms = uint32_t(0)
local HEARTBEAT_TIMEOUT = uint32_t(2000)
local position_est = nil
local velocity_est = 0
local havePositionEst = false

-- ODrive settings as found from: https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/firmware/P5x-2epyHO8DXkyYEYQCpBsdw9skZ1GP04WKg4RVIjo/flat_endpoints.json
local axis0 = {}
axis0.pos_estimate = {
   id = 225,
   type = "f"
}
axis0.config = {}
axis0.config.can = {}
axis0.config.can.encoder_msg_rate_ms = {
    id = 275,
    type = "I" -- uint32
}
axis0.config.can.temperature_msg_rate_ms = {
    id = 278,
    type = "I" -- uint32
}
axis0.config.can.bus_voltage_msg_rate_ms = {
    id = 279,
    type = "I", -- uint32
}

axis0.controller = {}
axis0.controller.config = {}
axis0.controller.config.vel_limit = {
    id = 384,
    type = "f" -- float
}

local PARAM_TABLE_KEY = 2
local PARAM_TABLE_PREFIX = "OD_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local POS_MAX = bind_add_param('POS_MAX', 1, 10) -- Max endpoint position, turns from centre
local POS_MIN = bind_add_param('POS_MIN', 2, -10) -- Min endpoint position, turns from centre
local DEBUG = bind_add_param('DEBUG', 5, 0.0) -- Debug enable/disable
local MAX_LIMIT = bind_add_param('MAX_LIMIT', 6,  13.15) -- Maximum operational limit in turns from center, will move to this at 2000PWM
local MIN_LIMIT = bind_add_param('MIN_LIMIT', 7, -13.15) -- Minimum operational limit in turns from center, will move to this at 1000PWM

-- Load CAN driver. The first will attach to a protocol of 10
local driver = assert(CAN:get_device(20), "No scripting CAN interfaces found")

local escId = 0
if periph ~= nil then
   escId = assert(param:get("CAN_NODE")) - 1
end

-- Node ID and command ID are packed into the CAN frame ID
local NODE_ID_SHIFT = 5

-- Helper to pack 11-bit ID format used by ODrive
local function get_id(cmd)
   return (target_node_id << NODE_ID_SHIFT) | cmd
end

-- Only accept data from the target node id
driver:add_filter(uint32_t(0x3F) << NODE_ID_SHIFT, uint32_t(target_node_id) << NODE_ID_SHIFT)

-- Helper to parse data from can frames
local function unpack_data(frame, start_bit, end_bit, format_str)
   local packed_str = ""
   for i = start_bit, end_bit do
      packed_str = packed_str .. string.char(frame:data(i))
   end
   return string.unpack("<" .. format_str, packed_str) -- Always little-endian for ODrive
end

-- Parse heartbeat from ODrive and store state
local function update_heartbeat(frame)

   odrive_status.axis_errors = uint32_t(frame:data(0) | (frame:data(1) << 8) | (frame:data(2) << 16) | (frame:data(3) << 24))
   odrive_status.axis_state = frame:data(4)

   -- We have a valid heartbeat, update timer and state
   last_heartbeat_ms = millis()
end

-- parse data from CMD.GET_BUS_VOLTAGE_CURRENT and stuff in ESC telem
local esc_telem_data = ESCTelemetryData()
local servo_telem_data = ServoTelemetryData()
local function update_volt_curr_telem(frame)
   local bus_voltage = unpack_data(frame, 0, 3, "f") -- float
   local bus_current = unpack_data(frame, 4, 7, "f") -- float

   if (not bus_voltage) or (not bus_current) then
      return
   end

   -- update esc telem data
   esc_telem_data:voltage(bus_voltage)
   esc_telem_data:current(bus_current)
end

-- parse data from CMD.GET_TEMPERATURE and stuff in esc telem
local function update_temp_telem(frame)
   local fet_temp = unpack_data(frame, 0, 3, "f") -- float

   if fet_temp ~= fet_temp then
      return -- nan
   end

   -- convert to cdeg
   fet_temp = math.floor(fet_temp * 100)

   -- update esc telem data
   esc_telem_data:temperature_cdeg(fet_temp)
end

-- update the reported position from the odrive
local function update_position_est(frame)
   local pos = unpack_data(frame, 0, 3, "f") -- float
   if pos == pos then
      -- not nan
      position_est = pos

      -- convert turns to degrees
      servo_telem_data:measured_position(pos * 360.0)

   else
      position_est = nil
   end
   havePositionEst = true

   velocity_est = unpack_data(frame, 4, 7, "f") -- float

   -- convert turns to degrees
   servo_telem_data:speed(velocity_est * 360.0)
end

-- Read data from can buffer
local function read_data()

   for _ = 1, 40 do
      local frame = driver:read_frame()

      if not frame then
         return
      end

      local cmd_id = (frame:id() & 0x1F):toint()

      if (cmd_id == CMD.HEARTBEAT) then
         update_heartbeat(frame)
      elseif (cmd_id == CMD.GET_BUS_VOLTAGE_CURRENT) then
         update_volt_curr_telem(frame)
      elseif (cmd_id == CMD.GET_TEMPERATURE) then
         update_temp_telem(frame)
      elseif (cmd_id == CMD.GET_ENCODER_ESTIMATES) then
         update_position_est(frame)
      end
   end

end

-- Set control mode on odrive. This is needed before we can drive the motor.
local state_msg = CANFrame()
state_msg:id(get_id(CMD.SET_AXIS_STATE))
state_msg:dlc(4)
local function set_odrive_state(tagetState)
   state_msg:data(0, tagetState)
   driver:write_frame(state_msg, 500)
end

local function LERP(x, xLow, xHigh, yLow, yHigh)
   local ratio = (x - xLow) / (xHigh - xLow)
   return yLow + ratio * (yHigh - yLow)
end

local function constrainedLERP(x, xLow, xHigh, yLow, yHigh)
   if x < xLow then
      return yLow
   end
   if x > xHigh then
      return yHigh
   end
   return LERP(x, xLow, xHigh, yLow, yHigh)
end

-- send position input command to odrive
local msg_input_pos = CANFrame()
msg_input_pos:id(get_id(CMD.SET_INPUT_POS))
msg_input_pos:dlc(8)
local function send_position_command(des_pos)
   -- convert turns to degrees
   local des_pos_deg = des_pos * 360.0
   servo_telem_data:command_position(des_pos_deg)

   -- populate force with desired position, command_position is not reported over DroneCAN
   servo_telem_data:force(des_pos_deg)

   local payload = string.pack("<f", des_pos)
   for i = 1, #payload do
      msg_input_pos:data(i - 1, string.byte(payload, i))
   end
   driver:write_frame(msg_input_pos, 500)
end

-- send torque command to odrive
local msg_input_torque = CANFrame()
msg_input_torque:id(get_id(CMD.SET_INPUT_TORQUE))
msg_input_torque:dlc(4)
local function send_torque_command(torque)
   local payload = string.pack("<f", torque)
   for i = 1, #payload do
      msg_input_torque:data(i - 1, string.byte(payload, i))
   end
   driver:write_frame(msg_input_torque, 500)
end

-- Set control mode command to odrive
local msg_controller_mode = CANFrame()
msg_controller_mode:id(get_id(CMD.SET_CONTROLLER_MODE))
msg_controller_mode:dlc(8)
local function send_set_control_mode(control_mode)
   -- input mode is always passthrough (1)
   local payload = string.pack("<LL", control_mode, 1)
   for i = 1, #payload do
      msg_controller_mode:data(i - 1, string.byte(payload, i))
   end
   driver:write_frame(msg_controller_mode, 500)
end

-- Read/Write an endpoint value
local function send_write_RxSdo(endpoint, value)
   local msg = CANFrame()

   msg:id(get_id(CMD.RXSDO))

   -- pack payload
   local format = "<BHB" .. endpoint.type
   local payload = string.pack(format, 0x01, endpoint.id, 0, value)
   for i = 1, #payload do
      msg:data(i - 1, string.byte(payload, i))
   end
   msg:dlc(#payload)
   driver:write_frame(msg, 1000)
end

-- Return true if the vehicle is armed
local armed_mask = uint64_t(0, 2)
local function is_armed()
   if not periph then
      return arming:is_armed()
   end
   return (periph:get_vehicle_state() & armed_mask):toint() ~= 0
end

-- Send all required settings to odrive when we first start talking to it
-- returns true when all setup has complete
local homing_start_ms = uint32_t(0)
local homing_min = 0
local homing_max = 0
local last_moving_ms = uint32_t(0)
local function run_setup()

   local now_ms = millis()
   local homing_torque = 0.1
   local homing_vel_limit = 2.0
   local normal_vel_limit = 150

   -- Finish homing when velocity is less than 5 deg/sec for half a second
   if math.abs(velocity_est) > (5 / 360.0) then
      last_moving_ms = now_ms
   end
   local stopped = (now_ms - last_moving_ms) > uint32_t(500)

   -- set message rates for cyclic telem
   if state == LOCAL_STATE.SETTINGS_REQUIRED then
      -- Setup message streaming
      send_write_RxSdo(axis0.config.can.bus_voltage_msg_rate_ms, 10)
      send_write_RxSdo(axis0.config.can.temperature_msg_rate_ms, 10)
      send_write_RxSdo(axis0.config.can.encoder_msg_rate_ms, 10)

      -- config applied, advance state
      state = LOCAL_STATE.NOT_CALIBRATED

   elseif state == LOCAL_STATE.NOT_CALIBRATED then
      -- wait until safety switch is disabled before trying to home
      if not SRV_Channels:get_safety_state() and not is_armed() then
         state = LOCAL_STATE.MOVE_FOIL_BACK
         send_write_RxSdo(axis0.controller.config.vel_limit, homing_vel_limit)
         send_set_control_mode(CONTROL_MODE.TORQUE_CONTROL)
         set_odrive_state(STATE.CLOSED_LOOP_CONTROL)
         last_moving_ms = now_ms
         homing_start_ms = now_ms
      end

   elseif state == LOCAL_STATE.MOVE_FOIL_BACK then
      -- Send constant torque
      send_torque_command(homing_torque)

      homing_max = math.max(homing_max, position_est)
      homing_min = math.min(homing_min, position_est)

      -- Move on once stopped
      if stopped then
         state = LOCAL_STATE.RETURN_TO_CENTER
         homing_start_ms = now_ms
         set_odrive_state(STATE.IDLE)
         send_write_RxSdo(axis0.controller.config.vel_limit, normal_vel_limit)
         send_set_control_mode(CONTROL_MODE.POSITION_CONTROL)
         set_odrive_state(STATE.CLOSED_LOOP_CONTROL)
      end

   elseif state == LOCAL_STATE.RETURN_TO_CENTER then
      -- Return to zero
      send_position_command(0)

      -- Wait for sometime to allow the actuator to move
      if (now_ms - homing_start_ms) > 2000 then
         state = LOCAL_STATE.MOVE_FOIL_FORWARD
         last_moving_ms = now_ms
         homing_start_ms = now_ms

         -- Torque control again
         set_odrive_state(STATE.IDLE)
         send_write_RxSdo(axis0.controller.config.vel_limit, homing_vel_limit)
         send_set_control_mode(CONTROL_MODE.TORQUE_CONTROL)
         set_odrive_state(STATE.CLOSED_LOOP_CONTROL)
      end

   elseif state == LOCAL_STATE.MOVE_FOIL_FORWARD then
      -- Send constant torque
      send_torque_command(-homing_torque)

      homing_max = math.max(homing_max, position_est)
      homing_min = math.min(homing_min, position_est)

      -- Move on once stopped
      if stopped then
         state = LOCAL_STATE.DISARMED
         -- Return to position control and idle state
         set_odrive_state(STATE.IDLE)
         send_set_control_mode(CONTROL_MODE.POSITION_CONTROL)
         send_write_RxSdo(axis0.controller.config.vel_limit, normal_vel_limit)

         -- Wrap the position estimate down to +-0.5
         local subTurn = math.fmod(position_est, 1.0)
         if subTurn < -0.5 then
            subTurn = subTurn + 1.0
         end

         print(string.format("Homed min: %f max: %f subturn: %f", homing_min, homing_max, subTurn))

--       send_write_RxSdo(axis0.pos_estimate, turn + subTurn)
      end
   end
end

local lastDebugSend = uint32_t(0)
local function update()

   local now = millis()

   -- Send back debug data if running on a vehicle
   if DEBUG:get() > 0 then
      -- Debug on periph at 1Hz
      if ((now - lastDebugSend) > 1000) then
         lastDebugSend = now

         local reportedPos = position_est
         if reportedPos == nil then
            reportedPos = 0/0
         end

         print(string.format("state: %i, pos: %0.4f",
            state,
            reportedPos))
      end
   end

   -- read data sent from the ODrive
   read_data()

   -- update timeout on heartbeat state
   if ((now - last_heartbeat_ms) > HEARTBEAT_TIMEOUT) or (last_heartbeat_ms == 0) then
      -- we are not speaking to the odrive, no point in continuing
      return update, 10
   end

   -- Update telem data
   esc_telem:update_telem_data(escId, esc_telem_data, 0x0D) -- voltage, current, temperature
   esc_telem:update_rpm(escId, odrive_status.axis_errors:tofloat(), state)
   servo_telem:update_telem_data(escId, servo_telem_data)

   -- Errors reset state and wait for them to clear
   if odrive_status.axis_errors:toint() ~= 0 then
      state = LOCAL_STATE.SETTINGS_REQUIRED
      return update, 10
   end

   -- Wait until any start up or calibration is done
   if (odrive_status.axis_state ~= STATE.IDLE) and (odrive_status.axis_state ~= STATE.CLOSED_LOOP_CONTROL) then
      return update, 10
   end

   if state < LOCAL_STATE.DISARMED then
      -- Must be getting position data for calibration
      if havePositionEst then
         run_setup()
      end
      return update, 10
   end

   -- Allow closed loop control if not safe
   local allowClosedLoop = SRV_Channels:get_safety_state() == false

   -- If disarmed then arm on safety state change
   if state == LOCAL_STATE.DISARMED then
      if allowClosedLoop then
         set_odrive_state(STATE.CLOSED_LOOP_CONTROL)
         state = LOCAL_STATE.ARMED
      end
      return update, 10
   end

   if state == LOCAL_STATE.ARMED then
      -- Return to idle if safety is disabled
      if not allowClosedLoop then
         set_odrive_state(STATE.IDLE)
         state = LOCAL_STATE.DISARMED
         return update, 10
      end

      -- Make sure the ODrive state is correct
      if odrive_status.axis_state == STATE.CLOSED_LOOP_CONTROL then
         -- Send position commands
         local PWMCmd = SRV_Channels:get_output_pwm_chan(0)
         if PWMCmd ~= 0 then
            send_position_command(constrainedLERP(PWMCmd, 1000, 2000, MIN_LIMIT:get(), MAX_LIMIT:get()))
         end
      end
   end

   return update, 10

end

return update()
