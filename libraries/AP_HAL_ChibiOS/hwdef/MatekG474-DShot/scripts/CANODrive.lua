-- Control a ODrive based actuator with and home with a analog pot feedback

local STATE = {
   UNDEFINED = 0,
   IDLE = 1,
   STARTUP_SEQUENCE = 2,
   FULL_CALIBRATION_SEQUENCE = 3,
   MOTOR_CALIBRATION = 4,
   ENCODER_INDEX_SEARCH = 6,
   ENCODER_OFFSET_CALIBRATION = 7,
   CLOSED_LOOP_CONTROL = 8,
   LOCKIN_SPIN = 9,
   ENCODER_DIR_FIND = 10,
   HOMING = 11,
   ENCODER_HALL_POLARITY_CALIBRATION = 12,
   ENCODER_HALL_PHASE_CALIBRATION = 13,
   ANTICOGGING_CALIBRATION = 14,
   HARMONIC_CALIBRATION = 15,
   HARMONIC_CALIBRATION_COMMUTATION = 16,
}

local CMD = {
   HEARTBEAT = 0x1,
   RXSDO = 0x04,
   SET_AXIS_STATE = 0x7,
   GET_ENCODER_ESTIMATES = 0x9,
   SET_INPUT_POS = 0x0C,
   GET_TEMPERATURE = 0x15,
   GET_BUS_VOLTAGE_CURRENT = 0x17,
}

local LOCAL_STATE = {
   SETTINGS_REQUIRED = 0,
   NOT_CALIBRATED = 1,
   INDEX_FIND_SENT = 2,
   DISARMED = 3,
   ARMED = 4,
   ERROR = 10,
}
-- init to not calibrated state
local state = LOCAL_STATE.SETTINGS_REQUIRED

local odrive_status = {
   axis_errors = uint32_t(0),
   axis_state = STATE.UNDEFINED,
   procedure_result = 0,
   trajectory_done_flag = 0
}

local target_node_id = 10

local last_heartbeat_ms = uint32_t(0)
local HEARTBEAT_TIMEOUT = uint32_t(2000)
local position_est = nil
local havePositionEst = false

local OPCODE_WRITE = 0x01

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
axis0.controller.config.vel_ramp_rate = {
    id = 386,
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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 50), 'could not add param table')

local POS_MAX = bind_add_param('POS_MAX', 1, 10) -- Max endpoint position, turns from centre
local POS_MIN = bind_add_param('POS_MIN', 2, -10) -- Min endpoint position, turns from centre
local POT_MAX_VOLT = bind_add_param('POT_MAX_VOLT', 3, 3.0) -- Potentiometer voltage reading corresponding to max position endpoint position
local POT_MIN_VOLT = bind_add_param('POT_MIN_VOLT', 4, 0.3) -- Potentiometer voltage reading corresponding to min position endpoint position
local DEBUG = bind_add_param('DEBUG', 5, 0.0) -- Debug enable/disable

-- Load CAN driver. The first will attach to a protocol of 10
local driver = assert(CAN:get_device(20), "No scripting CAN interfaces found")

-- Get analog input
local potInput = assert(analog:channel(), "No ADC")
local potPin = 14 -- CubeOrange battery 1 voltage
if periph ~= nil then
   potPin = 12 -- MATEK G4 PWM 8
end
assert(potInput:set_pin(potPin), "No ADC pin")

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
   odrive_status.procedure_result = frame:data(5)
   odrive_status.trajectory_done_flag = frame:data(6)

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
      -- nan
      return
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
   else
      position_est = nil
   end
   havePositionEst = true

   local vel = unpack_data(frame, 4, 7, "f") -- float
   servo_telem_data:speed(vel)
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
   -- note, requested state is a uint32_t, i am just being lazy as currently will only need one byte
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

-- send position input commands to odrive
local msg_input_pos = CANFrame()
msg_input_pos:id(get_id(CMD.SET_INPUT_POS))
msg_input_pos:dlc(8)
local function send_position_command(des_pos)
   servo_telem_data:command_position(des_pos)

   local payload = string.pack("<f", des_pos)
   for i = 1, #payload do
      msg_input_pos:data(i - 1, string.byte(payload, i))
   end

   -- timeout of 500us
   driver:write_frame(msg_input_pos, 500)
end

-- Read/Write an endpoint value
local function send_RxSdo(opcode, endpoint, value)
   local msg = CANFrame()

   msg:id(get_id(CMD.RXSDO))

   -- pack payload
   local format = "<BHB" .. endpoint.type
   local payload = string.pack(format, opcode, endpoint.id, 0, value)
   for i = 1, #payload do
      msg:data(i - 1, string.byte(payload, i))
   end

   msg:dlc(#payload)

   -- timeout of 1000us
   driver:write_frame(msg, 1000)
end

-- Return the position as calculated by the potentiometer
local function potPos()
   local voltage = potInput:voltage_average_ratiometric()
   return LERP(voltage, POT_MIN_VOLT:get(), POT_MAX_VOLT:get(), POS_MIN:get(), POS_MAX:get())
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
local function run_setup()

   -- set message rates for cyclic telem
   if state == LOCAL_STATE.SETTINGS_REQUIRED then
      -- Setup message streaming
      send_RxSdo(OPCODE_WRITE, axis0.config.can.bus_voltage_msg_rate_ms, 500)
      send_RxSdo(OPCODE_WRITE, axis0.config.can.temperature_msg_rate_ms, 500)
      send_RxSdo(OPCODE_WRITE, axis0.config.can.encoder_msg_rate_ms, 250)

      -- set kinematic limits
      send_RxSdo(OPCODE_WRITE, axis0.controller.config.vel_ramp_rate, 10.0) -- rev/s/s

      -- config applied, advance state
      state = LOCAL_STATE.NOT_CALIBRATED

   elseif state == LOCAL_STATE.NOT_CALIBRATED then
      -- wait until safety switch is disabled before trying to do index search
      if not SRV_Channels:get_safety_state() and not is_armed() then
         set_odrive_state(STATE.ENCODER_INDEX_SEARCH)
         state = LOCAL_STATE.INDEX_FIND_SENT
      end

   elseif state == LOCAL_STATE.INDEX_FIND_SENT then
      -- Once index search is done the ODrive will return to idle, but will now have a position estimate
      if (position_est ~= nil) and (odrive_status.axis_state == STATE.IDLE) then
         -- Round the pos position to the nearest turn
         local turn = math.floor(potPos() + 0.5)

         -- Wrap the position estimate down to +-0.5
         local subTurn = math.fmod(position_est, 1.0)
         if subTurn < -0.5 then
            subTurn = subTurn + 1.0
         end

         send_RxSdo(OPCODE_WRITE, axis0.pos_estimate, turn + subTurn)
         state = LOCAL_STATE.DISARMED
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

         print(string.format("state: %i, potVolt: %0.4f, potPos: %0.4f, pos: %0.4f",
            state,
            potInput:voltage_average_ratiometric(),
            potPos(),
            reportedPos))
      end
   end

   -- read data sent from the ODrive
   read_data()

   -- Update telem data
   esc_telem:update_telem_data(escId, esc_telem_data, 0x0D) -- voltage, current, temperature
   esc_telem:update_rpm(escId, odrive_status.axis_errors:tofloat(), state)
   servo_telem:update_telem_data(escId, servo_telem_data)


   -- update timeout on heartbeat state
   if ((now - last_heartbeat_ms) > HEARTBEAT_TIMEOUT) or (last_heartbeat_ms == 0) then
      -- we are not speaking to the odrive, no point in continuing
      return update, 10
   end

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
            send_position_command(constrainedLERP(PWMCmd, 1000, 2000, -13.15, 13.15))
         end
      end
   end

   return update, 10

end

return update()
