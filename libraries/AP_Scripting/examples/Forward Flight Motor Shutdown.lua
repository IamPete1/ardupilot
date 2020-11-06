-- This is a script that stops motors in forward flight, for use with a copter tail sitter

-- this is the motors to stop
--local stop_motors = {1,4,5,8}
local stop_motors = {1,4}

-- motors should be shutdown if throttle goes lower than this value (%)
local throttle_off_threshold = 50

-- motors should be re-enabled if throttle higher lower than this value (%)
local throttle_on_threshold = 75

local switch = assert(rc:find_channel_for_option(300),"Lua: Could not find switch")

-- read spin min param, we set motors to this PWM to stop them
local pwm_min = assert(param:get("Q_M_PWM_MIN"),"Lua: Could not read Q_M_PWM_MIN")

-- find any motors enabled
local motor_chan = {}
for i = 1, #stop_motors do
  -- convert motor number to servo function
  if stop_motors[i] >= 1 and stop_motors[i] <= 8 then
    motor_chan[i] = SRV_Channels:find_channel(stop_motors[i]+32)

  elseif stop_motors[i] >= 82 and stop_motors[i] <= 85 then
    motor_chan[i] = SRV_Channels:find_channel(stop_motors[i]+73)
  end

  if motor_chan[i] == nil then
    error(string.format("Lua: Could not find motor %i",stop_motors[i]))
  end

end

-- keep track of last time in a VTOL mode, this allows to delay switching after a transition
local last_vtol_mode = 0

-- current action
local script_enabled = false
local motors_disabled = false


function update() -- this is the loop which periodically runs

  if quadplane:in_vtol_mode() then
    -- in a VTOL mode, nothing to do
    last_vtol_mode = millis()
    if script_enabled then
      gcs:send_text(0, "Lua: Forward flight motor shutdown disabled")
    end
    script_enabled = false
    motors_disabled = false
    return update, 1000 -- reschedule at 1hz
  end

  -- in forward flight check switch
  if switch:get_aux_switch_pos() ~= 2 then
    -- switch not high
    if script_enabled then
      gcs:send_text(0, "Lua: Forward flight motor shutdown disabled")
    end
    script_enabled = false
    motors_disabled = false
    return update, 100 -- reschedule at 10hz
  end

  -- in forward flight and enabled with switch, if armed then check that we are at least 10s past transition
  if arming:is_armed() and ((millis() - last_vtol_mode) < 10000) then
    -- armed and have not been in a VTOL mode for longer than 10s
    script_enabled = false
    motors_disabled = false
    return update, 100 -- reschedule at 10hz
  end

  -- print a message updating the state of the script
  if not script_enabled then
    gcs:send_text(0, "Lua: Forward flight motor shutdown enabled")
  end
  script_enabled = true

  -- check throttle level to see if motors should be disabled or enabled
  local throttle = SRV_Channels:get_output_scaled(70)

  if motors_disabled and (throttle > throttle_on_threshold) then
    motors_disabled = false
    gcs:send_text(0, "Lua: Throttle high motors enabled")

  elseif (not motors_disabled) and (throttle < throttle_off_threshold) then
    motors_disabled = true
    gcs:send_text(0, "Lua: Throttle low motors disabled")
  end

  if motors_disabled then
    for i = 1, #motor_chan do
      -- override for 15ms, should be called every 10ms when active
      -- using timeout means if the script dies the timeout will expire and all motors will come back
      -- we cant leave the vehicle in a un-flyable state
      SRV_Channels:set_output_pwm_chan_timeout(motor_chan[i],pwm_min,15)
    end
  end

  return update, 10 -- reschedule at 100hz
end

return update() -- run immediately before starting to reschedule
