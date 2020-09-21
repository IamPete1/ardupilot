-- this is a script to log the necessary variables for calculation of throttle expo

-- global variables
local file_name -- current file name
local state -- currently logging, true or false
local over_ride_count -- pilot override counter
local switch = assert(rc:find_channel_for_option(300),"Expo: Could not find scripting RC switch 300") -- enable disable switch
local over_ride_print_count = 20

-- start an new csv file, without overwriting
local function start_new_file()

  -- search for a index without a file
  local index = 0
  while true do
    file_name = string.format("Expo vars %i.csv",index)
    local file = io.open(file_name)
    local first_line = file:read(1) -- try and read the first character
    io.close(file)
    if first_line == nil then
      break
    end
    index = index + 1
  end

  -- open file and make header
  file = assert(io.open(file_name, "w"), "Expo: Could not make file :" .. file_name)
  file:write("throttle, velocity (m/s), air density (kg/m^3), acceleration (m/s^2)\n")
  file:close()

end

 -- wait to be armed
function idle()
  state = false

  -- must be armed
  if arming:is_armed() and vehicle:get_likely_flying() then
    -- open new file and start logging
    start_new_file()
    over_ride_count = over_ride_print_count
    return update, 10
  end

  return idle, 100
end

function update() -- this is the loop records the data

  if not arming:is_armed() then
    -- disarmed - stop logging
    return idle, 100
  end

  if (switch:get_aux_switch_pos() ~= 2) or (vehicle:get_mode() ~= 0) then
    -- wait to be switched active and in stabilize
    if state ~= false then
      state = false
      gcs:send_text(4,"Expo: logging paused")
    end
    return update, 100
  end

  if state ~= true then
    state = true
    gcs:send_text(4,"Expo: logging started")
  end

  local targets = attitude_control:get_att_target_euler_cd()
  local throttle = motors:get_throttle()
  local hover_throttle = motors:get_throttle_hover()

  -- check for level attitude
  if (math.abs(targets:x()) > 1) or (math.abs(targets:y()) > 1) then
    over_ride_count = over_ride_count + 1
    if over_ride_count >= over_ride_print_count then
      gcs:send_text(4,"Expo: pilot override, roll and pitch not level")
      over_ride_count = 0
    end
    return update, 100
  end

  -- check for some throttle outside of hover range
  if math.abs(throttle - hover_throttle) < (hover_throttle * 0.1) then
    over_ride_count = over_ride_count + 1
    if over_ride_count >= over_ride_print_count then
      gcs:send_text(4,"Expo: pilot override, throttle too close to hover")
      over_ride_count = 0
    end
    return update, 100
  end

  -- resting to threshold means we get a instant message and slower repeats
  over_ride_count = over_ride_print_count

  local velocity = ahrs:earth_to_body(ahrs:get_velocity_NED())
  local acceleration = ahrs:get_accel()

  -- air density
  local press = baro:get_pressure()
  local temp = baro:get_external_temperature() + 273.2 -- convert deg centigrade to kelvin
  local density =  press / (temp * 287.058) -- calculate the air density, ideal gas law, constant is (R) specific gas constant for air


  -- write to csv
  file = io.open(file_name, "a")
  file:write(string.format("%0.4f, %0.4f, %0.4f, %0.4f\n",throttle,velocity:z()*-1,density,acceleration:z()))
  file:close()

  return update, 10 -- reschedule the loop (100hz)
end

return idle() -- run immediately before starting to reschedule
