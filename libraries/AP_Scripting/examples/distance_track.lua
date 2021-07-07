-- this script tracks the distance traveled as reported by wheel encoders
local reverse_dist = 0
local total_dist = 0
local origin = 0
local reversed = vehicle:get_reversed()

-- switch to zero distance measurement, RC option 300
local switch = rc:find_channel_for_option(300)

function update()

  -- take the average of left and right encoder to remove effect of turns
  local average_dist = (wheelencoder:get_distance(0) + wheelencoder:get_distance(1)) * 0.5

  -- calculate the distance traveled since last reverse
  local traveled_dist = average_dist - reverse_dist

  -- remove effect of reversing
  local return_dist
  if not reversed then
    return_dist = total_dist + traveled_dist
  else
    return_dist = total_dist - traveled_dist
  end

  -- check for reset-switch
  if switch then
    local sw_pos = switch:get_aux_switch_pos()
    if sw_pos == 2 and last_sw_pos ~= 2 then
      origin = return_dist
    end
    last_sw_pos = sw_pos
  end

  gcs:send_named_float('WheelDist',return_dist - origin)

  local new_reversed = vehicle:get_reversed()
  if not (new_reversed == reversed) then
    -- if change in reversing apply current value to total and reset
    if not reversed then
      total_dist = total_dist + traveled_dist
    else 
      total_dist = total_dist - traveled_dist
    end
    reverse_dist = average_dist
    reversed = new_reversed
  end

  return update, 1000
end

return update()
