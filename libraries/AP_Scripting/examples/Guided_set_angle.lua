local target_angle_1 = Vector3f()
local target_angle_2 = Vector3f()

target_angle_1:z(math.rad(45))
target_angle_2:z(math.rad(-45))

local flipflop = true
function update()
  if flipflop then
    vehicle:set_target_angle(target_angle_1)
  else 
    vehicle:set_target_angle(target_angle_2)
  end
  flipflop = not flipflop
  return update, 10000
end

return update()
