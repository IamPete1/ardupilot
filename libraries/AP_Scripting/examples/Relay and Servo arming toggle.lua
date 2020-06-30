-- This script setting a servo and a relay pin active when armed

function update() -- this is the loop which periodically runs
  if arming:is_armed() then
    relay:on(0)
    SRV_Channels:set_output_pwm(94,1900) -- Scripting output 1
  else
    relay:off(0)
    SRV_Channels:set_output_pwm(94,1100) -- Scripting output 1
  end

  return update, 100 -- 10hz
end

return update() -- run immediately before starting to reschedule
