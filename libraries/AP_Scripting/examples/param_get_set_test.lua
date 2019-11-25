-- This script is a test of param set and get
local count = 0

function update() -- this is the loop which periodically runs

  -- get and print all the scripting paramerters
  local value = param:get_by_name('SCR_ENABLE')
  if value then
    gcs:send_text(0, string.format('LUA: SCR_ENABLE: %i',value))
  else
    gcs:send_text(0, 'LUA: get SCR_ENABLE failed')
  end
  value = param:get_by_name('SCR_VM_I_COUNT')
  if value then
    gcs:send_text(0, string.format('LUA: SCR_VM_I_COUNT: %i',value))
  else
    gcs:send_text(0, 'LUA: get SCR_VM_I_COUNT failed')
  end
  value = param:get_by_name('SCR_HEAP_SIZE')
  if value then
    gcs:send_text(0, string.format('LUA: SCR_HEAP_SIZE: %i',value))
  else
    gcs:send_text(0, 'LUA: get SCR_HEAP_SIZE failed')
  end
  value = param:get_by_name('SCR_DEBUG_LVL')
  if value then
    gcs:send_text(0, string.format('LUA: SCR_DEBUG_LVL: %i',value))
  else
    gcs:send_text(0, 'LUA: get SCR_DEBUG_LVL failed')
  end

  -- increment the script heap size by one
  local heap_size = param:get_by_name('SCR_HEAP_SIZE')
  if heap_size then
    if not(param:set_by_name('SCR_HEAP_SIZE',heap_size + 1)) then
      gcs:send_text(0, 'LUA: param set failed')
    end
  else
    gcs:send_text(0, 'LUA: get SCR_HEAP_SIZE failed')
  end


  count = count + 1;

  -- self terminate after 30 loops
  if count > 30 then
    gcs:send_text(0, 'LUA: goodby, world')
    param:set_by_name('SCR_ENABLE',0)
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
