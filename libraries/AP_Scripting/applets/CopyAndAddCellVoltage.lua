--[[
   copy the readings of one battery monitor onto a scripting battery monitor

   This allows the scripting monitor to report the same voltage, current and
   temperature as the source monitor, with extra data added by this script
--]]

local src = 1 -- BATT2, monitor to copy from
local dst = 0 -- BATT, must be set to scripting

local MID_PIN = 10 -- PC0, hwdef SCALE is applied by voltage_average

local mid_pin = assert(analog:channel(), "BattCopy: no analog channel")
assert(mid_pin:set_pin(MID_PIN), "BattCopy: invalid mid cell pin")

local state = BattMonitorScript_State()
state:cell_count(2)

local function update()
   -- Copy over state

   state:healthy(battery:healthy(src))
   local voltage = battery:voltage(src)
   state:voltage(voltage)

   local current = battery:current_amps(src)
   if current then
      state:current_amps(current)
   end

   local consumed_mah = battery:consumed_mah(src)
   if consumed_mah then
      state:consumed_mah(consumed_mah)
   end

   local consumed_wh = battery:consumed_wh(src)
   if consumed_wh then
      state:consumed_wh(consumed_wh)
   end

   -- Set cell voltages, both cells are needed as the GCS spreads any
   -- difference between the pack voltage and the sum of cells over the cells
   local mid = mid_pin:voltage_average()
   state:cell_voltages(0, math.floor(mid * 1000))
   state:cell_voltages(1, math.floor(math.max(voltage - mid, 0) * 1000))

   battery:handle_scripting(dst, state)

   return update, 100
end

return update()
