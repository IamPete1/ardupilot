-- Return true if the vehicle is armed
local armed_mask = uint64_t(0, 2)
local function is_armed()
    if not periph then
       return arming:is_armed()
    end
    return (periph:get_vehicle_state() & armed_mask):toint() ~= 0
 end

local function update()

    print(string.format("Armed %s, Safe: %s", tostring(is_armed()), tostring(SRV_Channels:get_safety_state())))

    return update, 500
end

return update()
