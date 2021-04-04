
local battery_instance = 0
local percentage_filt = nil
local exp_filter = 0.99

-- Lookup table values to calculate battery percentage from voltage
local interp_voltage = {19.317000, 19.870091, 20.312182, 20.659182, 20.947545, 21.186273, 21.385545, 21.552818, 21.692909, 21.811545, 21.892273, 21.949000, 21.996091, 22.036000, 22.079000, 22.119000, 22.154727, 22.184545, 22.211091, 22.238636, 22.266818, 22.289455, 22.311000, 22.330909, 22.347045, 22.361364, 22.375000, 22.388409, 22.403727, 22.418636, 22.429000, 22.441818, 22.453273, 22.458000, 22.469000, 22.477000, 22.482318, 22.491000, 22.496273, 22.506727, 22.515818, 22.525364, 22.538000, 22.551455, 22.569000, 22.584273, 22.600182, 22.620636, 22.640182, 22.660091, 22.680273, 22.703182, 22.726000, 22.753364, 22.777000, 22.805000, 22.833455, 22.864909, 22.894636, 22.926409, 22.962455, 23.003000, 23.042000, 23.083727, 23.127000, 23.167455, 23.212000, 23.260000, 23.303818, 23.349364, 23.393182, 23.439364, 23.482864, 23.526182, 23.567818, 23.601000, 23.633818, 23.668000, 23.703000, 23.741045, 23.788364, 23.843000, 23.907182, 23.985273, 24.074545, 24.172455, 24.254182, 24.307455, 24.346000, 24.379091, 24.426000, 24.478909, 24.537000, 24.594000, 24.655136, 24.716000, 24.779364, 24.845818, 24.918000, 25.122000}
local interp_pct = {4.375000, 5.340909, 6.306818, 7.272727, 8.238636, 9.204545, 10.170455, 11.136364, 12.102273, 13.068182, 14.034091, 15.000000, 15.965909, 16.931818, 17.897727, 18.863636, 19.829545, 20.795455, 21.761364, 22.727273, 23.693182, 24.659091, 25.625000, 26.590909, 27.556818, 28.522727, 29.488636, 30.454545, 31.420455, 32.386364, 33.352273, 34.318182, 35.284091, 36.250000, 37.215909, 38.181818, 39.147727, 40.113636, 41.079545, 42.045455, 43.011364, 43.977273, 44.943182, 45.909091, 46.875000, 47.840909, 48.806818, 49.772727, 50.738636, 51.704545, 52.670455, 53.636364, 54.602273, 55.568182, 56.534091, 57.500000, 58.465909, 59.431818, 60.397727, 61.363636, 62.329545, 63.295455, 64.261364, 65.227273, 66.193182, 67.159091, 68.125000, 69.090909, 70.056818, 71.022727, 71.988636, 72.954545, 73.920455, 74.886364, 75.852273, 76.818182, 77.784091, 78.750000, 79.715909, 80.681818, 81.647727, 82.613636, 83.579545, 84.545455, 85.511364, 86.477273, 87.443182, 88.409091, 89.375000, 90.340909, 91.306818, 92.272727, 93.238636, 94.204545, 95.170455, 96.136364, 97.102273, 98.068182, 99.034091, 100.000000}


function linear_interpolate(low_output, high_output, var_value, var_low, var_high)
    local p = (var_value - var_low) / (var_high - var_low)
    return low_output + p * (high_output - low_output)
end


function interp_table(voltage)
    -- find the correct point in the table

    -- check extremes
    if voltage <= interp_voltage[1] then
        -- voltage lower than min value in table
        return interp_pct[1]
    end
    if voltage >= interp_voltage[#interp_voltage] then
        -- voltage hight than max value
        return interp_pct[#interp_voltage]
    end

    for i = 1, #interp_voltage-1 do
        if voltage >= interp_voltage[i] and voltage < interp_voltage[i+1] then
            return linear_interpolate(interp_pct[i], interp_pct[i+1], voltage, interp_voltage[i], interp_voltage[i+1])
        end
    end
end

-- test interpolation table
--[[
local file = io.open('table_test.csv', 'w')
for i = 28, 18, -0.01 do
    file:write(string.format('%0.2f, %0.2f\n',i,interp_table(i)))
end
file:close()
--]]


function update()

    if not battery:healthy(battery_instance) then
        -- the battery monitor is not healthy, don't change anything
        return update, 1000
    end

    -- to use raw voltage
    local voltage = battery:voltage(battery_instance)

    -- to use compensated voltage
    -- local voltage = battery:voltage_resting_estimate(battery_instance)

    -- Calculate the remaining percentage from voltage from the interpolation table
    local percentage = interp_table(voltage)

    -- constrain 0 to 100
    percentage = math.max(math.min(percentage,100),0)

    if percentage_filt then
        -- filter new reading
        percentage_filt = (percentage_filt * exp_filter) + (percentage * (1 - exp_filter))
    else
        -- init, set filter to latest reading
        percentage_filt = percentage
    end

    -- update the battery percentage
    -- note that current draw will still decrease the remaining percentage until we update it again
    battery:reset_remaining(battery_instance, percentage_filt)

    -- debug print
    -- gcs:send_text(0, string.format("Battery Set to: %.1f",percentage_filt))

    -- run at 1hz
    return update, 1000

end

-- give battery monitor 10 seconds to come good on boot
return update, 1000
