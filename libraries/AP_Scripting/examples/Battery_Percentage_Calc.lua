
local battery_instance = 0
local percentage_filt = nil
local exp_filter = 0.99

function update()

    if not battery:healthy(battery_instance) then
        -- the battery monitor is not healthy, don't change anything
        return update, 1000
    end

    -- to use raw voltage
    local voltage = battery:voltage(battery_instance)

    -- to use compensated voltage
    -- local voltage = battery:voltage_resting_estimate(battery_instance)

    -- Calculate the remaining percentage from voltage with the given polynomial
    -- need to use the correct equation here, this is just a place holder
    local percentage = (voltage / 13) * 100

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
    gcs:send_text(0, string.format("Battery Set to: %.1f",percentage_filt))

    -- run at 1hz
    return update, 1000

end

-- give battery monitor 10 seconds to come good on boot
return update, 1000