
-- Filter objects
local filter = {}

-- Return a low pass filter object using the supplied cutoff frequency parameter and a fixed sample rate
---@param cutoff_freq_param Parameter_ud_const|Parameter_ud
---@return table -- low pass filter object
function filter.get_low_pass(cutoff_freq_param)
    local self = {
        value = nil,
    }

    --- Update filter
    ---@param new_value number -- Latest reading to add to filter
    ---@param dt number -- time since last call in seconds
    ---@return number|nil -- filtered output
    function self.update(new_value, dt)
        if new_value == nil then
            -- reset
            self.value = nil
            return
        end

        local cutoff_freq = cutoff_freq_param:get()
        if (self.value == nil) or (cutoff_freq <= 0) then
            -- Have been reset or disabled
            self.value = new_value

        else
            -- Apply filter
            local rc = 1.0 / (math.pi * 2 * cutoff_freq)
            local alpha = dt / (dt + rc)
            self.value = self.value + (new_value - self.value) * alpha

        end

        return self.value
    end

    return self
end

-- Return a complementary filter object using the supplied cutoff frequency parameter
---@param cutoff_freq_param Parameter_ud_const|Parameter_ud
---@return table -- low pass filter object
function filter.get_complementary_filter(cutoff_freq_param)

    local self = {
        value = nil,
    }

    --- Update filter
    ---@param position_value number -- Latest position reading to apply
    ---@param rate_value number -- Latest rate reading to apply
    ---@param dt number -- time since last call in seconds
    ---@return number|nil -- filtered output position output
    function self.update(position_value, rate_value, dt)
        local cutoff_freq = cutoff_freq_param:get()

        if (self.value == nil) or (cutoff_freq <= 0) then
            -- Have been reset
            self.value = position_value
            return self.value
        end

        -- Calculate coefficients
        local rc = 1.0 / (math.pi * 2 * cutoff_freq)
        local alpha = math.min(dt / (dt + rc), 1.0)
        local beta = 1 - alpha

        -- Integrate rate value
        local integrated_position = self.value + rate_value * dt

        -- Apply filter
        self.value = (integrated_position * beta) + (position_value * alpha)

        return self.value
    end

    --- Reset filter
    function self.reset()
        self.value = nil
    end

    return self
end

return filter
