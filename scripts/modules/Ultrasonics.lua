
local ultrasonics = {}

local filter = require("filter")

---Rangefinder object
---@param instance integer -- RangeFinder instance to use
---@param filt Parameter_ud_const|Parameter_ud -- Filter frequency parameter
---@return table
local function Rangefinder(instance, filt)

    assert(instance < rangefinder:num_sensors(), "invalid rangefinder: " .. tostring(instance))

    local self = {
        -- Rangefinder backend object
        rangefinder = nil,

        -- Used to check if the reading is fresh
        last_reading_ms = uint32_t(0),

        -- Current raw height and weighting
        height = 0,

        -- Output low pass filter
        filter = filter.get_low_pass(filt),

        -- Time of last valid reading
        last_valid_ms = uint32_t(0),
    }

    -- get position
    local prefix = "RNGFND" .. tostring(instance + 1) .. "_"
    local pos_x = param:get(prefix .. "POS_X")
    local pos_y = param:get(prefix .. "POS_Y")
    local pos_z = param:get(prefix .. "POS_Z")
    assert((pos_x ~= nil) and (pos_y ~= nil) and (pos_z ~= nil), "Could not get rangefinder pos: " .. tostring(instance))

    -- Rangefinder position
    local pos = Vector3f()
    pos:x(pos_x)
    pos:y(pos_y)
    pos:z(pos_z)

    -- Status enum
    local rangefinder_status = {
        good = 4
    }

    -- update function
    ---@param vertical_velocity number -- AHRS vertical velocity estimate
    ---@param attitude Quaternion_ud -- quaternion representing the vehicle attitude
    ---@param dt number -- time since last call in seconds
    function self.update(vertical_velocity, attitude, dt)

        -- Make sure we have a backend.
        if self.rangefinder == nil then
            self.rangefinder = rangefinder:get_backend(instance)
        end
        if self.rangefinder == nil then
            return
        end

        -- get state
        local state = self.rangefinder:get_state()

        local last_reading = state:last_reading()
        local new_height_measurement = false
        if (last_reading ~= self.last_reading_ms) and (state:status() == rangefinder_status.good) then
            self.last_reading_ms = last_reading

            -- have a new reading
            -- project point from rangefinder position
            local point = pos:copy()
            point:z(point:z() + state:distance())

            -- convert into earth fame
            -- Note that this function is named incorrectly, it should be `body_to_earth`
            -- See: https://github.com/ArduPilot/ardupilot/pull/28178
            attitude:earth_to_body(point)
            local ef_height = point:z()

            -- Update last valid for next climb rate check
            self.last_valid_ms = last_reading

            -- Extrapolate from reading time to current time
            local reading_dt = (millis() - last_reading):tofloat() * 0.001
            self.height = ef_height + vertical_velocity * math.min(reading_dt, 0.2)

            -- Measurement successful
            new_height_measurement = true
        end

        if not new_height_measurement then
            -- No new reading, extrapolate from last update
            self.height = self.height + vertical_velocity * dt
        end

        -- Update filter
        self.filter.update(self.height, dt)

        -- Log
        logger:write("ULTA", "I,Raw,Filt", "Bff", "#mm", "000", instance, self.height, self.filter.value)

    end

    return self
end

--- Init rangefinders with make height and filter frequency
---@param param_prefix string -- param table prefix
---@param param_key integer -- param table key
function ultrasonics.init(param_prefix, param_key)

    -- Parameter helper
    local function bind_add_param(name, idx, default_value)
        assert(param:add_param(param_key, idx, name, default_value), string.format('could not add param %s', name))
        return Parameter(param_prefix .. name)
    end

    -- Bind param table
    assert(param:add_table(param_key, param_prefix, 10), 'could not add param table')

    -- Bind parameters
    local rng_filt = bind_add_param('FILT', 1, 10)
    local comp_filt = bind_add_param('CFLT', 2, 10)

    ultrasonics.front = Rangefinder(0, rng_filt)
    ultrasonics.rear = Rangefinder(1, rng_filt)
    ultrasonics.complementary_filter = filter.get_complementary_filter(comp_filt)
end

--- Update rangefinders and return a height above water estimate
---@param vertical_velocity number -- earth frame climb rate in m/s
---@param attitude Quaternion_ud -- quaternion representing the vehicle attitude
---@param dt number -- time since last call in seconds
---@return number|nil -- height above water in meters, nil if not available
function ultrasonics.get_height(vertical_velocity, attitude, dt)

    -- Update both sensors
    ultrasonics.front.update(vertical_velocity, attitude, dt)
    ultrasonics.rear.update(vertical_velocity, attitude, dt)

    -- Use front sensor by preference
    local now_ms = millis()
    if (now_ms - ultrasonics.front.last_valid_ms) < 200 then
        return ultrasonics.complementary_filter.update(ultrasonics.front.filter.value, vertical_velocity, dt)
    end

    -- Otherwise use rear sensor
    if (now_ms - ultrasonics.rear.last_valid_ms) < 200 then
        return ultrasonics.complementary_filter.update(ultrasonics.rear.filter.value, vertical_velocity, dt)
    end

    -- No readings
    ultrasonics.complementary_filter.value = nil
    return nil
end


return ultrasonics
