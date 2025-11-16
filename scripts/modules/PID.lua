
-- PID object
local PID = {}

local filter = require("filter")

--- PID object
---@param log_name string -- 4 character name to be used in logging
---@param param_prefix string -- param table prefix
---@param param_key integer -- param table key
---@return table -- PID object
function PID.controller(log_name, param_prefix, param_key)
    local self = {}

    -- Parameter helper
    local function bind_add_param(name, idx, default_value)
        assert(param:add_param(param_key, idx, name, default_value), string.format('could not add param %s', name))
        return Parameter(param_prefix .. name)
    end

    -- Bind param table
    assert(param:add_table(param_key, param_prefix, 10), 'could not add param table')

    -- Bind parameters
    local kFF = bind_add_param('FF', 1, 0)
    local kP = bind_add_param('P', 2, 0)
    local kI = bind_add_param('I', 3, 0)
    local kD = bind_add_param('D', 4, 0)
    local filt_e = bind_add_param('FE', 5, 0)
    local filt_d = bind_add_param('FD', 6, 10)
    local filt_t = bind_add_param('FT', 7, 10)

    -- private fields as locals
    local _last_err = 0
    local _I = 0
    local _relax_tc = 0.15
    local relaxed = false
    local error_filter = filter.get_low_pass(filt_e)
    local d_filter = filter.get_low_pass(filt_d)
    local target_filter = filter.get_low_pass(filt_t)

    -- update the controller
    ---@param target number
    ---@param current number
    ---@param upper_limited boolean
    ---@param lower_limited boolean
    ---@param FF_scale number
    ---@param dt number
    ---@return number -- output
    function self.update(target, current, upper_limited, lower_limited, FF_scale, dt)
        local tar = target_filter.update(target, dt)
        local err = error_filter.update(tar - current, dt)

        -- FF
        local FF = kFF:get() * target
        FF = FF * FF_scale

        -- P
        local P = kP:get() * err

        -- I
        local delta_I = kI:get() * err * dt

        -- Don't allow I to increase in a limited direction
        if upper_limited then
            delta_I = math.min(delta_I, 0.0)
        end
        if lower_limited then
            delta_I = math.max(delta_I, 0.0)
        end
        _I = _I + delta_I

        -- D
        local D = 0
        if dt > 0 then
            D = d_filter.update((err - _last_err) / dt, dt) * kD:get()
        end
        _last_err = err

        local ret = FF + P + _I + D

        logger:write(log_name,'Targ,Tf,Curr,FF,P,I,D,Total,upper,lower,relax', 'ffffffffBBB', 
            target,
            tar,
            current,
            FF,
            P,
            _I,
            D,
            ret,
            upper_limited,
            lower_limited,
            relaxed
        )

        -- reset relax flag for next run
        relaxed = false

        return ret
    end

    ---Relax the integrator value towards the given value
    ---@param integrator number -- target integrator value
    ---@param dt number -- time since last call in seconds
    ---@return number -- current integrator value
    function self.relax_integrator(integrator, dt)
        relaxed = true
        _I = _I + (integrator - _I) * (dt / (dt + _relax_tc))
        return _I
    end

    ---Reset error and d term filter
    function self.reset_filters()
        error_filter.update()
        d_filter.update()
    end

    ---Set the target filter value
    ---@param value number -- target value
    ---@param dt number -- time since last call in seconds
    function self.update_target_filter(value, dt)
        target_filter.update(value, dt)
    end

    return self
end

--- Get angle controler object
---@param log_name string -- log file name
---@param param_prefix string -- parameter table perfix
---@param param_key integer -- parameter table key
---@return table
function PID.get_angle_controller(log_name, param_prefix, param_key)
    local self = {
        angle_error = 0,
        target_rate = 0
    }

    local rate_pid = PID.controller(log_name, param_prefix, param_key)

    -- Parameter helper
    local function bind_add_param(name, idx, default_value)
        assert(param:add_param(param_key, idx, name, default_value), string.format('could not add param %s', name))
        return Parameter(param_prefix .. name)
    end

    -- This is a bit of a hack to add some more params to the PID param table
    local angle_p = bind_add_param('ANG_P', 9, 1)
    local rate_max_param = bind_add_param('RAT_MAX', 10, 0)

    --- Update angle controller
    ---@param target_angle number -- target angle in degrees
    ---@param measured_angle number -- measured angle in degrees
    ---@param measured_rate number -- measured rate in degrees / second
    ---@param scale number -- speed scale factor
    ---@param upper_limited boolean -- true if the output is limited high
    ---@param lower_limited boolean -- true if the output id limited low
    ---@param dt number -- time since last call in seconds
    ---@return number -- output to be sent to actuator
    function self.update(target_angle, measured_angle, measured_rate, scale, upper_limited, lower_limited, dt)

        self.angle_error = target_angle - measured_angle

        self.target_rate = self.angle_error * angle_p:get()

        local rate_max = rate_max_param:get()
        if rate_max > 0 then
            -- Constrain to rate limit
            self.target_rate = math.min(self.target_rate,  rate_max)
            self.target_rate = math.max(self.target_rate, -rate_max)
        end

        -- Apply speed scale squared to the targets input into the PID controller
        local target = self.target_rate * scale * scale
        local measured = measured_rate * scale * scale

        -- FF is scaled by speed scale only, one over here removes the scale applied to the inputs
        return rate_pid.update(target, measured, upper_limited, lower_limited, 1 / scale, dt)
    end

    ---Relax the integrator value towards the given value
    -- Wrapper on rate controller function
    ---@param integrator number -- target integrator value
    ---@param dt number -- time since last call in seconds
    ---@return number -- current integrator value
    function self.relax_integrator(integrator, dt)
        return rate_pid.relax_integrator(integrator, dt)
    end

    ---Set the target filter value
    ---@param value number -- target value
    ---@param dt number -- time since last call in seconds
    function self.update_target_filter(value, dt)
        rate_pid.update_target_filter(value, dt)
    end

    ---Reset error and d term filter
    function self.reset_filters()
        rate_pid.reset_filters()
    end

    return self
end

return PID
