-- Script to output to foil control surfaces

-- Use scripting input functions for roll/pitch control when disarmed
local roll_chan = assert(rc:find_channel_for_option(300), "no RCx_OPTION 300")
local pitch_chan = assert(rc:find_channel_for_option(301), "no RCx_OPTION 301")
local flap_chan = assert(rc:find_channel_for_option(302), "no RCx_OPTION 302")

-- Load in and setup modules
local filter = require("filter")
local PID = require("PID")
local ultrasonics = require("Ultrasonics")

--- Linear interpolation helper
---@param x number -- input value
---@param xLow number -- lower limit of input
---@param xHigh number -- upper limit of input
---@param yLow number -- lower limit of output
---@param yHigh number -- upper limit of output
---@return number
local function LERP(x, xLow, xHigh, yLow, yHigh)
    local ratio = (x - xLow) / (xHigh - xLow)
    return yLow + ratio * (yHigh - yLow)
end

--- Output channel helper object
---@param functionNum integer -- function number to output to
---@param minAngle number -- min angle in degrees
---@param maxAngle number -- max angle in degrees
---@param trimAngle Parameter_ud_const -- trim angle paramgeter in degrees
---@return table -- outputChan object
local function outputChan(functionNum, minAngle, maxAngle, trimAngle)
    local self = {
        upperLimit = true,
        lowerLimit = true,
        value = 0.5,
    }

    -- Set range type output
    SRV_Channels:set_range(functionNum, 1.0)

    --- Set output for given chanel in the range -1 to 1
    ---@param value number
    function self.update(value)
        -- apply trim
        value = value + LERP(trimAngle:get(), minAngle, maxAngle, 0.0, 1.0)

        -- Constrain to output range, set flags if limited
        self.upperLimit = value >= 1.0
        if self.upperLimit then
            value = 1.0
        end

        self.lowerLimit = value <= 0.0
        if self.lowerLimit then
            value = 0.0
        end

        -- Store scaled value
        self.value = value

        -- Output to servo
        SRV_Channels:set_output_scaled(functionNum, value)
    end

    --- Return output angle in degrees
    ---@return number
    function self.getAngle()
        return LERP(self.value, 0.0, 1.0, minAngle, maxAngle)
    end

    return self
end

local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "FOIL_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 50), 'could not add param table')

-- Trim is applied directly to the actuator outputs with no scale
local FRONT_TRIM = bind_add_param('FRONT_TRIM', 1, 0)
local REAR_TRIM = bind_add_param('REAR_TRIM', 2, 0)

-- Pitch target
local PITCH_TARGET = bind_add_param('PIT_ANG_TAR', 3, 0)

-- Speed scaling for control surfaces
local SCALE_SPEED = bind_add_param('SCL_SPD', 4, 10)
local SCALE_SPEED_MIN = bind_add_param('SCL_SPD_MIN', 5, 10)
local SCALE_SPEED_MAX = bind_add_param('SCL_SPD_MAX', 6, 10)

-- Height target above the water
local HEIGHT_TAR = bind_add_param('HEIGHT_TAR', 7, 0.125)

-- Height target filter in Hz
local HEIGHT_FILT = bind_add_param('HEIGHT_FILT', 8, 10)

-- Speed at which we cannot be flying, used for is-flying checks
local FLY_SPD_MIN = bind_add_param('FLY_SPD_MIN', 9, 4.0)

-- Height below which we cannot be flying.
local FLY_H_MIN = bind_add_param('FLY_H_MIN', 10, 0.06)

-- ANGLE P, rate PID
local roll_PID = PID.get_angle_controller("FRLL", PARAM_TABLE_PREFIX .. 'RLL_', PARAM_TABLE_KEY + 1)
local pitch_PID = PID.get_angle_controller("FPIT", PARAM_TABLE_PREFIX .. 'PIT_', PARAM_TABLE_KEY + 2)

local height_PID = PID.controller("FALT", PARAM_TABLE_PREFIX .. 'HEIGHT_', PARAM_TABLE_KEY + 3)

ultrasonics.init(PARAM_TABLE_PREFIX .. 'ULTRA_', PARAM_TABLE_KEY + 4)

local outputs = {
    frontRight = outputChan(94, -5.0,  15.0, FRONT_TRIM), -- Scripting 1
    rearRight  = outputChan(95, -10.0, 10.0, REAR_TRIM),  -- Scripting 2
    rearLeft   = outputChan(96, -10.0, 10.0, REAR_TRIM),  -- Scripting 3
    frontLeft  = outputChan(97, -5.0,  15.0, FRONT_TRIM), -- Scripting 4
}
local heightFilter = filter.get_low_pass(HEIGHT_FILT)

-- Get speed scale factor, this reduces with speed to try and linearize force resulting from actuator deflection
---@param speed number -- current speed in m/s
---@return number -- speed scale factor
local function get_speed_scale(speed)

    local scale_min = SCALE_SPEED:get() / SCALE_SPEED_MAX:get()
    local scale_max = SCALE_SPEED:get() / SCALE_SPEED_MIN:get()
    local scale = SCALE_SPEED:get() / speed

    scale = math.min(scale, scale_max)
    scale = math.max(scale, scale_min)

    return scale
end

-- Write log message
---@param scale number -- speed scale factor
---@param is_flying boolean -- is flying flag
---@param raw_height number|nil -- unfiltered height estimate
---@param height number|nil -- filtered height estimate
---@param dt number -- time since last call in seconds
local function write_log(scale, is_flying, raw_height, height, dt)

    -- Can't log nils
    local log_raw_height = -1
    if raw_height ~= nil then
        log_raw_height = raw_height
    end

    local log_height = -1
    if height ~= nil then
        log_height = height
    end

    logger:write('FOIL', 'SS,FR,RR,RL,FL,Rh,Fh,flying,dt', 'fffffffBf',
        scale,
        outputs.frontRight.getAngle(),
        outputs.rearRight.getAngle(),
        outputs.rearLeft.getAngle(),
        outputs.frontLeft.getAngle(),
        log_raw_height,
        log_height,
        is_flying,
        dt
    )

end

-- Logging for angle and rate controllers
---@param target_roll number
---@param measured_roll number
---@param target_pitch number
---@param measured_pitch number
---@param roll_rate number
---@param pitch_rate number
local function log_angle_control(target_roll, measured_roll, target_pitch, measured_pitch, roll_rate, pitch_rate)

    logger:write('FANG', 'Rt,R,Re,Pt,P,Pe,Rrt,Rr,Prt,Pr', 'ffffffffff',
        target_roll,
        measured_roll,
        roll_PID.angle_error,
        target_pitch,
        measured_pitch,
        pitch_PID.angle_error,
        roll_PID.target_rate,
        roll_rate,
        pitch_PID.target_rate,
        pitch_rate
    )

end

-- Under this height is flying is always false
local flying_height = 0.2

local last_update_us = micros()
local function update()

    -- Calculate the time since last update
    local now_us = micros()
    local dt = (now_us - last_update_us):tofloat() * 0.000001
    last_update_us = now_us

    -- Read AHRS in one place for consistency and speed
    local attitude = ahrs:get_quaternion()
    local velocity_ned = ahrs:get_velocity_NED()
    local gyro = ahrs:get_gyro()

    -- Ensure attitude is valid
    if attitude == nil then
        attitude = Quaternion()
    end

    -- Calculate speed and climb rate from velocity
    local speed = 0.0
    local climb_rate = 0.0
    if velocity_ned then
        climb_rate = velocity_ned:z() * -1.0

        -- Note that this function is named incorrectly, it should be `body_to_earth`
        -- See: https://github.com/ArduPilot/ardupilot/pull/28178
        attitude:inverse():earth_to_body(velocity_ned)
        speed = velocity_ned:x()
    end

    -- Get height estimate from ultrasonics
    local raw_height = ultrasonics.get_height(climb_rate, attitude, dt)
    local height = heightFilter.update(raw_height, dt)

    local roll_out
    local pitch_out
    local flap_out

    -- If armed run the PIDs
    local is_flying = false
    local scale = 1
    if not arming:is_armed() then
        -- Pilot controls direct to outputs
        -- Roll pitch inputs scale -1 to 1 to -0.5 to 0.5
        roll_out = roll_chan:norm_input() * 0.5
        pitch_out = pitch_chan:norm_input() * 0.5
        flap_out = flap_chan:norm_input() * 0.5

        -- Reset and relax PIDs
        roll_PID.update_target_filter(0.0, dt)
        pitch_PID.update_target_filter(0.0, dt)
        height_PID.update_target_filter(flying_height, dt)

        roll_PID.reset_filters()
        pitch_PID.reset_filters()
        height_PID.reset_filters()

        roll_PID.relax_integrator(0.0, dt)
        pitch_PID.relax_integrator(0.0, dt)
        height_PID.relax_integrator(0.0, dt)

    else
        -- Automatic stabilization

        -- Update speed scale factor
        scale = get_speed_scale(speed)

        -- Decide if flying or not
        if (height ~= nil) and (height > FLY_H_MIN:get()) and (speed > FLY_SPD_MIN:get()) then
            is_flying = true
        end
        local I_relax = is_flying == false

        -- Work out limit flags
        local roll_upper = I_relax or outputs.rearLeft.upperLimit or outputs.rearRight.lowerLimit
        local roll_lower = I_relax or outputs.rearLeft.lowerLimit or outputs.rearRight.upperLimit

        local pitch_upper = I_relax or outputs.rearLeft.lowerLimit or outputs.rearRight.lowerLimit
        local pitch_lower = I_relax or outputs.rearLeft.lowerLimit or outputs.rearRight.lowerLimit

        local flap_upper = I_relax or outputs.frontRight.upperLimit or outputs.frontLeft.upperLimit
        local flap_lower = I_relax or outputs.frontRight.lowerLimit or outputs.frontLeft.lowerLimit

        -- If not flying decay I term to 0, limit flags will prevent I increase
        if I_relax then
            roll_PID.relax_integrator(0.0, dt)
            pitch_PID.relax_integrator(0.0, dt)
        end

        -- Constant roll, pitch and height targets
        local targetPitch = PITCH_TARGET:get()
        local targetRoll = 0.0
        local targetHeight = HEIGHT_TAR:get()

        -- Measurments
        local measuredRoll = math.deg(attitude:get_euler_roll())
        local measuredPitch = math.deg(attitude:get_euler_pitch())
        local rollRate =  math.deg(gyro:x())
        local pitchRate =  math.deg(gyro:y())

        -- ANGLE P, rate PID for roll and pitch
        roll_out = roll_PID.update(targetRoll, measuredRoll, rollRate, scale, roll_upper, roll_lower, dt)
        pitch_out = pitch_PID.update(targetPitch, measuredPitch, pitchRate, scale, pitch_upper, pitch_lower, dt)

        -- Log angles
        log_angle_control(targetRoll, measuredRoll, targetPitch, measuredPitch, rollRate, pitchRate)

        -- Height PID with flap
        if I_relax or (height == nil) then
            flap_out = height_PID.relax_integrator(0.0, dt)
        end
        if height ~= nil then
            flap_out = height_PID.update(targetHeight, height, flap_upper, flap_lower, 1.0, dt)
        end

    end

    -- Only flap on front foils
    outputs.frontLeft.update(flap_out)
    outputs.frontRight.update(flap_out)

    -- Rear do both roll and pitch
    outputs.rearLeft.update(-pitch_out + roll_out)
    outputs.rearRight.update(-pitch_out - roll_out)

    -- Write log
    write_log(scale, is_flying, raw_height, height, dt)

    -- Run at 100Hz
    return update, 10
end

return update()
