-- This script is an example setting up a custom interpolated motor matrix
-- this allows a 1D interpolation between roll pitch and yaw factors for up to 12 motors

-- Tilt angles used for following interpolations (hover -> forward flight)
local interpolation_points = {0.000000, 9.000000, 18.000000, 27.000000, 36.000000, 45.000000, 54.000000, 63.000000, 72.000000, 81.000000, 90.000000}

-- Mechanical gain scaling, interpolated at `interpolation_points`: Per motor: Roll, Pitch, Yaw
local mechanical_gain_scaling = {}
-- hover
mechanical_gain_scaling[1] = {{-0.500000, 0.500000, 0.500000},
                              {0.499999, -0.250459, 0.306012},
                              {0.500000, 0.500000, -0.500000},
                              {-0.499999, -0.250459, -0.306012}}

mechanical_gain_scaling[2] = {{-0.499898, 0.514665, 0.389858},
                              {0.395665, -0.253968, 0.226701},
                              {0.499898, 0.514665, -0.389858},
                              {-0.395665, -0.253968, -0.226701}}

mechanical_gain_scaling[3] = {{-0.504949, 0.544114, 0.339880},
                              {0.320929, -0.264124, 0.192179},
                              {0.504949, 0.544114, -0.339880},
                              {-0.320929, -0.264124, -0.192179}}

mechanical_gain_scaling[4] = {{-0.514273, 0.597126, 0.327628},
                              {0.267929, -0.284468, 0.182473},
                              {0.514273, 0.597126, -0.327628},
                              {-0.267929, -0.284468, -0.182473}}

mechanical_gain_scaling[5] = {{-0.523734, 0.688088, 0.349329},
                              {0.231537, -0.320505, 0.193704},
                              {0.523734, 0.688088, -0.349329},
                              {-0.231537, -0.320505, -0.193704}}

mechanical_gain_scaling[6] = {{-0.527820, 0.842441, 0.411708},
                              {0.207807, -0.381288, 0.230259},
                              {0.527820, 0.842441, -0.411708},
                              {-0.207807, -0.381288, -0.230259}}

mechanical_gain_scaling[7] = {{-0.523949, 1.111749, 0.531412},
                              {0.193401, -0.483516, 0.305937},
                              {0.523949, 1.111749, -0.531412},
                              {-0.193401, -0.483516, -0.305937}}

mechanical_gain_scaling[8] = {{-0.514827, 1.624802, 0.733449},
                              {0.185531, -0.663765, 0.450602},
                              {0.514827, 1.624802, -0.733449},
                              {-0.185531, -0.663765, -0.450602}}

mechanical_gain_scaling[9] = {{-0.506211, 2.832834, 1.029646},
                              {0.182044, -1.024499, 0.717791},
                              {0.506211, 2.832834, -1.029646},
                              {-0.182044, -1.024499, -0.717791}}

mechanical_gain_scaling[10] = {{-0.503686, 8.248986, 1.340039},
                               {0.181420, -2.009697, 1.136927},
                               {0.503686, 8.248986, -1.340039},
                               {-0.181420, -2.009697, -1.136927}}

-- forward flight
mechanical_gain_scaling[11] = {{-0.511821, 0.000000, 1.444169},
                               {0.182644, 0.000000, 1.444169},
                               {0.511821, 0.000000, -1.444169},
                               {-0.182644, 0.000000, -1.444169}}

-- Inertia gain scaling, interpolated at `interpolation_points`: Roll, Pitch, Yaw
local inertia_gain_scaling = {}
inertia_gain_scaling[1] = {1.000000, 1.000000, 1.000000} -- hover
inertia_gain_scaling[2] = {1.127464, 1.005565, 1.032627}
inertia_gain_scaling[3] = {1.507883, 0.961043, 1.090441}
inertia_gain_scaling[4] = {2.046453, 0.878957, 1.164576}
inertia_gain_scaling[5] = {2.626953, 0.771826, 1.255688}
inertia_gain_scaling[6] = {3.141053, 0.652174, 1.360921}
inertia_gain_scaling[7] = {3.526329, 0.531826, 1.464909}
inertia_gain_scaling[8] = {3.785222, 0.419826, 1.555013}
inertia_gain_scaling[9] = {3.935341, 0.324522, 1.619737}
inertia_gain_scaling[10] = {3.989587, 0.254261, 1.649684}
inertia_gain_scaling[11] = {3.953125, 0.217391, 1.638889} -- forward flight

-- Manual gain scaling, interpolated at `interpolation_points`: Roll, Pitch, Yaw
local manual_gain_scaling = {}
manual_gain_scaling[1] = {1.000000, 1.000000, 1.000000} -- hover
manual_gain_scaling[2] = {1.000000, 1.000000, 1.000000}
manual_gain_scaling[3] = {1.000000, 1.000000, 0.200000}
manual_gain_scaling[4] = {1.000000, 1.000000, 0.200000}
manual_gain_scaling[5] = {1.000000, 0.900000, 0.200000}
manual_gain_scaling[6] = {1.000000, 0.800000, 0.200000}
manual_gain_scaling[7] = {1.000000, 0.700000, 0.200000}
manual_gain_scaling[8] = {1.000000, 0.600000, 0.200000}
manual_gain_scaling[9] = {1.000000, 0.500000, 0.200000}
manual_gain_scaling[10] = {1.000000, 0.500000, 0.200000}
manual_gain_scaling[11] = {1.000000, 0.50000, 0.200000} -- forward flight

-- P to D ratio gain scaling, interpolated at `interpolation_points`: Roll, Pitch, Yaw
local PD_gain_scaling = {}
PD_gain_scaling[1] = {0.013043, 0.025676, 0.000000} -- hover
PD_gain_scaling[2] = {0.013043, 0.025676, 0.010000}
PD_gain_scaling[3] = {0.013043, 0.025676, 0.010000}
PD_gain_scaling[4] = {0.013043, 0.025676, 0.010000}
PD_gain_scaling[5] = {0.013043, 0.025676, 0.010000}
PD_gain_scaling[6] = {0.013043, 0.025, 0.010000}
PD_gain_scaling[7] = {0.013043, 0.02, 0.010000}
PD_gain_scaling[8] = {0.013043, 0.015, 0.010000}
PD_gain_scaling[9] = {0.013043, 0.01, 0.010000}
PD_gain_scaling[10] = {0.013043, 0.01, 0.010000}
PD_gain_scaling[11] = {0.013043, 0.01, 0.010000} -- forward flight

-- wing control surface scaling, interpolated at `interpolation_points`: aileron, elevator
local aileron_gain_scaling = {}
aileron_gain_scaling[1] = {0.000000, 0.000000} -- hover
aileron_gain_scaling[2] = {0.000000, 0.000000}
aileron_gain_scaling[3] = {0.000000, 0.000000}
aileron_gain_scaling[4] = {0.000000, 0.000000}
aileron_gain_scaling[5] = {0.000000, 0.000000}
aileron_gain_scaling[6] = {1.000000, 0.000000}
aileron_gain_scaling[7] = {0.33333, 0.000000}
aileron_gain_scaling[8] = {0.666667, 0.000000}
aileron_gain_scaling[9] = {1.000000, 0.000000}
aileron_gain_scaling[10] = {1.000000, 0.000000}
aileron_gain_scaling[11] = {1.000000, 0.000000} -- forward flight

-- tail control surface scaling, interpolated at `interpolation_points`: rudder, elevator
local tail_gain_scaling = {}
tail_gain_scaling[1] = {0.000000, 0.000000} -- hover
tail_gain_scaling[2] = {0.333333, 0.333333}
tail_gain_scaling[3] = {0.666667, 0.666667}
tail_gain_scaling[4] = {1.000000, 1.000000}
tail_gain_scaling[5] = {1.000000, 1.000000}
tail_gain_scaling[6] = {1.000000, 1.000000}
tail_gain_scaling[7] = {1.000000, 1.000000}
tail_gain_scaling[8] = {1.000000, 1.000000}
tail_gain_scaling[9] = {1.000000, 1.000000}
tail_gain_scaling[10] = {1.000000, 1.000000}
tail_gain_scaling[11] = {1.000000, 1.000000} -- forward flight


-- Max overall scaling values for motors
local motor_max = 1.5

-- setup code
assert(#interpolation_points == #mechanical_gain_scaling,'incorrect number of mechanical scaling points')
assert(#interpolation_points == #inertia_gain_scaling,'incorrect number of inertia scaling points')
assert(#interpolation_points == #manual_gain_scaling,'incorrect number of manual scaling points')
assert(#interpolation_points == #PD_gain_scaling,'incorrect number of PD scaling points')
assert(#interpolation_points == #aileron_gain_scaling,'incorrect number of aileron scaling points')
assert(#interpolation_points == #tail_gain_scaling,'incorrect number of tail scaling points')

-- apply manual and inertial scaling to mechanical per-motor gains and load
for i = 1, #interpolation_points do
    -- loop over interpolation points
    for j = 1, #mechanical_gain_scaling[i] do
        -- loop over motors
        for k = 1, 3 do
            -- loop over roll, pitch and yaw
            mechanical_gain_scaling[i][j][k] = mechanical_gain_scaling[i][j][k] * inertia_gain_scaling[i][k] * manual_gain_scaling[i][k]
            -- constrain to max value
            mechanical_gain_scaling[i][j][k] = math.min(mechanical_gain_scaling[i][j][k],motor_max)
            mechanical_gain_scaling[i][j][k] = math.max(mechanical_gain_scaling[i][j][k],-motor_max)
        end
    end
end

local factors = factor_table()
-- populate factors table and load
for j = 1, 3 do
    factors:roll(k-1,mechanical_gain_scaling[1][j][1])
    factors:pitch(k-1,mechanical_gain_scaling[1][j][2])
    factors:yaw(k-1,mechanical_gain_scaling[1][j][3])
    factors:throttle(k-1,1)
end
Motors_dynamic:load_factors(factors)

Motors_dynamic:add_motor(0, 1) -- motor number (zero indexed) and testing order (1 indexed)
Motors_dynamic:add_motor(1, 3)
Motors_dynamic:add_motor(2, 4)
Motors_dynamic:add_motor(3, 2)

-- setup pitch offset before motors init so arming will fail
local Q_trim = Parameter()
assert(Q_trim:init('Q_TRIM_PITCH'),'get Q_TRIM_PITCH failed')

-- load PID params
local roll_P = Parameter()
assert(roll_P:init('Q_A_RAT_RLL_P'),'get Q_A_RAT_RLL_P failed')

local pitch_P = Parameter()
assert(pitch_P:init('Q_A_RAT_PIT_P'),'get Q_A_RAT_PIT_P failed')

local yaw_P =  Parameter()
assert(yaw_P:init('Q_A_RAT_YAW_P'),'get Q_A_RAT_YAW_P failed')

local roll_D = Parameter()
assert(roll_D:init('Q_A_RAT_RLL_D'),'get Q_A_RAT_RLL_D failed')

local pitch_D = Parameter()
assert(pitch_D:init('Q_A_RAT_PIT_D'),'get Q_A_RAT_PIT_D failed')

local yaw_D =  Parameter()
assert(yaw_D:init('Q_A_RAT_YAW_D'),'get Q_A_RAT_YAW_D failed')

-- try and init with 4 motors, each motor must have been added first
assert(Motors_dynamic:init(4), "Failed to init Motors_interp")

local current_tilt
local wing_aileron_scale
local wing_elevator_scale
local tail_rudder_scale
local tail_elevator_scale

local function linear_interpolate(low_output, high_output, var_value, var_low, var_high)
    local p = (var_value - var_low) / (var_high - var_low)
    return low_output + p * (high_output - low_output)
end

local function interp_table(tilt, lookup_table)
    -- find the correct points in the lookup table
    -- use a table structure to hold return values
    local return_val = {}

    -- check extremes
    if tilt <= interpolation_points[1] then
        -- tilt lower than min value in table
        for j = 1, #lookup_table[1] do
            return_val[j] = lookup_table[1][j]
        end

    elseif tilt >= interpolation_points[#interpolation_points] then
        -- tilt higher than max value
        for j = 1, #lookup_table[#interpolation_points] do
            return_val[j] = lookup_table[#interpolation_points][j]
        end

    else
        for i = 1, #interpolation_points-1 do
            if tilt >= interpolation_points[i] and tilt < interpolation_points[i+1] then
                for j = 1, #lookup_table[#interpolation_points] do
                    return_val[j] = linear_interpolate(lookup_table[i][j], lookup_table[i+1][j], tilt, interpolation_points[i], interpolation_points[i+1])
                end
                break
            end
        end
    end

    -- unpack table and return multiple values
    return table.unpack(return_val)
end

local function update_interpolation()

    -- update motors interpolation point from the script in real-time
    for k = 1, 3 do
        -- check extremes
        if current_tilt <= interpolation_points[1] then
            -- tilt lower than min value in table
            factors:roll(k-1,  mechanical_gain_scaling[1][k][1])
            factors:pitch(k-1, mechanical_gain_scaling[1][k][2])
            factors:yaw(k-1,   mechanical_gain_scaling[1][k][3])
            factors:throttle(k-1,1)

        elseif current_tilt >= interpolation_points[#interpolation_points] then
            -- tilt higher than max value
            factors:roll(k-1,  mechanical_gain_scaling[#interpolation_points][k][1])
            factors:pitch(k-1, mechanical_gain_scaling[#interpolation_points][k][2])
            factors:yaw(k-1,   mechanical_gain_scaling[#interpolation_points][k][3])
            factors:throttle(k-1,1)

        else
            for i = 1, #interpolation_points-1 do
                if current_tilt >= interpolation_points[i] and current_tilt < interpolation_points[i+1] then
                    factors:roll(k-1,  linear_interpolate(mechanical_gain_scaling[i][k][1], mechanical_gain_scaling[i+1][k][1], current_tilt, interpolation_points[i], interpolation_points[i+1])
                    factors:pitch(k-1, linear_interpolate(mechanical_gain_scaling[i][k][2], mechanical_gain_scaling[i+1][k][2], current_tilt, interpolation_points[i], interpolation_points[i+1])
                    factors:yaw(k-1,   linear_interpolate(mechanical_gain_scaling[i][k][3], mechanical_gain_scaling[i+1][k][3], current_tilt, interpolation_points[i], interpolation_points[i+1])
                    factors:throttle(k-1,1)
                    break
                end
            end
        end
    end
    Motors_dynamic:load_factors(factors)

    -- update pitch offset to calculated angle
    --Q_trim:set(current_tilt)
    attitude_control_rotate:set_offset_roll_pitch(0,current_tilt)

    -- print to GCS
    gcs:send_named_float("TILT_ANGLE",current_tilt)

    -- get the gain scaling at the current tilt for the wing control surfaces
    wing_aileron_scale, wing_elevator_scale = interp_table(current_tilt,aileron_gain_scaling)

    -- get the gain scaling at the current tilt for the tail control surfaces
    tail_rudder_scale, tail_elevator_scale = interp_table(current_tilt,tail_gain_scaling)

    -- get P to D ratio
    local roll_PD, pitch_PD, yaw_PD = interp_table(current_tilt,PD_gain_scaling)
    
    -- set D gains from P
    roll_D:set(roll_P:get() * roll_PD)
    pitch_D:set(pitch_P:get() * pitch_PD)
    yaw_D:set(yaw_P:get() * yaw_PD)

end

function update()

    -- k_motor_tilt is option 41, it has a range of 1000, convert to angle 0 -> 90
    local new_tilt = (SRV_Channels:get_output_scaled(41) / 1000) * 90

    -- only update interpolation points if there is a change
    if new_tilt ~= current_tilt then
        current_tilt = new_tilt
        local success, err = pcall(update_interpolation)
        if not success then
            current_tilt = nil
            gcs:send_text(0, "Interp Error: " .. err)
            return update, 10
        end
    end

    -- get the aileron, elevator and rudder outputs
    local aileron = SRV_Channels:get_output_scaled(4)
    local elevator = SRV_Channels:get_output_scaled(19)
    local rudder = SRV_Channels:get_output_scaled(21)

    -- apply to scripting outputs, set_output_scaled is expecting a int

    -- Wing control surfaces on 94:Script 1 and 95:Script 2
    -- tail control surfaces on 96:Script 3 and 97:Script 4

    -- left aileron
    SRV_Channels:set_output_scaled(94, math.floor(aileron*wing_aileron_scale - elevator*wing_elevator_scale + 0.5))

    -- right aileron
    SRV_Channels:set_output_scaled(95, math.floor(aileron*wing_aileron_scale + elevator*wing_elevator_scale + 0.5))

    -- left A tail
    SRV_Channels:set_output_scaled(96, math.floor(elevator*tail_elevator_scale - rudder*tail_rudder_scale + 0.5))

    -- right A tail
    SRV_Channels:set_output_scaled(97, math.floor(elevator*tail_elevator_scale + rudder*tail_rudder_scale+ 0.5))

    return update, 10 -- 100 hz
end

return update()
