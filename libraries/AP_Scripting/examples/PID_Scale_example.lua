-- Example of accessing AC_PID objects, tilt angle and aileron gains
-- DO NOT FLY WITH THIS SCRIPT

-- get quadplane PID objects
local roll_pid = vehicle:get_AC_PID(vehicle.RATE_ROLL)
local pitch_pid = vehicle:get_AC_PID(vehicle.RATE_PITCH)
local yaw_pid = vehicle:get_AC_PID(vehicle.RATE_YAW)

if not roll_pid or not pitch_pid or not yaw_pid then
  error('Could not find Quadplane PIDs')
end

-- set the range of a scripting servo output to use a corrected tilt
SRV_Channels:set_range(94,1000)

function update()
  ------------------------------------------------------------------------------------------------------------------------
  -- Quadplane PID
  ------------------------------------------------------------------------------------------------------------------------

  -- Print the gain values from the PID controllers
  gcs:send_text(0,string.format("Roll: P %0.2f, I %0.2f, D %0.2f, scale %0.2f",roll_pid:kP(),roll_pid:kI(),roll_pid:kD(),roll_pid:scale()))
  gcs:send_text(0,string.format("Pitch: P %0.2f, I %0.2f, D %0.2f, scale %0.2f",pitch_pid:kP(),pitch_pid:kI(),pitch_pid:kD(),pitch_pid:scale()))
  gcs:send_text(0,string.format("Yaw: P %0.2f, I %0.2f, D %0.2f, scale %0.2f",yaw_pid:kP(),yaw_pid:kI(),yaw_pid:kD(),yaw_pid:scale()))

  -- Example of setting and getting the one of new scale parameters Q_A_RAT_PIT_SCAL, Q_A_RAT_RLL_SCAL, Q_A_RAT_YAW_SCAL
  local pitch_scale = pitch_pid:scale()
  pitch_scale = pitch_scale + 0.1 -- increment the scale factor by 0.1 each loop, obviously don't fly with this
  pitch_pid:set_scale(pitch_scale)


  --[[ All methods available
  reset_I()
  reset_filter()

  -- get accessors
  kP()
  kI()
  kD()
  ff()
  filt_T_hz()
  filt_E_hz()
  filt_D_hz()
  imax()
  get_filt_alpha(float)
  get_filt_T_alpha()
  get_filt_E_alpha()
  get_filt_D_alpha()
  scale()

  -- set accessors, only saved permanently once save_gains() is called (scale factor is never saved, so will always default to whatever it is set to manually)
  set_kP(float)
  set_kI(float)
  set_kD(float)
  set_ff(float)
  set_imax(float)
  set_filt_T_hz(float)
  set_filt_E_hz(float)
  set_filt_D_hz(float)
  save_gains()
  set_scale()
  ]]--

  ------------------------------------------------------------------------------------------------------------------------
  -- Reading tilt angle and output to scripting servo
  ------------------------------------------------------------------------------------------------------------------------
  -- Tilt angle in AP is a linear output.  This example shows how the linear tilt angle is read from the tilt servo function (Where no servo should be connected), 
  -- modified/mapped to make it non-linear, and the subsequent position is then output on a scripting servo (where the tilt servo is actually connected).

  local tilt_servo = SRV_Channels:get_output_scaled(41) -- 41 is the SERVOx_FUNCTION value of motor tilt
  -- tilt angle is in the range +-1000, note that this is not the same for all servo functional
  -- see: https://github.com/ArduPilot/ardupilot/blob/83d5df257e8aff7804b72c7216fe0806931b512a/libraries/SRV_Channel/SRV_Channel_aux.cpp#L104
  -- 0 is VTOl, 1000 is forward flight

  -- convert the servo position to a angle in deg, 0 is VTOL, 90 is forward flight
  local tilt_angle = (tilt_servo/1000) * 90

  -- AP assumes that the servo output is a linear map to the tilt angle, you could correct for this and output on a second servo function
  -- the motor tilt function we are reading does not need to be setup on a output for this to work
  local corrected_tilt_angle = tilt_angle * 1 -- some clever maths function that mapps from linear to non-linear to output
  local corrected_tilt = (corrected_tilt_angle / 90) * 1000 -- we set the range to 1000 on line 14 to match the tilt servo, output is a integer so we cant just use a range of 1 and set a output as a decimal
  SRV_Channels:set_output_scaled(94,math.floor(corrected_tilt)) -- output to servo function 94 (scripting 1), we must round to a integer

  gcs:send_text(0,string.format("Tilt input angle %0.2f deg, corrected %0.2f deg",tilt_angle,corrected_tilt_angle))

  ------------------------------------------------------------------------------------------------------------------------
  -- Plane aileron
  ------------------------------------------------------------------------------------------------------------------------
  -- new scaling parameter RLL2SRV_SCALE and PTCH2SRV_SCALE
  -- plane roll and pitch are not as nice to access as AC_PID used for VTOL flight as we have to go via vehicle each time
  gcs:send_text(0,string.format("Plane Roll: P %0.2f, I %0.2f, D %0.2f, scale %0.2f",vehicle:roll_kP(),vehicle:roll_kI(),vehicle:roll_kD(),vehicle:roll_scale()))
  gcs:send_text(0,string.format("Plane Pitch: P %0.2f, I %0.2f, D %0.2f, scale %0.2f",vehicle:pitch_kP(),vehicle:pitch_kI(),vehicle:pitch_kD(),vehicle:pitch_scale()))

  -- setting the scale factor for roll
  local roll_scale = vehicle:roll_scale()
  roll_scale = roll_scale + 0.1
  vehicle:roll_set_scale(pitch_scale)

  --[[ All methods
    roll_kP()
    roll_kI()
    roll_kD()
    roll_kFF()
    roll_scale()

    roll_set_kP(float)
    roll_set_kI(float)
    roll_set_kD(float)
    roll_set_kFF(float)
    roll_set_scale(float)

    float pitch_kP()
    float pitch_kI()
    float pitch_kD()
    float pitch_kFF()
    float pitch_scale()

    pitch_set_kP(float)
    pitch_set_kI(float)
    pitch_set_kD(float)
    pitch_set_kFF(float)
    pitch_set_scale(float)
  ]]

  ------------------------------------------------------------------------------------------------------------------------
  -- Logging
  ------------------------------------------------------------------------------------------------------------------------
  -- the new scale factors are not logged in the existing PID logs, so we can log them manually here
  -- we can also log whatever other calculations from the script we like

  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- not all format types are supported by scripting.  These are the only types supported: i, L, e, f, n, M, B, I, E, and N
  -- lua automatically adds a timestamp in micro seconds
  logger.write('SCR','QR,QP,QY,R,P,tilt_ang,corrected','fffffff',roll_pid:scale(),pitch_pid:scale(),yaw_pid:scale(),vehicle:roll_scale(),vehicle:pitch_scale(),tilt_angle,corrected_tilt_angle)



  -- this is a 1hz loop so the prints don't get overwhelming, should be able to run at 400hz at least with out the prints, this is the quadplane loop rate so any faster would be pointless anyway
  -- logging might need a slower rate to keep up depending on how much other stuff is logging ect
  return update, 1000 -- reschedules the loop in 1 second
end

return update() -- run immediately before starting to reschedule
