# Setting up a actuator

This is a guide setting up a foil actuator. A brush less motor is with absolute encode is connected to the ODrive. The ODrive is connected to a flight controller or periph node running this script.
The linear potentiometer is also connected, when using a CubeOrange the battery 1 voltage pin should be used, when using a G474 node the PWM 8 pin should be used.

# ODrive

To start with only the ODrive is used. It may be necessary to disconnect it from the lead screw to get it to calibrate successfully.

Setup the ODrive for motor and encoder and run calibration.
* Encoder is [AMT212B-V-OD](https://shop.odriverobotics.com/products/cui-amt212b-v-od)
* Control mode should be set to "position".
* CAN should be enabled:
    * baud rate: 1000000
    * Node ID: 10
    * Heartbeat enabled
    * Feedback sent every 250ms
* Setup [absolute encoder reference frame](https://docs.odriverobotics.com/v/latest/manual/control.html#absolute-encoder-reference-frame)

If the setup is successful you should be able to control the motor from the web gui in position mode.

# AP setup

This can be done on periph or a flight controller.

The safety switch must be enabled, changing safety state is what causes the script to initialize the ODrive.

### Flight controller

Setup a flight controller with the script and a scripting CAN port. Setup the GCS to display some named value floats.

* `stat`: script state
* `potVolt`: raw potentiometer voltage
* `potPos`: calculated potentiometer positions
* `pos`: position reported by ODrive (maybe NaN)


### Periph setup

See also node setup section. On periph the same debug values are available. They are printed to the debug console (if the debug param is set). This can be seen in the DroneCAN setup page of mission planner. If the flight controller has `CAN_LOGLEVEL` set (which it should) then the message should be duplicated in the messages tab.

### Common

Connect the flight controller CAN port to the ODrive.

There are a number of parameters with the `OD_` prefix:

* `OD_POS_MAX`: Max endpoint position, turns from centre
* `OD_POS_MIN`: Min endpoint position, turns from centre
* `OD_POT_MAX_VOLT`: Potentiometer voltage reading corresponding to max position endpoint position
* `OD_POT_MIN_VOLT`: Potentiometer voltage reading corresponding to min position endpoint position
* `OD_DEBUG`: Debug print enable on periph 0 to disable, 1 to enable.

The script is looking at the first servo output function, set this to `SERVOn_TRIM`, the output should be 1500 PWM.

# Setup

Now the endpoints of the potentiometer and feedback voltage from the pot must be calibrated.

1. Use the web gui to position the motor such that the rear surface of the foil is vertical.

2. Set `odrv0.axis0.pos_vel_mapper.config.offset` to the negative of the currently reported raw encoder position in the ODrive tool. This number should be in the range -0.5 to 0.5.

3. Reboot the ODrive without moving it and verify that the reported position is now zero.

4. Using the gui drive the motor to a number of positions (5+) in the possible full travel including the min and max endpoints. At each position take a note of:
    * The reported position in the gui.
    * The voltage reported to the GCS as `potVolt`.

5. Plot the points to double check that they are linear. If not check wiring.

6. Set the extreme endpoints in the flight controller parameters. `OD_POS_MAX` is the largest reported position and `OD_POT_MAX_VOLT` is the voltage which corresponds with that position. `OD_POS_MIN` and `OD_POT_MIN_VOLT` are for the smallest reported position.

7. The extremes of position should be larger than 13.15 turns as the code is expecting +-13.15 turns to equate to +-10 deg of foil movement.

8. Reboot both the ODrive and the flight controller.

9. Disable the safety switch, this will cause the script to set the position of the ODrive. It will then drive to the zero point as the servo output is 1500.

10. Verify that the foil has correctly returned to zero.

11. Change the servo trim to control the position demand. Move it to a few points in the travel and reboot both the flight controller and ODrive so it starts up again away from zero. Return the servo trim to 1500 and disable the safety switch and verify that the foil correctly returns to zero.

# Node setup

Now the configuration can be transferred over to the node. Flash the node firmware and copy the values of the `OD_` parameters over.

Setup the first output function to be the number of the servo output which the node should drive. EG 51 for front right, 52 for rear right, 53 for rear left, 54 for front left.

A static node ID should be used, this value is re-used for the ESC index for reporting ESC telem (small numbers should be used, less than 10).
