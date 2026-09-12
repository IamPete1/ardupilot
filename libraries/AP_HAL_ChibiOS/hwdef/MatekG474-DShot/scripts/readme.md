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

Setup a min endstop (but leave it disabled), set absolute_setpoints.

# AP setup

This can be done on periph or a flight controller.

The safety switch must be enabled, changing safety state is what causes the script to initialize the ODrive.

### Flight controller

Setup a flight controller with the script and a scripting CAN port. Setup the GCS to display some named value floats.

* `stat`: script state
* `pos`: position reported by ODrive (maybe NaN)


### Periph setup

See also node setup section. On periph the same debug values are available. They are printed to the debug console (if the debug param is set). This can be seen in the DroneCAN setup page of mission planner. If the flight controller has `CAN_LOGLEVEL` set (which it should) then the message should be duplicated in the messages tab.

### Common

Connect the flight controller CAN port to the ODrive.

There are a number of parameters with the `OD_` prefix:

* `OD_POS_MAX`: Max endpoint position, turns from centre
* `OD_POS_MIN`: Min endpoint position, turns from centre
* `OD_DEBUG`: Debug print enable on periph 0 to disable, 1 to enable.

The script is looking at the first servo output function, set this to `SERVOn_TRIM`, the output should be 1500 PWM.

# Setup


# Node setup

Now the configuration can be transferred over to the node. Flash the node firmware and copy the values of the `OD_` parameters over.

Setup the first output function to be the number of the servo output which the node should drive. EG 51 for front right, 52 for rear right, 53 for rear left, 54 for front left.

A static node ID should be used, this value is re-used for the ESC index for reporting ESC telem (small numbers should be used, less than 10).
