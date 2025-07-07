// Example for testing mode filters

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/ModeFilter
    ./build/linux/examples/ModeFilter
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create an filter instance to test
ModeFilterFloat_Size5 filter {3};

// setup routine
void setup()
{
    hal.console->printf("ModeFilter test\n");
    hal.console->printf("input, output\n");

    // Run 1000 samples
    for (uint16_t i = 0; i < 1000; i++) {

        const float input = rand_float();
        const float output = filter.apply(input);

        hal.console->printf("%+0.6f, %+0.6f\n", input, output);
    }

    // Wait a while for print buffer to empty and exit
    hal.scheduler->delay(1000);
    exit(0);
}

void loop() {};

AP_HAL_MAIN();
