
#pragma once

class Spool {
public:

    // spool states
    enum class State : uint8_t {
        SHUT_DOWN = 0,                      // all motors stop
        GROUND_IDLE = 1,                    // all motors at ground idle
        SPOOLING_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        SPOOLING_DOWN = 4,                     // decreasing maximum throttle while stabilizing
    };

    // desired spool states
    enum class DesiredState : uint8_t {
        SHUT_DOWN = 0,              // all motors should move to stop
        GROUND_IDLE = 1,            // all motors should move to ground idle
        THROTTLE_UNLIMITED = 2,     // motors should move to being a state where throttle is unconstrained (e.g. by start up procedure)
    };

};
