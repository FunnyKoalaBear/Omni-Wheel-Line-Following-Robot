#ifndef HCSR04_H
#define HCSR04_H

#include "mbed.h"

class HCSR04 {
public:
    // Constructor
    HCSR04(PinName triggerPin, PinName echoPin);
    
    // Public methods
    void trigger();
    uint32_t pulse_us() const;
    uint32_t distance_cm() const; 

private:
    // Hardware pins and timers
    DigitalOut _trigger;
    InterruptIn _echo;
    Timer _timer;
    
    // Internal state
    volatile uint32_t _pulse_us;
    
    // Interrupt service routines
    void echo_rise_isr();
    void echo_fall_isr();
};

#endif