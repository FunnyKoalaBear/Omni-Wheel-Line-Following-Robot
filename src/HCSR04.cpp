#include "HCSR04.h"
 
HCSR04::HCSR04(PinName triggerPin, PinName echoPin)
    : _trigger(triggerPin), _echo(echoPin), _pulse_us(0)
{
    _timer.stop();
    _timer.reset();
    _echo.rise(callback(this, &HCSR04::echo_rise_isr));
    _echo.fall(callback(this, &HCSR04::echo_fall_isr));
    _trigger = 0;
}
 
void HCSR04::trigger()
{
    _pulse_us = 0;
    _trigger = 1;
    wait_us(10);
    _trigger = 0;
}
 
void HCSR04::echo_rise_isr()
{
    _timer.reset();
    _timer.start();
}
 
void HCSR04::echo_fall_isr()
{
    _timer.stop();
    _pulse_us = _timer.elapsed_time().count();
    _timer.reset();
}
 
uint32_t HCSR04::pulse_us() const
{
    return _pulse_us;
}
 
uint32_t HCSR04::distance_cm() const
{
    if (_pulse_us == 0) {
        return 0; // no valid measurement
    }
 
    /*
     * Calculation:
     * Speed of sound = 343 m/s
     * Convert to cm/µs:
     *   343 m/s = (343 x 100) / 1,000,000
     *
     * The echo pulse width represents the round-trip travel time
     * of the ultrasonic wave (to the object and back), so the
     * one-way distance is half of this value.
     *
     * Distance (cm) = pulse_us x (343 x 100) / 1,000,000 / 2
     *               = pulse_us x 343 / 20000
     */
    return (_pulse_us * 343U) / 20000U;
}
 
 