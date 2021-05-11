#ifndef MOTOR
#define MOTOR
#include "Arduino.h"
#include <encoder/encoder.h>

class Motor
{
public:
    enum Direction
    {
        fwd,
        rev,
        stop
    };

    Motor(uint8_t, uint8_t, Encoder &);

    /**
     * Get the encoder assigned to the motor.
     * 
     * @return the encoder assigned to the motor
     */
    Encoder &getEncoder();

    /**
     * Get the spin direction of the motor.
     * 
     * @return the spin direction of the motor: fwd, rev or stop
     */
    Direction getDirection();

    /**
     * Stop the motor
     */
    void brake();

    /**
     * Set the motor for forward movement.
     * 
     * @param int the pwm value for the spinning speed
     */
    void forward(int);

    /**
     * Set the the motor for reverse movement.
     * 
     * @param int the pwm value for the spinning speed
     */
    void reverse(int);

private:
    const uint8_t channel1, channel2;

    // ms to wait after braking to change direction
    const uint8_t DELAY_CHANGE_DIRECTION = 20;

    Direction direction;
    Encoder &encoder;
};

#endif