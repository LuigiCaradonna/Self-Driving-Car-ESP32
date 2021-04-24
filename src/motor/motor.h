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

    Motor(uint8_t channel1, uint8_t channel2, Encoder encoder);

    /*
     * Get the encoder assigned to the motor
     * @return the encoder assigned to the motor
     */
    Encoder getEncoder();

    /*
     * Get the spin direction of the motor
     * @return the spin direction of the motor: fwd, rev or stop
     */
    Direction getDirection();

    /*
     * Stop the motor
     */
    void brake();

    /*
     * Set the motor for forward movement
     * @param the pwm value for the spinning sped
     */
    void forward(int pwm);

    /*
     * Set the the motor for reverse movement
     * @param the pwm value for the spinning sped
     */
    void reverse(int pwm);

private:
    const uint8_t channel1, channel2;
    const uint8_t DELAY_CHANGE_DIRECTION = 20; // ms to wait after braking to change direction

    Direction direction;
    Encoder encoder;
};

#endif