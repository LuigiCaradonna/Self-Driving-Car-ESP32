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

    Motor(uint16_t channel1, uint16_t channel2, Encoder encoder);

    // Get the motor encoder
    Encoder getEncoder();

    // Get motor direction
    Direction getDirection();

    // Stop the motor
    void brake();

    // Move forward
    void forward(int pwm);

    // Move backward
    void reverse(int pwm);

private:
    const uint16_t channel1, channel2;

    Direction direction;
    Encoder encoder;
};

#endif