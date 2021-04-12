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

    Motor(uint16_t channel, uint16_t pin1, uint16_t pin2, Encoder encoder);

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
    const uint16_t channel, pin1, pin2;

    Direction direction;
    Encoder encoder;
};

#endif