#include <motor/motor.h>

Motor::Motor(uint16_t _channel,
             uint16_t _pin1,
             uint16_t _pin2,
             Encoder _encoder) : channel{_channel},
                                 pin1{_pin1},
                                 pin2{_pin2},
                                 encoder{_encoder}
{
}

Encoder Motor::getEncoder()
{
    return encoder;
}

Motor::Direction Motor::getDirection()
{
    return direction;
}

void Motor::brake()
{
    // Set the direction to stop
    direction = Direction::stop;
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
}

void Motor::forward(int pwm)
{
    // stop the motor before to invert the direction to prevent a current peek
    if (direction == Direction::rev)
    {
        brake();
        vTaskDelay(20);
    }

    // Set the direction to forward
    direction = Direction::fwd;
    ledcWrite(channel, pwm);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
}

void Motor::reverse(int pwm)
{
    // stop the motor before to invert the direction to prevent a current peek
    if (direction == Direction::fwd)
    {
        brake();
        vTaskDelay(20);
    }

    // Set the direction to reverse
    direction = Direction::rev;
    ledcWrite(channel, pwm);
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
}
