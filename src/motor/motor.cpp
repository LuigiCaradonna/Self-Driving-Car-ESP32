#include <motor/motor.h>

Motor::Motor(uint16_t _channel1,
             uint16_t _channel2,
             Encoder _encoder) : channel1{_channel1},
                                 channel2{_channel2},
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
    ledcWrite(channel1, 0);
    ledcWrite(channel2, 0);
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
    ledcWrite(channel1, pwm);
    ledcWrite(channel2, 0);
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
    ledcWrite(channel2, 0);
    ledcWrite(channel2, pwm);
}
