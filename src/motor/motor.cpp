#include <motor/motor.h>

Motor::Motor(uint8_t _channel1,
             uint8_t _channel2,
             Encoder *_encoder) : channel1{_channel1},
                                  channel2{_channel2},
                                  encoder{_encoder}
{
}

/*
 * Get the encoder assigned to the motor
 * @return the encoder assigned to the motor
 */
Encoder &Motor::getEncoder()
{
    return *encoder;
}

/*
 * Get the spin direction of the motor
 * @return the spin direction of the motor: fwd, rev or stop
 */
Motor::Direction Motor::getDirection()
{
    return direction;
}

/*
 * Stop the motor
 */
void Motor::brake()
{
    // Set the direction to stop
    direction = Direction::stop;
    ledcWrite(channel1, 0);
    ledcWrite(channel2, 0);
}

/*
 * Set the motor for forward movement
 * @param the pwm value for the spinning speed
 */
void Motor::forward(int pwm)
{
    // stop the motor before to invert the direction to prevent a current peak
    if (direction == Direction::rev)
    {
        brake();
        vTaskDelay(DELAY_CHANGE_DIRECTION);
    }

    // Set the direction to forward
    direction = Direction::fwd;
    ledcWrite(channel1, pwm);
    ledcWrite(channel2, 0);
}

/*
 * Set the the motor for reverse movement
 * @param the pwm value for the spinning speed
 */
void Motor::reverse(int pwm)
{
    // stop the motor before to invert the direction to prevent a current peak
    if (direction == Direction::fwd)
    {
        brake();
        vTaskDelay(DELAY_CHANGE_DIRECTION);
    }

    // Set the direction to reverse
    direction = Direction::rev;
    ledcWrite(channel1, 0);
    ledcWrite(channel2, pwm);
}
