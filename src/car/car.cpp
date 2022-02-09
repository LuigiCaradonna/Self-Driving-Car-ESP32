#include <car/car.h>

Car::Car(uint16_t _length,
         uint16_t _width,
         uint16_t _wheeltrack,
         uint16_t _wheelbase,
         uint8_t _wheelDiameter,
         uint8_t _maxPwm,
         uint8_t _minPwm,
         double _turnSpeedMultiplier,
         Motor &_motorFR,
         Motor &_motorFL,
         Motor &_motorRR,
         Motor &_motorRL) : length{_length},
                            width{_width},
                            wheeltrack{_wheeltrack},
                            wheelbase{_wheelbase},
                            wheelDiameter{_wheelDiameter},
                            maxPwm{_maxPwm},
                            minPwm{_minPwm},
                            turnSpeedMultiplier{_turnSpeedMultiplier},
                            motorFR{_motorFR},
                            motorFL{_motorFL},
                            motorRR{_motorRR},
                            motorRL{_motorRL}
{
    // Any encoder is fine, all the encoders have the same number of slots
    mmPerStep = (wheelDiameter * 3.14159265358979323846) / motorRR.getEncoder().getSlots();
}

Motor &Car::getMotorFR()
{
    return motorFR;
}

Motor &Car::getMotorFL()
{
    return motorFL;
}

Motor &Car::getMotorRR()
{
    return motorRR;
}

Motor &Car::getMotorRL()
{
    return motorRL;
}

Car::Direction Car::getDirection()
{
    return direction;
}

void Car::forward(const uint16_t &pwmFR, const uint16_t &pwmFL, const uint16_t &pwmRR, const uint16_t &pwmRL)
{
    direction = Direction::FWD;
    motorFR.forward(pwmFR);
    motorFL.forward(pwmFL);
    motorRR.forward(pwmRR);
    motorRL.forward(pwmRL);
}

void Car::reverse(const uint16_t &pwmFR, const uint16_t &pwmFL, const uint16_t &pwmRR, const uint16_t &pwmRL)
{
    direction = Direction::REV;
    motorFR.reverse(pwmFR);
    motorFL.reverse(pwmFL);
    motorRR.reverse(pwmRR);
    motorRL.reverse(pwmRL);
}

void Car::brake()
{
    direction = Direction::STOP;
    motorFR.brake();
    motorFL.brake();
    motorRR.brake();
    motorRL.brake();
}

void Car::turnRight(const double &deviation)
{
    /*
     * More deviation implies more speed difference between the two sides
     * i.e. with a deviation of 0.8 (turn a lot) the left motors will receive a PWM value 80% higher than the right motors
     *      with a deviation of 0.2 (turn a bit) the left motors will receive a PWM value 20% higher than the right motors
     * 
     * To those values, an additional multiplier (turnSpeedMultiplier) is needed or the difference in speed would be to low
     */

    // Right motors' speed, set to the minimum pwm value
    uint16_t rightPwm = minPwm;

    // Set the left motors' to run faster than the right ones
    uint16_t leftPwm = (int)((rightPwm + (rightPwm * deviation)) * turnSpeedMultiplier);
    // Be sure that the pwm value is not higher than the maximum acceptable
    leftPwm = leftPwm <= maxPwm ? leftPwm : maxPwm;

    if (getDirection() == Direction::FWD)
    {
        forward(rightPwm, leftPwm, rightPwm, leftPwm);
    }
    else if (getDirection() == Direction::REV)
    {
        reverse(rightPwm, leftPwm, rightPwm, leftPwm);
    }
}

void Car::turnLeft(const double &deviation)
{
    /*
     * More deviation implies more speed difference between the two sides
     * i.e. with a deviation of 0.8 (turn a lot) the right motors will receive a PWM value 80% higher than the left motors
     *      with a deviation of 0.2 (turn a bit) the right motors will receive a PWM value 20% higher than the left motors
     * 
     * To those values, an additional multiplier (turnSpeedMultiplier) is needed or the difference in speed would be to low
     */

    // Left motors' speed, set to the minimum pwm value
    uint16_t leftPwm = minPwm;

    // Set the right motors' to run faster than the left ones
    uint16_t rightPwm = (int)((leftPwm + (leftPwm * deviation)) * turnSpeedMultiplier);
    // Be sure that the pwm value is not higher than the maximum acceptable
    rightPwm = rightPwm <= maxPwm ? rightPwm : maxPwm;

    if (getDirection() == Direction::FWD)
    {
        forward(rightPwm, leftPwm, rightPwm, leftPwm);
    }
    else if (getDirection() == Direction::REV)
    {
        reverse(rightPwm, leftPwm, rightPwm, leftPwm);
    }
}

int Car::mmToSlots(const uint16_t &mm)
{
    return (int)round(mm / mmPerStep);
}

double Car::rpmToMs(const uint16_t &rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min / 60 = m/sec
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) / 60;
}

double Car::rpmToKmh(const uint16_t &rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min * 0.06 = Km/h
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) * 0.06;
}

int Car::speedRatioToPwm(const double &ratio)
{
    double pwm;

    // Values higher than 1.0 are not allowed, stop the car
    if (abs(ratio) > 1)
        return  0;

    /* 
     * Negative speed ratio is allowed to express the will to move in reverse 
     * but the PWM's duty cycle can only be positive, the negative part to 
     * decide the direction must be managed elsewhere
     */
    pwm = abs(maxPwm * ratio);

    // Be sure to return a duty cycle of 0 or not less than the minimum or not more than the maximum
    if (pwm != 0 && pwm < minPwm)
    {
        pwm = minPwm;
    }
    else if (pwm > maxPwm)
    {
        pwm = maxPwm;
    }

    return (int)pwm;
}