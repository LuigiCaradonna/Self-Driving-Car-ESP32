#include <car/car.h>

Car::Car(uint16_t _length,
         uint16_t _width,
         uint16_t _wheeltrack,
         uint16_t _wheelbase,
         uint8_t _wheelDiameter,
         uint8_t _maxPwm,
         uint8_t _minPwm,
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

void Car::forward(uint16_t &pwmFR, uint16_t &pwmFL, uint16_t &pwmRR, uint16_t &pwmRL)
{
    direction = Direction::fwd;
    motorFR.forward(pwmFR);
    motorFL.forward(pwmFL);
    motorRR.forward(pwmRR);
    motorRL.forward(pwmRL);
}

void Car::reverse(uint16_t &pwmFR, uint16_t &pwmFL, uint16_t &pwmRR, uint16_t &pwmRL)
{
    direction = Direction::rev;
    motorFR.reverse(pwmFR);
    motorFL.reverse(pwmFL);
    motorRR.reverse(pwmRR);
    motorRL.reverse(pwmRL);
}

void Car::brake()
{
    direction = Direction::stop;
    motorFR.brake();
    motorFL.brake();
    motorRR.brake();
    motorRL.brake();
}

void Car::turnRight(double &deviation)
{
    /*
     * More deviation implies more speed difference between the two sides
     * i.e. with a deviation of 0.8 (turn a lot) the left motors will turn 80% faster than the right motors
     *      with a deviation of 0.2 (turn a bit) the left motors will turn 20% faster than the right motors
     */

    // Left motors' speed, set to the minimum pwm value
    uint16_t rightPwm = minPwm;

    // Left motors' speed, deviation% faster than the right ones
    uint16_t leftPwm = (int)(rightPwm + (rightPwm * deviation));
    // Be sure that the pwm value is not higher than the maximum acceptable
    leftPwm = leftPwm <= maxPwm ? leftPwm : maxPwm;

    if (getDirection() == Direction::fwd)
    {
        forward(rightPwm, leftPwm, rightPwm, leftPwm);
    }
    else if (getDirection() == Direction::rev)
    {
        reverse(rightPwm, leftPwm, rightPwm, leftPwm);
    }
}

void Car::turnLeft(double &deviation)
{
    /*
     * More deviation implies more speed difference between the two sides
     * i.e. with a deviation of 0.8 (turn a lot) the right motors will turn 80% faster than the left motors
     *      with a deviation of 0.2 (turn a bit) the right motors will turn 20% faster than the left motors
     */

    // Left motors' speed, set to the minimum pwm value
    uint16_t leftPwm = minPwm;

    // Right motors' speed, deviation% faster than the left ones
    uint16_t rightPwm = (int)(leftPwm + (leftPwm * deviation));
    // Be sure that the pwm value is not higher than the maximum acceptable
    rightPwm = rightPwm <= maxPwm ? rightPwm : maxPwm;

    if (getDirection() == Direction::fwd)
    {
        forward(rightPwm, leftPwm, rightPwm, leftPwm);
    }
    else if (getDirection() == Direction::rev)
    {
        reverse(rightPwm, leftPwm, rightPwm, leftPwm);
    }
}

int Car::mmToSlots(uint16_t &mm)
{
    return (int)round(mm / mmPerStep);
}

double Car::rpmToMs(uint16_t &rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min / 60 = m/sec
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) / 60;
}

double Car::rpmToKmh(uint16_t &rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min * 0.06 = Km/h
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) * 0.06;
}

int Car::speedRatioToPwm(double &ratio)
{
    double pwm;

    // Values higher than 1.0 are not allowed, stop the car
    if (abs(ratio) > 1)
        ratio = 0;

    /* 
     * Negative speed ratio is allowed to express the will to move in reverse 
     * but the PWM's duty cycle can only be positive, the negative part to 
     * decide the direction must be managed elsewhere
     */
    pwm = abs(maxPwm * ratio);

    // Be sure to return a duty cycle of 0 or not less than the minimum or not mor than the maximum
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