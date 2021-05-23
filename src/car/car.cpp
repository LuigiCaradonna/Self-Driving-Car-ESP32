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

void Car::forward(int pwmFR, int pwmFL, int pwmRR, int pwmRL)
{
    motorFR.forward(pwmFR);
    motorFL.forward(pwmFL);
    motorRR.forward(pwmRR);
    motorRL.forward(pwmRL);
}

void Car::reverse(int pwmFR, int pwmFL, int pwmRR, int pwmRL)
{
    motorFR.reverse(pwmFR);
    motorFL.reverse(pwmFL);
    motorRR.reverse(pwmRR);
    motorRL.reverse(pwmRL);
}

void Car::brake()
{
    motorFR.brake();
    motorFL.brake();
    motorRR.brake();
    motorRL.brake();
}

void Car::turnRight(double rate)
{
    // TODO: slow down right wheels and/or accelerate left wheels
}

void Car::turnLeft(double rate)
{
    // TODO: slow down left wheels and/or accelerate right wheels
}

int Car::mmToSlots(int mm)
{
    return (int)round(mm / mmPerStep);
}

double Car::rpmToMs(int rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min / 60 = m/sec
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) / 60;
}

double Car::rpmToKmh(int rpm)
{
    /*
     * diameter * pi = circumference in millimeters
     * circ. in mm / 1000 = circ. in meters
     * circ. in m * rpm = m/min
     * m/min * 0.06 = Km/h
     */
    return (((wheelDiameter * 3.14159265358979323846) / 1000) * rpm) * 0.06;
}

int Car::speedRatioToPwm(double ratio)
{
    double pwm;

    // Negative values are normalized to 0
    if (ratio <= 0)
        ratio = 0;
    // Values higher than 100% are normalized to 1
    else if (ratio > 100)
        ratio = 1;
    // Values higher than 1 and lower or equal than 100 are normalized to the 0-1 interval
    else if (ratio > 1)
        ratio /= 100;
    
    /* 
     * the only other case: ratio > 0 and ratio <=1 
     * is already in the proper format used by this function
     */

    pwm = maxPwm * ratio;

    // Be sure to return a duty cycle of 0 or not less than the minimum
    if (pwm != 0 && pwm < minPwm) pwm = minPwm;

    return (int)pwm;
}