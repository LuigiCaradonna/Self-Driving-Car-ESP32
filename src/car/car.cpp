#include <car/car.h>

Car::Car(uint16_t _length,
         uint16_t _width,
         uint16_t _wheeltrack,
         uint16_t _wheelbase,
         uint8_t _wheelDiameter,
         Motor &_motorFR,
         Motor &_motorFL,
         Motor &_motorRR,
         Motor &_motorRL) : length{_length},
                            width{_width},
                            wheeltrack{_wheeltrack},
                            wheelbase{_wheelbase},
                            wheelDiameter{_wheelDiameter},
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
