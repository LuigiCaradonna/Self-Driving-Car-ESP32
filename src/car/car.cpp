#include <car/car.h>

Car::Car(Wheel _wheel,
         double _length,
         double _width,
         double _wheeltrack,
         double _wheelbase,
         Motor _motorFR,
         Motor _motorRR,
         Motor _motorFL,
         Motor _motorRL) : wheel(_wheel),
                           length{_length},
                           width{_width},
                           wheeltrack{_wheeltrack},
                           wheelbase{_wheelbase},
                           motorFR{_motorFR},
                           motorRR{_motorRR},
                           motorFL{_motorFL},
                           motorRL{_motorRL}
{
}

Motor Car::getMotorFR()
{
    return motorFR;
}

Motor Car::getMotorRR()
{
    return motorRR;
}

Motor Car::getMotorFL()
{
    return motorFL;
}

Motor Car::getMotorRL()
{
    return motorRL;
}

void Car::turnRight(double rate)
{
    // TODO: slow down right wheels and/or accelerate left wheels
}

void Car::turnLeft(double rate)
{
    // TODO: slow down left wheels and/or accelerate right wheels
}

// Returns how many slots must the sensors count to move the given
// distance in centimeters
int Car::cmToSlots(double cm)
{
    // Wheels' circumference in cm
    float circumference = (wheel.getDiameter() * 3.14) / 10;
    // cm per step (any encoder is fine, all of them have the same slots)
    float cm_step = circumference / motorRR.getEncoder().getSlots();
    // Result as float
    float f_result = cm / cm_step;
    // Convert to integer (not rounded) and return
    return (int)f_result;
}
