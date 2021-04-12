#ifndef CAR
#define CAR
#include <map>
#include <string>
#include "Arduino.h"
#include <motor/motor.h>
#include <wheel/wheel.h>

class Car
{
public:
    Car(Wheel wheel, double length, double width, double wheeltrack, double wheelbase, Motor motorRR, Motor motorRL);

    // Returns how many slots must the sensors count to move the given
    // distance in centimeters
    int cmToSlots(double cm);

    Motor getMotorRR();
    Motor getMotorRL();

    void turnRight(double rate);
    void turnLeft(double rate);

private:
    Wheel wheel;
    // Car's dimensions
    // wheeltrack is the distance between the center of the wheels on the same axle
    // wheelbase is the distance between the front and rear axles
    const double length, width, wheeltrack, wheelbase;
    Motor motorRR, motorRL;
};

#endif
