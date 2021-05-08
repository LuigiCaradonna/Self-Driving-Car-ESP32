#ifndef WHEEL
#define WHEEL
#include "Arduino.h"

class Wheel
{
public:
    Wheel(double diameter);

    double getDiameter();

private:
    const double diameter;
};

#endif