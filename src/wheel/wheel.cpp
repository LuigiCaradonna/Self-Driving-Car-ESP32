#include <wheel/wheel.h>

Wheel::Wheel(double _diameter) : diameter{_diameter}
{
}

double Wheel::getDiameter()
{
    return diameter;
}