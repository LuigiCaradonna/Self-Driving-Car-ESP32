#include <encoder/encoder.h>

Encoder::Encoder(int _slots) : slots{_slots}
{
    startTime = 0;
}

int Encoder::getSlots()
{
    return slots;
}

// Given the wanted RPM returns the number of interrupt per second that is the number of times
// that an encoder's slot activates the optocoupler sensor
int Encoder::rpmToInterruptsPerSecond(int rpm)
{
    double interruptsPerSecond = (rpm * slots) / 60;
    return (int)interruptsPerSecond;
}

// Given the nummber of interrupt per second coming from the optocoupler sensors returns the RPM
int Encoder::interruptsPerSecondToRPM(int interruptsPerSecond)
{
    double rpm = (interruptsPerSecond * 60) / slots;
    return (int)rpm;
}

double Encoder::isr(unsigned long nowTime)
{
    double input = -1;
    // count sufficient interrupts to get accurate timing
    // inputX is the motor RPM
    intCount++;
    // Wait a complete rotation of the encoder
    if (intCount == slots)
    {
        int interruptsPerSecond = (int)((double)slots * 1000 / (double)(nowTime - startTime));
        input = interruptsPerSecondToRPM(interruptsPerSecond);
        startTime = nowTime;
        // Reset the interrupt count
        intCount = 0;
    }

    return input;
}
