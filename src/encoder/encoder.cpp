#include <encoder/encoder.h>

Encoder::Encoder(uint8_t _slots) : slots{_slots}
{
    startTime = 0;
    intCount = 0;
}

uint8_t Encoder::getSlots()
{
    return slots;
}

uint16_t Encoder::rpmToInterruptsPerSecond(uint16_t rpm)
{
    double interruptsPerSecond = (rpm * slots) / 60.00;
    return (uint16_t)interruptsPerSecond;
}

uint16_t Encoder::interruptsPerSecondToRPM(double interruptsPerSecond)
{
    double rpm = (interruptsPerSecond / slots) * 60.00;
    return (uint16_t)rpm;
}

int16_t Encoder::isr(unsigned long nowTime)
{
    uint16_t rpm = -1;
    
    intCount++;
    
    // Execute only after at least half rotation of the encoder since the previous execution
    if (intCount >= (slots/4))
    {
        double timeDiff = nowTime - startTime;
        double interruptsPerSecond = intCount / (timeDiff) * 1000;
        rpm = interruptsPerSecondToRPM(interruptsPerSecond);
        startTime = nowTime;

        // Reset the interrupt count
        intCount = 0;
    }

    return rpm;
}
