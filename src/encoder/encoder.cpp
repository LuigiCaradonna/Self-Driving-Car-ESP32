#include <encoder/encoder.h>

Encoder::Encoder(uint8_t _slots) : slots{_slots}
{
    startTime = 0;
    intCount = 0;
}

/*
 * Return the number of slots of the encoder
 * @return uint8_t the number of slots of the encoder
 */
uint8_t Encoder::getSlots()
{
    return slots;
}

/* 
 * Given the wanted RPM returns the number of interrupt per second that is the number of times
 * that an encoder's slot activates the optocoupler sensor
 * @param int rpm value to convert in nterrupt per second
 * @return int the number of interrupts per second
 */
int Encoder::rpmToInterruptsPerSecond(int rpm)
{
    double interruptsPerSecond = (rpm * slots) / 60.00;
    return (int)interruptsPerSecond;
}

/*
 * Given the nummber of interrupt per second coming from the optocoupler sensor returns the RPM
 * @param int interrupt per second value to convert in rpm
 * @return int the number of rpm
 */
int Encoder::interruptsPerSecondToRPM(int interruptsPerSecond)
{
    double rpm = (interruptsPerSecond / slots) * 60.00;
    return (int)rpm;
}

/*
 * Interrupt Service Routine called upon encoder's raising signal, calculates the motor's rpm
 * after an interval of at least half rotation of the wheel
 * @return int the motor's rpm
 */
int Encoder::isr(unsigned long nowTime)
{
    double rpm = -1;
    
    intCount++;
    // Execute only after at least half rotation of the encoder since the previous execution
    if (intCount >= (slots/2))
    {
        int interruptsPerSecond = (int)(slots * 1000 / (nowTime - startTime));
        rpm = interruptsPerSecondToRPM(interruptsPerSecond);
        startTime = nowTime;
        // Reset the interrupt count
        intCount = 0;
    }

    return rpm;
}
