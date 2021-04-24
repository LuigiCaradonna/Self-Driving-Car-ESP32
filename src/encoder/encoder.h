#ifndef ENCODER
#define ENCODER
#include "Arduino.h"

class Encoder
{
public:
    Encoder(uint8_t slots);

    /*
     * Return the number of slots of the encoder
     * @return uint8_t the number of slots of the encoder
     */
    uint8_t getSlots();

    /*
     * Interrupt Service Routine called upon encoder's raising signal, calculates the motor's rpm
     * after an interval of at least one complete rotation
     * @return int the motor's rpm
     */
    int isr(unsigned long nowTime);

private:
    // Number of slots on the encoder
    const uint8_t slots;
    // Number of interrupts counted during the time interval
    unsigned long intCount;
    // Updated each time the ISR is called, it is used to calculate the time interval between interrupts
    unsigned long startTime;

    /* 
     * Given the wanted RPM returns the number of interrupt per second that is the number of times
     * that an encoder's slot activates the optocoupler sensor
     * @param int rpm value to convert in nterrupt per second
     * @return int the number of interrupts per second
     */
    int rpmToInterruptsPerSecond(int rpm);

    /*
     * Given the nummber of interrupt per second coming from the optocoupler sensor returns the RPM
     * @param int interrupt per second value to convert in rpm
     * @return int the number of rpm
     */
    int interruptsPerSecondToRPM(int interruptsPerSecond);
};

#endif