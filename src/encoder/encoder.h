#ifndef ENCODER_H
#define ENCODER_H
#include "Arduino.h"

class Encoder
{
public:
    Encoder(uint8_t);

    /**
     * Return the number of slots of the encoder
     * 
     * @return uint8_t the number of slots of the encoder
     */
    uint8_t getSlots();

    /**
     * Interrupt Service Routine called upon encoder's raising signal, calculates the motor's rpm 
     * after an interval of at least 1/4 of a complete rotation of the wheel
     * 
     * @return the motor's rpm, -1 if a new value is not yet ready
     */
    int16_t isr(unsigned long);

private:
    // Number of slots on the encoder
    const uint8_t slots;
    // Number of interrupts counted during the time interval
    double volatile intCount;
    // Updated each time the ISR is called, it is used to calculate the time interval between interrupts
    unsigned long volatile startTime;

    /** 
     * Given the wanted RPM returns the number of interrupt per second, that is the number of times 
     * that an encoder's slot activates the optocoupler sensor
     * 
     * @param int rpm value to convert in nterrupt per second
     * @return int the number of interrupts per second
     */
    uint16_t rpmToInterruptsPerSecond(uint16_t);

    /**
     * Given the number of interrupt per second coming from the optocoupler sensor returns the RPM
     * 
     * @param int interrupt per second value to convert in rpm
     * @return int the number of rpm
     */
    uint16_t interruptsPerSecondToRPM(double);
};

#endif