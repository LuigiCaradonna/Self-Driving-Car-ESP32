#ifndef ENCODER
#define ENCODER
#include "Arduino.h"

class Encoder
{
public:
    Encoder(int slots);

    // Interrupt Service Routine called upon a raised interrupt from this encoder
    double isr(unsigned long nowTime);

    int rpmToInterruptsPerSecond(int rpm);

    int interruptsPerSecondToRPM(int interruptsPerSecond);

    int getSlots();

private:
    // Number of slots on the encoder
    const int slots;
    // Number of interrupts counted during the time interval
    unsigned long intCount;
    unsigned long startTime;
};

#endif