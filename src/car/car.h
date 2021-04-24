#ifndef CAR
#define CAR
#include <map>
#include <string>
#include "Arduino.h"
#include <motor/motor.h>
#include <math.h>

class Car
{
public:
    /*
     * Constructor
     * @param double car length in mm
     * @param double car width in mm
     * @param double wheels diameter in mm
     * @param double wheeltrack in mm
     * @param double wheelbase in mm
     * @param Motor front right motor
     * @param Motor front left motor
     * @param Motor rear right motor
     * @param Motor rear left motor
     */
    Car(
        double length,
        double width,
        double wheelDiameter,
        double wheeltrack,
        double wheelbase,
        Motor motorFR,
        Motor motorFL,
        Motor motorRR,
        Motor motorRL);

    /*
     * Getter fir the front right motor
     * @return Motor front right motor
     */
    Motor getMotorFR();
    
    /*
     * Getter fir the front left motor
     * @return Motor front left motor
     */
    Motor getMotorFL();
    
    /*
     * Getter fir the rear right motor
     * @return Motor rear right motor
     */
    Motor getMotorRR();
    
    /*
     * Getter fir the rear left motor
     * @return Motor rear left motor
     */
    Motor getMotorRL();

    /*
     * Given how many millimeters to move, returns how many encoedr's steps to count
     * @param int distance in millimeters to move
     * @return int number of encoder's slots to count
     */
    int mmToSlots(int mm);

    /*
     * Set the motor for forward movement
     * @param int pwm value for the FR motor
     * @param int pwm value for the FL motor
     * @param int pwm value for the RR motor
     * @param int pwm value for the RL motor
     */
    void forward(int pwmFR, int pwmFL, int pwmRR, int pwmRL);

    /*
     * Set the motor for reverse movement
     * @param int pwm value for the FR motor
     * @param int pwm value for the FL motor
     * @param int pwm value for the RR motor
     * @param int pwm value for the RL motor
     */
    void reverse(int pwmFR, int pwmFL, int pwmRR, int pwmRL);

    /*
     * Stop the car
     */
    void brake();

    /*
     * Set the motor to turn right
     * @param double how much to turn
     */
    void turnRight(double rate);

    /*
     * Set the motor to turn left
     * @param double how much to turn
     */
    void turnLeft(double rate);

private:
    // Car's dimensions
    // wheeltrack is the distance between the center of the wheels on the same axle
    // wheelbase is the distance between the front and rear axles
    const double length, width, wheelDiameter, wheeltrack, wheelbase;
    double mmPerStep;
    Motor motorFR, motorFL, motorRR, motorRL;
};

#endif
