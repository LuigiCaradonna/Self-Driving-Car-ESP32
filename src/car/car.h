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
     * @param uint16_t car length in mm
     * @param uint16_t car width in mm
     * @param double wheels diameter in mm
     * @param uint16_t wheeltrack in mm
     * @param uint16_t wheelbase in mm
     * @param Motor front right motor
     * @param Motor front left motor
     * @param Motor rear right motor
     * @param Motor rear left motor
     */
    Car(uint16_t,
        uint16_t,
        uint16_t,
        uint16_t,
        uint8_t,
        Motor *,
        Motor *,
        Motor *,
        Motor *);

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
    int mmToSlots(int);

    /*
     * Set the motor for forward movement
     * @param int pwm value for the FR motor
     * @param int pwm value for the FL motor
     * @param int pwm value for the RR motor
     * @param int pwm value for the RL motor
     */
    void forward(int, int, int, int);

    /*
     * Set the motor for reverse movement
     * @param int pwm value for the FR motor
     * @param int pwm value for the FL motor
     * @param int pwm value for the RR motor
     * @param int pwm value for the RL motor
     */
    void reverse(int, int, int, int);

    /*
     * Stop the car
     */
    void brake();

    /*
     * Set the motor to turn right
     * @param double how much to turn
     */
    void turnRight(double);

    /*
     * Set the motor to turn left
     * @param double how much to turn
     */
    void turnLeft(double);

private:
    // Car's dimensions in millimeters
    // wheeltrack is the distance between the center of the wheels on the same axle
    // wheelbase is the distance between the front and rear axles
    const uint16_t length, width, wheeltrack, wheelbase;
    const uint8_t wheelDiameter;

    // Distance in mm covered for each encoder's step
    double mmPerStep;
    Motor *motorFR, *motorFL, *motorRR, *motorRL;
};

#endif
