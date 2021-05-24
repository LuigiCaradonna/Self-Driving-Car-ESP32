#ifndef CAR_H
#define CAR_H
#include "Arduino.h"
#include <motor/motor.h>
#include <math.h>

class Car
{
public:
    /**
     * Constructor.
     * 
     * @param uint16_t car length in mm
     * @param uint16_t car width in mm
     * @param uint16_t wheeltrack in mm
     * @param uint16_t wheelbase in mm
     * @param uint8_t wheels diameter in mm
     * @param uint8_t max pwm duty cycle
     * @param uint8_t min pwm duty cycle
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
        uint8_t,
        uint8_t,
        Motor &,
        Motor &,
        Motor &,
        Motor &);

    /**
     * Getter for the front right motor.
     * 
     * @return front right motor
     */
    Motor &getMotorFR();

    /**
     * Getter for the front left motor.
     * 
     * @return front left motor
     */
    Motor &getMotorFL();

    /**
     * Getter for the rear right motor.
     * 
     * @return rear right motor
     */
    Motor &getMotorRR();

    /**
     * Getter for the rear left motor.
     * 
     * @return rear left motor
     */
    Motor &getMotorRL();

    /**
     * Given how many millimeters to move, returns how many encoedr's steps to count.
     * 
     * @param int distance in millimeters to move
     * @return the number of encoder's slots to count
     */
    int mmToSlots(int);

    /**
     * Given the rpm (avg if more motors with different rpm are running) calculates the speed in m/s.
     * 
     * @param int rpm
     * @return the speed in m/s
     */
    double rpmToMs(int);

    /**
     * Given the rpm (avg if more motors with different rpm are running) calculates the speed in Km/h.
     * 
     * @param int rpm
     * @return the speed in m/s
     */
    double rpmToKmh(int);

    /**
     * Set the motors for forward movement.
     * 
     * @param int pwm value for the Front Right motor
     * @param int pwm value for the Front Left motor
     * @param int pwm value for the Rear Right motor
     * @param int pwm value for the Rear Left motor
     */
    void forward(int, int, int, int);

    /**
     * Set the motors for reverse movement.
     * 
     * @param int pwm value for the Front Right motor
     * @param int pwm value for the Front Left motor
     * @param int pwm value for the Rear Right motor
     * @param int pwm value for the Rear Left motor
     */
    void reverse(int, int, int, int);

    /**
     * Stop the car
     */
    void brake();

    /**
     * Set the motors to turn right.
     * 
     * @param double how much to turn
     */
    void turnRight(double);

    /**
     * Set the motors to turn left.
     * 
     * @param double how much to turn
     */
    void turnLeft(double);

    /**
     * Given a speed ratio as an interval from 0.0 (stop) to 1.0 (full throttle), calculates 
     * the corresponding PWM duty cycle relative to the max value provided to the class constructor. 
     * Negative values can be provided, they will be converted to positive. 
     * The method also takes care to do not return a value lower than the minimum 
     * provided to the class constructor, unless it has to return 0 which is an acceptable value. 
     * 
     * WARNING: When you want a speed of 0 you should use the Car::brake() method to stop the car.
     * 
     * @param double the wanted speed ratio in the interval 0.0 (stop) to 1.0 (full throttle)
     * @return the duty cycle to set for the PWM pins to have the wanted speed
     */
    int speedRatioToPwm(double);
    
private:
    /**
     * Car's dimensions in millimeters 
     * wheeltrack is the distance between the center of the wheels on the same axle 
     * wheelbase is the distance between the front and rear axles
     */
    const uint16_t length, width, wheeltrack, wheelbase;
    const uint8_t wheelDiameter, maxPwm, minPwm;

    // Distance in mm covered for each encoder's step
    double mmPerStep;
    // References to the motors instances
    Motor &motorFR, &motorFL, &motorRR, &motorRL;
};

#endif
