#ifndef CAR_H
#define CAR_H
#include "Arduino.h"
#include <motor/motor.h>
#include <math.h>

class Car
{
public:
    enum Direction
    {
        FWD,
        REV,
        STOP
    };

    /**
     * Constructor.
     * 
     * @param uint16_t length - Length in mm
     * @param uint16_t width - Width in mm
     * @param uint16_t wheeltrack - Wheeltrack in mm
     * @param uint16_t wheelbase - Wheelbase in mm
     * @param uint8_t wheelDiameter - Wheels diameter in mm
     * @param uint8_t maxPwm - Max pwm duty cycle
     * @param uint8_t minPwm - Min pwm duty cycle
     * @param double turnSpeedMultiplier - Multiplier for turning speed
     * @param Motor motorFR - Front right motor
     * @param Motor motorFL - Front left motor
     * @param Motor motorRR - Rear right motor
     * @param Motor motorRL - Rear left motor
     */
    Car(uint16_t,
        uint16_t,
        uint16_t,
        uint16_t,
        uint8_t,
        uint8_t,
        uint8_t,
        double,
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
     * Gets the direction where the car is heading (fwd, rev, stop).
     * 
     * @return Car's direction
     */
    Direction getDirection();

    /**
     * Given how many millimeters to move, returns how many encoedr's steps to count.
     * 
     * @param int distance in millimeters to move
     * @return the number of encoder's slots to count
     */
    int mmToSlots(const uint16_t &);

    /**
     * Given the rpm (avg if more motors with different rpm are running) calculates the speed in m/s.
     * 
     * @param int rpm
     * @return the speed in m/s
     */
    double rpmToMs(const uint16_t &);

    /**
     * Given the rpm (avg if more motors with different rpm are running) calculates the speed in Km/h.
     * 
     * @param int rpm
     * @return the speed in m/s
     */
    double rpmToKmh(const uint16_t &);

    /**
     * Set the motors for forward movement.
     * 
     * @param int pwm value for the Front Right motor
     * @param int pwm value for the Front Left motor
     * @param int pwm value for the Rear Right motor
     * @param int pwm value for the Rear Left motor
     */
    void forward(const uint16_t &, const uint16_t &, const uint16_t &, const uint16_t &);

    /**
     * Set the motors for reverse movement.
     * 
     * @param int pwm value for the Front Right motor
     * @param int pwm value for the Front Left motor
     * @param int pwm value for the Rear Right motor
     * @param int pwm value for the Rear Left motor
     */
    void reverse(const uint16_t &, const uint16_t &, const uint16_t &, const uint16_t &);

    /**
     * Stops the car
     */
    void brake();

    /**
     * Set the motors to turn right.
     * 
     * @param double how much to turn, must be a value within the range 0.0 to 1.0
     */
    void turnRight(const double &);

    /**
     * Set the motors to turn left.
     * 
     * @param double how much to turn, must be a value within the range 0.0 to 1.0
     */
    void turnLeft(const double &);

    /**
     * Given a speed ratio as an interval from 0.0 (stop) to 1.0 (full throttle), calculates 
     * the corresponding PWM duty cycle relative to the max value provided to the class constructor. 
     * Negative values can be provided to specify a backward movement. 
     * The method also takes care to do not return a value lower than the minimum 
     * provided to the class constructor, unless it has to return 0 which is an acceptable value. 
     * 
     * WARNING: When you want a speed of 0 you should use the Car::brake() method to stop the car.
     * 
     * @param double the wanted speed ratio in the interval 0.0 (stop) to 1.0 (full throttle)
     * @return the duty cycle to set for the PWM pins to have the wanted speed
     */
    int speedRatioToPwm(const double &);
    
private:
    /**
     * Car's dimensions in millimeters 
     * wheeltrack is the distance between the center of the wheels on the same axle 
     * wheelbase is the distance between the front and rear axles
     */
    const uint16_t length, width, wheeltrack, wheelbase;
    const uint8_t wheelDiameter, maxPwm, minPwm;
    const double turnSpeedMultiplier;

    // Distance in mm covered for each encoder's step
    double mmPerStep;
    // References to the motors instances
    Motor &motorFR, &motorFL, &motorRR, &motorRL;

    // Car's direction (fwd, rev, stop)
    Direction direction;
};

#endif
