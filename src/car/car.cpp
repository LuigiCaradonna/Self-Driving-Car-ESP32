#include <car/car.h>

Car::Car(double _length,
         double _width,
         double _wheeltrack,
         double _wheelDiameter,
         double _wheelbase,
         Motor _motorFR,
         Motor _motorFL,
         Motor _motorRR,
         Motor _motorRL) : length{_length},
                           width{_width},
                           wheelDiameter{_wheelDiameter},
                           wheeltrack{_wheeltrack},
                           wheelbase{_wheelbase},
                           motorFR{_motorFR},
                           motorFL{_motorFL},
                           motorRR{_motorRR},
                           motorRL{_motorRL}
{
    // Any encoder is fine, all the encoders have the same number of slots
    mmPerStep = (wheelDiameter * 3.14159265358979323846) / motorRR.getEncoder().getSlots();
}

/*
 * Getter fir the front right motor
 * @return Motor front right motor
 */
Motor Car::getMotorFR()
{
    return motorFR;
}

/*
 * Getter fir the front left motor
 * @return Motor front left motor
 */
Motor Car::getMotorFL()
{
    return motorFL;
}

/*
 * Getter fir the rear right motor
 * @return Motor rear right motor
 */
Motor Car::getMotorRR()
{
    return motorRR;
}

/*
 * Getter fir the rear left motor
 * @return Motor rear left motor
 */
Motor Car::getMotorRL()
{
    return motorRL;
}

/*
 * Set the motor for forward movement
 * @param int pwm value for the FR motor
 * @param int pwm value for the FL motor
 * @param int pwm value for the RR motor
 * @param int pwm value for the RL motor
 */
void Car::forward(int pwmFR, int pwmFL, int pwmRR, int pwmRL)
{
    motorFR.forward(pwmFR);
    motorFL.forward(pwmFL);
    motorRR.forward(pwmRR);
    motorRL.forward(pwmRL);
}

/*
 * Set the motor for reverse movement
 * @param int pwm value for the FR motor
 * @param int pwm value for the FL motor
 * @param int pwm value for the RR motor
 * @param int pwm value for the RL motor
 */
void Car::reverse(int pwmFR, int pwmFL, int pwmRR, int pwmRL)
{
    motorFR.reverse(pwmFR);
    motorFL.reverse(pwmFL);
    motorRR.reverse(pwmRR);
    motorRL.reverse(pwmRL);
}

/*
 * Stop the car
 */
void Car::brake()
{
    motorFR.brake();
    motorFL.brake();
    motorRR.brake();
    motorRL.brake();
}

/*
 * Set the motor to turn right
 * @param double how much to turn
 */
void Car::turnRight(double rate)
{
    // TODO: slow down right wheels and/or accelerate left wheels
}

/*
 * Set the motor to turn left
 * @param double how much to turn
 */
void Car::turnLeft(double rate)
{
    // TODO: slow down left wheels and/or accelerate right wheels
}

/*
 * Given how many millimeters to move, returns how many encoedr's steps to count
 * @param int distance in millimeters to move
 * @return int number of encoder's slots to count
 */
int Car::mmToSlots(int mm)
{
    return (int)round(mm / mmPerStep);
}
