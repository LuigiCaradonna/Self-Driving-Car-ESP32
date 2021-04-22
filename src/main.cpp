#include <Arduino.h>
#include <pid/pid.h>
#include <car/car.h>
#include <motor/motor.h>
#include <wheel/wheel.h>
#include <encoder/encoder.h>

// Right Motors
const uint16_t RIGHT_M_EN = 5;  // Right DRV883 SLEEP pin
const uint16_t MOTOR_FR_1 = 18; // FR Motor IN1
const uint16_t MOTOR_FR_2 = 19; // FR Motor IN2
const uint16_t MOTOR_RR_1 = 16; // RR Motor IN1
const uint16_t MOTOR_RR_2 = 17; // RR Motor IN2

// Left Motors
const uint16_t LEFT_M_EN = 25;  // Left DRV883 SLEEP pin
const uint16_t MOTOR_FL_1 = 32; // FL Motor IN1
const uint16_t MOTOR_FL_2 = 33; // FL Motor IN2
const uint16_t MOTOR_RL_1 = 26; // RL Motor IN1
const uint16_t MOTOR_RL_2 = 27; // RL Motor IN2

// Car's dimensions in cm
const double CARLENGTH = 25.6;
const double CARWIDTH = 15.3;
const double WHEELTRACK = 12.8; // Distance between the center of the wheels on the same axle
const double WHEELBASE = 11.5;  // Distance between the front and rear axles

// Encoders' pins
const uint16_t ENC_FR = 34;
const uint16_t ENC_FL = 35;
const uint16_t ENC_RR = 36;
const uint16_t ENC_RL = 39;

// Number of slots of the speed encoder
const int ENC_SLOTS = 20;
// Wheels' diameter in millimeters
const float WHEEL_DIAMETER = 66.00;

// Setting PWM properties
const uint16_t FREQ = 50000;         // change to uint32_t for frequencies higher than 65535
const uint16_t PWM_CHANNEL_FR_1 = 0; // PWM channel front right IN1
const uint16_t PWM_CHANNEL_FR_2 = 1; // PWM channel front right IN2
const uint16_t PWM_CHANNEL_RR_1 = 2; // PWM channel rear right IN1
const uint16_t PWM_CHANNEL_RR_2 = 3; // PWM channel rear right IN2
const uint16_t PWM_CHANNEL_FL_1 = 4; // PWM channel front left IN1
const uint16_t PWM_CHANNEL_FL_2 = 5; // PWM channel front left IN2
const uint16_t PWM_CHANNEL_RL_1 = 6; // PWM channel rear left IN1
const uint16_t PWM_CHANNEL_RL_2 = 7; // PWM channel rear left IN2
const uint16_t RESOLUTION = 8;       // 0 to 255
const uint16_t MAX_PWM = 255;
const uint16_t MIN_PWM = 170; // Make sure the motor turns

// Motor timing
unsigned long nowTime = 0; // updated on every loop

// PID
const unsigned long SAMPLE_TIME = 100;      // time between PID updates
double setpointFR = 110;                    // setpoint is rpm
double inputFR = 0;                         // input is motor's RPM
double outputFR = 0;                        // output is PWM calculated by PID
double setpointRR = 110;                    // setpoint is rpm
double inputRR = 0;                         // input is motor's RPM
double outputRR = 0;                        // output is PWM calculated by PID
double setpointFL = 110;                    // setpoint is rpm
double inputFL = 0;                         // input is motor's RPM
double outputFL = 0;                        // output is PWM calculated by PID
double setpointRL = 110;                    // setpoint is rpm
double inputRL = 0;                         // input is motor's RPM
double outputRL = 0;                        // output is PWM calculated by PID
double aggKp = 4, aggKi = 0.2, aggKd = 0;   // Aggressive Tuning Parameters
double conKp = 0.2, conKi = 0.2, conKd = 0; // Conservative Tuning Parameters
PID pidMFR(&inputFR, &outputFR, &setpointFR, conKp, conKi, conKd, DIRECT);
PID pidMRR(&inputRR, &outputRR, &setpointRR, conKp, conKi, conKd, DIRECT);
PID pidMFL(&inputFL, &outputFL, &setpointFL, conKp, conKi, conKd, DIRECT);
PID pidMRL(&inputRL, &outputRL, &setpointRL, conKp, conKi, conKd, DIRECT);

// Instantiate car parts
Wheel wheel(WHEEL_DIAMETER);
Encoder encoderFR(ENC_SLOTS);
Encoder encoderRR(ENC_SLOTS);
Encoder encoderFL(ENC_SLOTS);
Encoder encoderRL(ENC_SLOTS);
Motor motorFR(PWM_CHANNEL_FR_1, PWM_CHANNEL_FR_2, encoderFR);
Motor motorRR(PWM_CHANNEL_RR_1, PWM_CHANNEL_RR_2, encoderRR);
Motor motorFL(PWM_CHANNEL_FL_1, PWM_CHANNEL_FL_2, encoderFL);
Motor motorRL(PWM_CHANNEL_RL_1, PWM_CHANNEL_RL_2, encoderRL);
Car car(wheel, CARLENGTH, CARWIDTH, WHEELTRACK, WHEELBASE, motorFR, motorRR, motorFL, motorRL);

// For debugging purpose only
int rpmFR = 0;
int rpmRR = 0;
int rpmFL = 0;
int rpmRL = 0;

void initPins()
{
  // Output pins
  pinMode(LEFT_M_EN, OUTPUT);
  pinMode(RIGHT_M_EN, OUTPUT);
  pinMode(MOTOR_FR_1, OUTPUT);
  pinMode(MOTOR_FR_2, OUTPUT);
  pinMode(MOTOR_RR_1, OUTPUT);
  pinMode(MOTOR_RR_2, OUTPUT);
  pinMode(MOTOR_FL_1, OUTPUT);
  pinMode(MOTOR_FL_2, OUTPUT);
  pinMode(MOTOR_RL_1, OUTPUT);
  pinMode(MOTOR_RL_2, OUTPUT);

  // Input pins
  // Encoders
  pinMode(ENC_FR, INPUT);
  pinMode(ENC_RR, INPUT);
  pinMode(ENC_FL, INPUT);
  pinMode(ENC_RL, INPUT);
}

void initPID()
{
  pidMFR.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRR.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMFL.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRL.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMFR.SetSampleTime(SAMPLE_TIME);
  pidMRR.SetSampleTime(SAMPLE_TIME);
  pidMFL.SetSampleTime(SAMPLE_TIME);
  pidMRL.SetSampleTime(SAMPLE_TIME);
  pidMFR.SetMode(AUTOMATIC);
  pidMRR.SetMode(AUTOMATIC);
  pidMFL.SetMode(AUTOMATIC);
  pidMRL.SetMode(AUTOMATIC);
}

// ISR called by the front right wheel encoder
void ISR_FR()
{
  double newInput = car.getMotorFR().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputFR = newInput;
    rpmFR = newInput;
  }
}

// ISR called by the rear right wheel encoder
void ISR_RR()
{
  double newInput = car.getMotorRR().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputRR = newInput;
    rpmRR = newInput;
  }
}

// ISR called by the front left wheel encoder
void ISR_FL()
{
  double newInput = car.getMotorFL().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputFL = newInput;
    rpmFL = newInput;
  }
}

// ISR called by the rear left wheel encoder
void ISR_RL()
{
  double newInput = car.getMotorRL().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputRL = newInput;
    rpmRL = newInput;
  }
}

void initPWM()
{
  // configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL_FR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_2, FREQ, RESOLUTION);

  // attach the channel to the GPIOs to be controlled
  // one channel can be attached to any number of pins if they need the same settings
  ledcAttachPin(MOTOR_FR_1, PWM_CHANNEL_FR_1);
  ledcAttachPin(MOTOR_FR_2, PWM_CHANNEL_FR_2);
  ledcAttachPin(MOTOR_RR_1, PWM_CHANNEL_RR_1);
  ledcAttachPin(MOTOR_RR_2, PWM_CHANNEL_RR_2);
  ledcAttachPin(MOTOR_FL_1, PWM_CHANNEL_FL_1);
  ledcAttachPin(MOTOR_FL_2, PWM_CHANNEL_FL_2);
  ledcAttachPin(MOTOR_RL_1, PWM_CHANNEL_RL_1);
  ledcAttachPin(MOTOR_RL_2, PWM_CHANNEL_RL_2);
}

void initInterrupts()
{
  // attach interrupts to the sensors pins
  attachInterrupt(digitalPinToInterrupt(ENC_FR), ISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR), ISR_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL), ISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL), ISR_RL, RISING);
}

void setup()
{
  initPins();
  initPWM();
  initPID();
  initInterrupts();

  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial to start
  }

  // testing
  Serial.println("Testing DC Motors...");
}

void loop()
{
  nowTime = millis();

  pidMFR.Compute();
  pidMRR.Compute();
  pidMFL.Compute();
  pidMRL.Compute();
  car.getMotorFR().forward((int)outputFR);
  car.getMotorRR().forward((int)outputRR);
  car.getMotorFL().forward((int)outputFL);
  car.getMotorRL().forward((int)outputRL);

  // debug
  if (rpmRR != 0)
  {
    Serial.print("FR: ");
    Serial.print(rpmFR);
    Serial.print(" - RR: ");
    Serial.println(rpmRR);
    Serial.print("FL: ");
    Serial.print(rpmFL);
    Serial.print(" - RL: ");
    Serial.println(rpmRL);
    // reset the RPM
    rpmFR = 0;
    rpmRR = 0;
    rpmFL = 0;
    rpmRL = 0;
  }
}
