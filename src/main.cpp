#include <Arduino.h>
#include <pid/pid.h>
#include <car/car.h>
#include <motor/motor.h>
#include <wheel/wheel.h>
#include <encoder/encoder.h>

// Right Motors (at the moment in parallel)
const uint16_t RMPin1 = 27; // IN1
const uint16_t RMPin2 = 26; // IN2
const uint16_t EN1 = 14;    // EN1

// Left Motors (at the moment in parallel)
const uint16_t LMPin1 = 33; // IN3
const uint16_t LMPin2 = 32; // IN4
const uint16_t EN2 = 25;    // EN2

// Car's dimensions in cm
const double CARLENGTH = 25.6;
const double CARWIDTH = 15.3;
const double WHEELTRACK = 12.8;
const double WHEELBASE = 11.5;

// Encoders' pins
const uint16_t ENC_RR = 34;
const uint16_t ENC_RL = 35;

// Number of slots of the speed encoder
const int ENC_SLOTS = 20;
// Wheels' diameter in millimeters
const float WHEEL_DIAMETER = 66.00;

// Setting PWM properties
const uint16_t FREQ = 30000;       // change to uint32_t for frequencies higher than 65535
const uint16_t PWM_CHANNEL_RR = 0; // PWM rear right channel
const uint16_t PWM_CHANNEL_RL = 1; // PWM rear left channel
const uint16_t RESOLUTION = 8;     // 0 to 255
const uint16_t MAX_PWM = pow(2, RESOLUTION) - 1;
const uint16_t MIN_PWM = 175; // Make sure the motor turns

// Motor timing
unsigned long nowTime = 0; // updated on every loop

// PID
const unsigned long SAMPLE_TIME = 100;      // time between PID updates
double setpointRR = 110;                    // setpoint is rpm
double inputRR = 0;                         // input is motor's RPM
double outputRR = 0;                        // output is PWM calculated by PID
double setpointRL = 110;                    // setpoint is rpm
double inputRL = 0;                         // input is motor's RPM
double outputRL = 0;                        // output is PWM calculated by PID
double aggKp = 4, aggKi = 0.2, aggKd = 0;   // Aggressive Tuning Parameters
double conKp = 0.2, conKi = 0.2, conKd = 0; // Conservative Tuning Parameters
PID pidMRR(&inputRR, &outputRR, &setpointRR, conKp, conKi, conKd, DIRECT);
PID pidMRL(&inputRL, &outputRL, &setpointRL, conKp, conKi, conKd, DIRECT);

Wheel wheel(WHEEL_DIAMETER);
Encoder encoderRR(ENC_SLOTS);
Encoder encoderRL(ENC_SLOTS);
Motor motorRR(PWM_CHANNEL_RR, RMPin1, RMPin2, encoderRR);
Motor motorRL(PWM_CHANNEL_RL, LMPin1, LMPin2, encoderRL);
Car car(wheel, CARLENGTH, CARWIDTH, WHEELTRACK, WHEELBASE, motorRR, motorRL);

// Only for debugging purpose
int rpmRR = 0;
int rpmRL = 0;

void initPins()
{
  // Output pins
  pinMode(RMPin1, OUTPUT);
  pinMode(RMPin2, OUTPUT);
  pinMode(EN1, OUTPUT);

  pinMode(LMPin1, OUTPUT);
  pinMode(LMPin2, OUTPUT);
  pinMode(EN2, OUTPUT);

  // Input pins
  // Encoders
  pinMode(ENC_RR, INPUT);
  pinMode(ENC_RL, INPUT);
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
  ledcSetup(PWM_CHANNEL_RR, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL, FREQ, RESOLUTION);

  // attach the channel to the GPIOs to be controlled
  // one channel can be attached to any number of pins if they need the same settings
  ledcAttachPin(EN1, PWM_CHANNEL_RR);
  ledcAttachPin(EN2, PWM_CHANNEL_RL);

  pidMRR.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRL.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRR.SetSampleTime(SAMPLE_TIME);
  pidMRL.SetSampleTime(SAMPLE_TIME);
  pidMRR.SetMode(AUTOMATIC);
  pidMRL.SetMode(AUTOMATIC);
}

void initInterrupts()
{
  // attach interrupts to the sensors pins
  attachInterrupt(digitalPinToInterrupt(ENC_RR), ISR_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL), ISR_RL, RISING);
}

void setup()
{
  initPins();
  initPWM();
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

  pidMRR.Compute();
  pidMRL.Compute();
  car.getMotorRR().forward((int)outputRR);
  car.getMotorRL().forward((int)outputRL);

  // debug
  if (rpmRR != 0)
  {
    Serial.print("RR: ");
    Serial.print(rpmRR);
    Serial.print(" - RL: ");
    Serial.println(rpmRL);
    // reset the RPM
    rpmRR = 0;
    rpmRL = 0;
  }
}
