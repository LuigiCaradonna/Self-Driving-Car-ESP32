#include <Arduino.h>
#include <pid/pid.h>
#include <car/car.h>
#include <motor/motor.h>
#include <encoder/encoder.h>

// Front Motors
const uint8_t FRONT_M_EN = 5;  // Front DRV8833 SLEEP pin
const uint8_t MOTOR_FR_1 = 18; // FR Motor IN1
const uint8_t MOTOR_FR_2 = 19; // FR Motor IN2
const uint8_t MOTOR_FL_1 = 32; // FL Motor IN1
const uint8_t MOTOR_FL_2 = 33; // FL Motor IN2

// Rear Motors
const uint8_t REAR_M_EN = 25;  // Rear DRV8833 SLEEP pin
const uint8_t MOTOR_RR_1 = 16; // RR Motor IN1
const uint8_t MOTOR_RR_2 = 17; // RR Motor IN2
const uint8_t MOTOR_RL_1 = 26; // RL Motor IN1
const uint8_t MOTOR_RL_2 = 27; // RL Motor IN2

// Car's dimensions in mm
const double CARLENGTH = 256.00;
const double CARWIDTH = 153.00;
const double WHEELTRACK = 128.00;    // Distance between the center of the wheels on the same axle
const double WHEELBASE = 115.00;     // Distance between the front and rear axles
const double WHEEL_DIAMETER = 66.00; // Wheels' diameter

// Encoders' pins
const uint8_t ENC_FR = 34;
const uint8_t ENC_FL = 35;
const uint8_t ENC_RR = 36; // sp
const uint8_t ENC_RL = 39; // sn

// Number of slots of the speed encoder
const uint8_t ENC_SLOTS = 20;

// Setting PWM properties
const uint16_t FREQ = 50000;        // change to uint32_t for frequencies higher than 65535
const uint8_t PWM_CHANNEL_FR_1 = 0; // PWM channel front right IN1
const uint8_t PWM_CHANNEL_FR_2 = 1; // PWM channel front right IN2
const uint8_t PWM_CHANNEL_FL_1 = 2; // PWM channel front left IN1
const uint8_t PWM_CHANNEL_FL_2 = 3; // PWM channel front left IN2
const uint8_t PWM_CHANNEL_RR_1 = 4; // PWM channel rear right IN1
const uint8_t PWM_CHANNEL_RR_2 = 5; // PWM channel rear right IN2
const uint8_t PWM_CHANNEL_RL_1 = 6; // PWM channel rear left IN1
const uint8_t PWM_CHANNEL_RL_2 = 7; // PWM channel rear left IN2
const uint8_t RESOLUTION = 8;       // 0 to 255
const uint8_t MAX_PWM = 255;
const uint8_t MIN_PWM = 170; // Make sure the motor turns

// Motor timing
unsigned long nowTime = 0; // updated on every loop

// PID
const unsigned long SAMPLE_TIME = 100;      // time between PID updates in ms
double setpointFR = 110;                    // setpoint is rpm
double inputFR = 0;                         // input is motor's RPM
double outputFR = 0;                        // output is PWM calculated by PID
double setpointFL = 110;                    // setpoint is rpm
double inputFL = 0;                         // input is motor's RPM
double outputFL = 0;                        // output is PWM calculated by PID
double setpointRR = 110;                    // setpoint is rpm
double inputRR = 0;                         // input is motor's RPM
double outputRR = 0;                        // output is PWM calculated by PID
double setpointRL = 110;                    // setpoint is rpm
double inputRL = 0;                         // input is motor's RPM
double outputRL = 0;                        // output is PWM calculated by PID
double aggKp = 4, aggKi = 0.2, aggKd = 0;   // Aggressive Tuning Parameters
double conKp = 0.2, conKi = 0.2, conKd = 0; // Conservative Tuning Parameters
PID pidMFR{&inputFR, &outputFR, &setpointFR, conKp, conKi, conKd, DIRECT};
PID pidMFL{&inputFL, &outputFL, &setpointFL, conKp, conKi, conKd, DIRECT};
PID pidMRR{&inputRR, &outputRR, &setpointRR, conKp, conKi, conKd, DIRECT};
PID pidMRL{&inputRL, &outputRL, &setpointRL, conKp, conKi, conKd, DIRECT};

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS};
Encoder encoderFL{ENC_SLOTS};
Encoder encoderRR{ENC_SLOTS};
Encoder encoderRL{ENC_SLOTS};
Motor motorFR{PWM_CHANNEL_FR_1, PWM_CHANNEL_FR_2, encoderFR};
Motor motorFL{PWM_CHANNEL_FL_1, PWM_CHANNEL_FL_2, encoderFL};
Motor motorRR{PWM_CHANNEL_RR_1, PWM_CHANNEL_RR_2, encoderRR};
Motor motorRL{PWM_CHANNEL_RL_1, PWM_CHANNEL_RL_2, encoderRL};
Car car{CARLENGTH, CARWIDTH, WHEEL_DIAMETER, WHEELTRACK, WHEELBASE, motorFR, motorFL, motorRR, motorRL};

// For debugging purpose only
int rpmFR = 0;
int rpmFL = 0;
int rpmRR = 0;
int rpmRL = 0;

void initPins()
{
  // Output pins
  pinMode(REAR_M_EN, OUTPUT);
  pinMode(FRONT_M_EN, OUTPUT);
  pinMode(MOTOR_FR_1, OUTPUT);
  pinMode(MOTOR_FR_2, OUTPUT);
  pinMode(MOTOR_FL_1, OUTPUT);
  pinMode(MOTOR_FL_2, OUTPUT);
  pinMode(MOTOR_RR_1, OUTPUT);
  pinMode(MOTOR_RR_2, OUTPUT);
  pinMode(MOTOR_RL_1, OUTPUT);
  pinMode(MOTOR_RL_2, OUTPUT);

  // Input pins
  // Encoders
  pinMode(ENC_FR, INPUT);
  pinMode(ENC_FL, INPUT);
  pinMode(ENC_RR, INPUT);
  pinMode(ENC_RL, INPUT);
}

void initPID()
{
  pidMFR.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMFL.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRR.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMRL.SetOutputLimits(MIN_PWM, MAX_PWM);
  pidMFR.SetSampleTime(SAMPLE_TIME);
  pidMFL.SetSampleTime(SAMPLE_TIME);
  pidMRR.SetSampleTime(SAMPLE_TIME);
  pidMRL.SetSampleTime(SAMPLE_TIME);
  pidMFR.SetMode(AUTOMATIC);
  pidMFL.SetMode(AUTOMATIC);
  pidMRR.SetMode(AUTOMATIC);
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
  ledcSetup(PWM_CHANNEL_FR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_2, FREQ, RESOLUTION);

  // attach the channel to the GPIOs to be controlled
  // one channel can be attached to any number of pins if they need the same settings
  ledcAttachPin(MOTOR_FR_1, PWM_CHANNEL_FR_1);
  ledcAttachPin(MOTOR_FR_2, PWM_CHANNEL_FR_2);
  ledcAttachPin(MOTOR_FL_1, PWM_CHANNEL_FL_1);
  ledcAttachPin(MOTOR_FL_2, PWM_CHANNEL_FL_2);
  ledcAttachPin(MOTOR_RR_1, PWM_CHANNEL_RR_1);
  ledcAttachPin(MOTOR_RR_2, PWM_CHANNEL_RR_2);
  ledcAttachPin(MOTOR_RL_1, PWM_CHANNEL_RL_1);
  ledcAttachPin(MOTOR_RL_2, PWM_CHANNEL_RL_2);
}

void initInterrupts()
{
  // attach interrupts to the sensors pins
  attachInterrupt(digitalPinToInterrupt(ENC_FR), ISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL), ISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR), ISR_RR, RISING);
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
  pidMFL.Compute();
  pidMRR.Compute();
  pidMRL.Compute();
  car.forward((int)outputFR, (int)outputFL, (int)outputRR, (int)outputRL);

  // debug
  if (rpmRR != 0)
  {
    Serial.print("FR: ");
    Serial.print(rpmFR);
    Serial.print("FL: ");
    Serial.print(rpmFL);
    Serial.print(" - RR: ");
    Serial.println(rpmRR);
    Serial.print(" - RL: ");
    Serial.println(rpmRL);
    // reset the RPM
    rpmFR = 0;
    rpmFL = 0;
    rpmRR = 0;
    rpmRL = 0;
  }
}
