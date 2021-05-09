#include <Arduino.h>
#include <pid/pid.h>
#include <car/car.h>
#include <motor/motor.h>
#include <encoder/encoder.h>

// Front Motors
const uint8_t MOTOR_FR_1 = 18; // FR Motor IN1
const uint8_t MOTOR_FR_2 = 19; // FR Motor IN2
const uint8_t MOTOR_FL_1 = 32; // FL Motor IN1
const uint8_t MOTOR_FL_2 = 33; // FL Motor IN2

// Rear Motors
const uint8_t MOTOR_RR_1 = 16; // RR Motor IN1
const uint8_t MOTOR_RR_2 = 17; // RR Motor IN2
const uint8_t MOTOR_RL_1 = 26; // RL Motor IN1
const uint8_t MOTOR_RL_2 = 27; // RL Motor IN2

// DRV8833 SLEEP pin
// Both the drivers are connected to the same pin,
// at the moment it is not necessary to disable them separatedly
const uint8_t MOTORS_EN = 5;

// Car's dimensions in mm
const uint16_t CARLENGTH = 256;
const uint16_t CARWIDTH = 153;
const uint16_t WHEELTRACK = 128;   // Distance between the center of the wheels on the same axle
const uint16_t WHEELBASE = 115;    // Distance between the front and rear axles
const uint8_t WHEEL_DIAMETER = 66; // Wheels' diameter

// Encoders' pins
const uint8_t ENC_FR = 36; // SP
const uint8_t ENC_FL = 39; // SN
const uint8_t ENC_RR = 34;
const uint8_t ENC_RL = 35;

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
const uint8_t MIN_PWM = 100; // Make sure the motors run

// Motor timing
unsigned long nowTime = 0; // updated on every loop
unsigned long resTime = 0; // Debug only

// PID
const unsigned long SAMPLE_TIME = 100; // time between PID updates in ms
int setpointFR = 110;                  // setpoint is rpm
int volatile inputFR = 0;              // input is motor's RPM
int outputFR = 0;                      // output is PWM calculated by PID
int setpointFL = 110;                  // setpoint is rpm
int volatile inputFL = 0;              // input is motor's RPM
int outputFL = 0;                      // output is PWM calculated by PID
int setpointRR = 110;                  // setpoint is rpm
int volatile inputRR = 0;              // input is motor's RPM
int outputRR = 0;                      // output is PWM calculated by PID
int setpointRL = 110;                  // setpoint is rpm
int volatile inputRL = 0;              // input is motor's RPM
int outputRL = 0;                      // output is PWM calculated by PID
double kp = 0.2, ki = 0.2, kd = 0;     // Tuning Parameters
PID pidMFR{&inputFR, &outputFR, &setpointFR, kp, ki, kd, DIRECT};
PID pidMFL{&inputFL, &outputFL, &setpointFL, kp, ki, kd, DIRECT};
PID pidMRR{&inputRR, &outputRR, &setpointRR, kp, ki, kd, DIRECT};
PID pidMRL{&inputRL, &outputRL, &setpointRL, kp, ki, kd, DIRECT};

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS};
Encoder encoderFL{ENC_SLOTS};
Encoder encoderRR{ENC_SLOTS};
Encoder encoderRL{ENC_SLOTS};
Motor motorFR{PWM_CHANNEL_FR_1, PWM_CHANNEL_FR_2, &encoderFR};
Motor motorFL{PWM_CHANNEL_FL_1, PWM_CHANNEL_FL_2, &encoderFL};
Motor motorRR{PWM_CHANNEL_RR_1, PWM_CHANNEL_RR_2, &encoderRR};
Motor motorRL{PWM_CHANNEL_RL_1, PWM_CHANNEL_RL_2, &encoderRL};
Car car{CARLENGTH, CARWIDTH, WHEELTRACK, WHEELBASE, WHEEL_DIAMETER, &motorFR, &motorFL, &motorRR, &motorRL};

void initPins()
{
  // Output pins
  pinMode(MOTORS_EN, OUTPUT);
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
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_RR, INPUT_PULLUP);
  pinMode(ENC_RL, INPUT_PULLUP);

  // Enable the motor drivers
  digitalWrite(MOTORS_EN, HIGH);
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
void IRAM_ATTR ISR_FR()
{
  double newInput = car.getMotorFR().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputFR = newInput;
  }
}

// ISR called by the front left wheel encoder
void IRAM_ATTR ISR_FL()
{
  double newInput = car.getMotorFL().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputFL = newInput;
  }
}

// ISR called by the rear right wheel encoder
void IRAM_ATTR ISR_RR()
{
  double newInput = car.getMotorRR().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputRR = newInput;
  }
}

// ISR called by the rear left wheel encoder
void IRAM_ATTR ISR_RL()
{
  double newInput = car.getMotorRL().getEncoder().isr(nowTime);
  if (newInput != -1)
  {
    inputRL = newInput;
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
  attachInterrupt(ENC_FR, ISR_FR, RISING);
  attachInterrupt(ENC_FL, ISR_FL, RISING);
  attachInterrupt(ENC_RR, ISR_RR, RISING);
  attachInterrupt(ENC_RL, ISR_RL, RISING);
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
  // Print the motors' rpm each half a second
  if (nowTime - resTime > 500)
  {
    Serial.print("FR: ");
    Serial.print(inputFR);
    Serial.print(" - FL: ");
    Serial.println(inputFL);
    Serial.print("RR: ");
    Serial.print(inputRR);
    Serial.print(" - RL: ");
    Serial.println(inputRL);

    resTime = nowTime;
  }
}
