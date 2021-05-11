#include <Arduino.h>
#include <pid/pid.h>
#include <car/car.h>
#include <motor/motor.h>
#include <encoder/encoder.h>

// Front Motors
const uint8_t MOTOR_FR_1 = 18; // FR Motor IN1 pin
const uint8_t MOTOR_FR_2 = 19; // FR Motor IN2 pin
const uint8_t MOTOR_FL_1 = 32; // FL Motor IN1 pin
const uint8_t MOTOR_FL_2 = 33; // FL Motor IN2 pin

// Rear Motors
const uint8_t MOTOR_RR_1 = 16; // RR Motor IN1 pin
const uint8_t MOTOR_RR_2 = 17; // RR Motor IN2 pin
const uint8_t MOTOR_RL_1 = 26; // RL Motor IN1 pin
const uint8_t MOTOR_RL_2 = 27; // RL Motor IN2 pin

/** 
 * DRV8833 SLEEP pin
 * Both the drivers are connected to the same pin,
 * at the moment it is not necessary to disable them separatedly
 */
const uint8_t MOTORS_EN = 5;

// Car's dimensions in mm
const uint16_t CARLENGTH = 256;    // Car's full length (mm)
const uint16_t CARWIDTH = 153;     // Car's full width (mm)
const uint16_t WHEELTRACK = 128;   // Distance between the center of the wheels on the same axle (mm)
const uint16_t WHEELBASE = 115;    // Distance between the front and rear axles (mm)
const uint8_t WHEEL_DIAMETER = 66; // Wheels' diameter (mm)

// Encoders' pins
const uint8_t ENC_FR = 36; // Front Right Encoder, pin SP
const uint8_t ENC_FL = 39; // Front Left Encoder, pin SN
const uint8_t ENC_RR = 34; // Rear Right Encoder
const uint8_t ENC_RL = 35; // Rear Left Encoder

// Number of slots of the speed encoder
const uint8_t ENC_SLOTS = 20;

// Setting PWM properties
const uint16_t FREQ = 50000;        // Change to uint32_t for frequencies higher than 65535
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
const uint8_t MIN_PWM = 90; // Make sure the motors run

// Motor timing updated on every loop
unsigned long nowTime = 0; // Current timestamp

// Debugging variables ------------------------------------------------------

unsigned long debugResTime = 0;
int debugRpmFR = 0, debugDiffTimeFR = 0, debugIntCountFR = 0, debugIpsFR = 0;
int debugRpmFL = 0, debugDiffTimeFL = 0, debugIntCountFL = 0, debugIpsFL = 0;
int debugRpmRR = 0, debugDiffTimeRR = 0, debugIntCountRR = 0, debugIpsRR = 0;
int debugRpmRL = 0, debugDiffTimeRL = 0, debugIntCountRL = 0, debugIpsRL = 0;
//---------------------------------------------------------------------------

// PID
const unsigned long SAMPLE_TIME = 100; // Time between PID updates in ms
int setpointFR = 110;                  // Wanted RPM for FR Motor
int volatile inputFR = 0;              // FR Motor's RPM
int outputFR = 0;                      // PWM calculated by PID for FR Motor
int setpointFL = 110;                  // Wanted RPM for FL Motor
int volatile inputFL = 0;              // FL Motor's RPM
int outputFL = 0;                      // PWM calculated by PID for FL Motor
int setpointRR = 110;                  // Wanted RPM for RR Motor
int volatile inputRR = 0;              // RR Motor's RPM
int outputRR = 0;                      // PWM calculated by PID for RR Motor
int setpointRL = 110;                  // Wanted RPM for RL Motor
int volatile inputRL = 0;              // RL Motor's RPM
int outputRL = 0;                      // PWM calculated by PID for RL Motor
double kpa = 1.0, kia = 0.5, kda = 0;  // Aggressive Tuning Parameters
double kpc = 0.1, kic = 0.1, kdc = 0;  // Condervative Tuning Parameters
PID pidMFR{&inputFR, &outputFR, &setpointFR, kpc, kic, kdc, DIRECT}; // PID for FR Motor
PID pidMFL{&inputFL, &outputFL, &setpointFL, kpc, kic, kdc, DIRECT}; // PID for FL Motor
PID pidMRR{&inputRR, &outputRR, &setpointRR, kpc, kic, kdc, DIRECT}; // PID for RR Motor
PID pidMRL{&inputRL, &outputRL, &setpointRL, kpc, kic, kdc, DIRECT}; // PID for RL Motor

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS}; // FR Encoder
Encoder encoderFL{ENC_SLOTS}; // FL Encoder
Encoder encoderRR{ENC_SLOTS}; // RR Encoder
Encoder encoderRL{ENC_SLOTS}; // RL Encoder
Motor motorFR{PWM_CHANNEL_FR_1, PWM_CHANNEL_FR_2, encoderFR}; // FR Motor
Motor motorFL{PWM_CHANNEL_FL_1, PWM_CHANNEL_FL_2, encoderFL}; // FL Motor
Motor motorRR{PWM_CHANNEL_RR_1, PWM_CHANNEL_RR_2, encoderRR}; // RR Motor
Motor motorRL{PWM_CHANNEL_RL_1, PWM_CHANNEL_RL_2, encoderRL}; // RL Motor
Car car{CARLENGTH, CARWIDTH, WHEELTRACK, WHEELBASE, WHEEL_DIAMETER, motorFR, motorFL, motorRR, motorRL}; // The car

/**
 * Initialize I/O Pins
 */
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
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_RR, INPUT_PULLUP);
  pinMode(ENC_RL, INPUT_PULLUP);

  // Enable the motor drivers
  digitalWrite(MOTORS_EN, HIGH);
}

/**
 * Initialize PID
 */
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

/**
 * ISR called by the front right wheel encoder
 */
void IRAM_ATTR ISR_FR()
{
  // Current RPM value for the FR motor
  uint8_t newInput = car.getMotorFR().getEncoder().isr(nowTime);
  
  if (newInput != -1)
  {
    inputFR = newInput;
  }
}

// ISR called by the front left wheel encoder
void IRAM_ATTR ISR_FL()
{
  // Current RPM value for the FL motor
  uint8_t newInput = car.getMotorFL().getEncoder().isr(nowTime);
  
  if (newInput != -1)
  {
    inputFL = newInput;
  }
}

// ISR called by the rear right wheel encoder
void IRAM_ATTR ISR_RR()
{
  // Current RPM value for the RR motor
  uint8_t newInput = car.getMotorRR().getEncoder().isr(nowTime);
  
  if (newInput != -1)
  {
    inputRR = newInput;
  }
}

// ISR called by the rear left wheel encoder
void IRAM_ATTR ISR_RL()
{
  // Current RPM value for the RL motor
  uint8_t newInput = car.getMotorRL().getEncoder().isr(nowTime);
  
  if (newInput != -1)
  {
    inputRL = newInput;
  }
}

void initPWM()
{
  // Configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL_FR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL_2, FREQ, RESOLUTION);

  /*
   * Attach the channel to the GPIOs to be controlled
   * one channel can be attached to any number of pins if they need the same settings
   */
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
  // Attach interrupts to the sensors pins
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
    ; // Wait for serial to start
  }

  // Testing
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

  // Use aggressive tuning parameters if the current value is too far from the target
  // Use conservative tuning parameters if the current value is close to the target
  if (abs(inputFR - setpointFR) > 5)
    pidMFR.SetTunings(kpa, kia, kda);
  else
    pidMFR.SetTunings(kpc, kic, kdc);

  if (abs(inputFL - setpointFL) > 5)
    pidMFL.SetTunings(kpa, kia, kda);
  else
    pidMFL.SetTunings(kpc, kic, kdc);

  if (abs(inputRR - setpointRR) > 5)
    pidMRR.SetTunings(kpa, kia, kda);
  else
    pidMRR.SetTunings(kpc, kic, kdc);

  if (abs(inputRL - setpointRL) > 5)
    pidMRL.SetTunings(kpa, kia, kda);
  else
    pidMRL.SetTunings(kpc, kic, kdc);

  // Debug
  // Print the motors' rpm each half a second
  if (nowTime - debugResTime > 500)
  {
    Serial.println("----------------------------");
    Serial.println("Front Right:");
    Serial.print("- RPM: ");
    Serial.println(debugRpmFR);
    Serial.print("- IntCount: ");
    Serial.println(debugIntCountFR);
    Serial.print("- DiffTime: ");
    Serial.println(debugDiffTimeFR);
    Serial.print("- IPS: ");
    Serial.println(debugIpsFR);
    Serial.println();
    Serial.println("Front Left:");
    Serial.print("- RPM: ");
    Serial.println(debugRpmFL);
    Serial.print("- IntCount: ");
    Serial.println(debugIntCountFL);
    Serial.print("- DiffTime: ");
    Serial.println(debugDiffTimeFL);
    Serial.print("- IPS: ");
    Serial.println(debugIpsFL);
    Serial.println();
    Serial.println("Rear Right:");
    Serial.print("- RPM: ");
    Serial.println(debugRpmRR);
    Serial.print("- IntCount: ");
    Serial.println(debugIntCountRR);
    Serial.print("- DiffTime: ");
    Serial.println(debugDiffTimeRR);
    Serial.print("- IPS: ");
    Serial.println(debugIpsRR);
    Serial.println();
    Serial.println("Rear Left:");
    Serial.print("- RPM: ");
    Serial.println(debugRpmRL);
    Serial.print("- IntCount: ");
    Serial.println(debugIntCountRL);
    Serial.print("- DiffTime: ");
    Serial.println(debugDiffTimeRL);
    Serial.print("- IPS: ");
    Serial.println(debugIpsRL);
    Serial.println();
    Serial.println("----------------------------");
    Serial.println();

    debugResTime = nowTime;
  }
}
