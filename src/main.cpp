#include <Arduino.h>
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
 * at the moment it is not necessary to disable them separately
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
const uint8_t MIN_PWM = 80; // Make sure the motors run

// Motor timing updated on every loop
unsigned long nowTime = 0; // Current timestamp

// Debugging variables ------------------------------------------------------
unsigned long startTime = millis();
unsigned long elapsedTime = 0;
unsigned long debugResTime = 0;
//---------------------------------------------------------------------------

int16_t volatile inputFR = 0; // FR Motor's RPM
uint16_t outputFR = 200;      // PWM calculated by PID for FR Motor
int16_t volatile inputFL = 0; // FL Motor's RPM
uint16_t outputFL = 150;      // PWM calculated by PID for FL Motor
int16_t volatile inputRR = 0; // RR Motor's RPM
uint16_t outputRR = 200;      // PWM calculated by PID for RR Motor
int16_t volatile inputRL = 0; // RL Motor's RPM
uint16_t outputRL = 150;      // PWM calculated by PID for RL Motor

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS};                                                                            // FR Encoder
Encoder encoderFL{ENC_SLOTS};                                                                            // FL Encoder
Encoder encoderRR{ENC_SLOTS};                                                                            // RR Encoder
Encoder encoderRL{ENC_SLOTS};                                                                            // RL Encoder
Motor motorFR{PWM_CHANNEL_FR_1, PWM_CHANNEL_FR_2, encoderFR};                                            // FR Motor
Motor motorFL{PWM_CHANNEL_FL_1, PWM_CHANNEL_FL_2, encoderFL};                                            // FL Motor
Motor motorRR{PWM_CHANNEL_RR_1, PWM_CHANNEL_RR_2, encoderRR};                                            // RR Motor
Motor motorRL{PWM_CHANNEL_RL_1, PWM_CHANNEL_RL_2, encoderRL};                                            // RL Motor
Car car{CARLENGTH, CARWIDTH, WHEELTRACK, WHEELBASE, WHEEL_DIAMETER, motorFR, motorFL, motorRR, motorRL}; // The car

/**
 * ISR called by the front right wheel encoder
 */
void IRAM_ATTR ISR_FR()
{
  // Current RPM value for the FR motor
  int16_t newInput = car.getMotorFR().getEncoder().isr(nowTime);

  if (newInput != -1)
  {
    inputFR = newInput;
  }
}

/**
 * ISR called by the front left wheel encoder
 */
void IRAM_ATTR ISR_FL()
{
  // Current RPM value for the FL motor
  int16_t newInput = car.getMotorFL().getEncoder().isr(nowTime);

  if (newInput != -1)
  {
    inputFL = newInput;
  }
}

/**
 * ISR called by the rear right wheel encoder
 */
void IRAM_ATTR ISR_RR()
{
  // Current RPM value for the RR motor
  int16_t newInput = car.getMotorRR().getEncoder().isr(nowTime);

  if (newInput != -1)
  {
    inputRR = newInput;
  }
}

/**
 * ISR called by the rear left wheel encoder
 */
void IRAM_ATTR ISR_RL()
{
  // Current RPM value for the RL motor
  int16_t newInput = car.getMotorRL().getEncoder().isr(nowTime);

  if (newInput != -1)
  {
    inputRL = newInput;
  }
}

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
}

/**
 * Initialize the PWM pins
 */
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

/**
 * Initialize the interrupt pins
 */
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
  initInterrupts();

  // Enable the motor driver
  digitalWrite(MOTORS_EN, HIGH);

  Serial.begin(115200);

  while (!Serial)
  {
    ; // Wait for serial to start
  }

  // Testing
  Serial.println("Testing DC Motors...");
}

// Debug function
void printRpm()
{
  Serial.println("----------------------------");
  Serial.println(elapsedTime);
  Serial.print("FR RPM: ");
  Serial.println(inputFR);
  Serial.print("FL RPM: ");
  Serial.println(inputFL);
  Serial.print("RR RPM: ");
  Serial.println(inputRR);
  Serial.print("RL RPM: ");
  Serial.println(inputRL);
  Serial.println("----------------------------");
  Serial.println();
}

void loop()
{
  // Get the current timestamp
  nowTime = millis();
  elapsedTime = nowTime - startTime;

  // Move the car forward using the just calculcated PWM values
  car.forward((int)outputFR, (int)outputFL, (int)outputRR, (int)outputRL);

  // Debug
  // Print the motors' rpm each half a second
  if (nowTime - debugResTime > 500)
  {
    printRpm();

    debugResTime = nowTime;
  }
}
