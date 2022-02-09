#include <Arduino.h>
#include <config.h>
#include <WiFi.h>
#include <../.pio/libdeps/esp32dev/PubSubClient/src/PubSubClient.h>
#include <string>
#include "car/car.h"
#include "motor/motor.h"
#include "encoder/encoder.h"

// Debugging variables ------------------------------------------
unsigned long startTime = millis();
unsigned long nowTime = 0; // Current timestamp
unsigned long elapsedTime = 0;
unsigned long debugResTime = 0;
//---------------------------------------------------------------

// The car should move or not
bool move = false;
// Minimum value of the deviation to tell the car to turn
const double min_dev_to_turn = 0.03;

WiFiClient wifiClient;               // Client WiFi
PubSubClient mqttClient{wifiClient}; // Client MQTT

int16_t volatile rpmFR = 0; // Front Right Motor's RPM
int16_t volatile rpmFL = 0; // Front Left Motor's RPM
int16_t volatile rpmRR = 0; // Rear Right Motor's RPM
int16_t volatile rpmRL = 0; // Rear Left Motor's RPM

// Initial PWM values
uint16_t pwmFR = REF_PWM; // Front Right Motor's PWM
uint16_t pwmFL = REF_PWM; // Front Left Motor's PWM
uint16_t pwmRR = REF_PWM; // Rear Right Motor's PWM
uint16_t pwmRL = REF_PWM; // Rear Left Motor's PWM

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS};                       // Front Right Encoder
Encoder encoderFL{ENC_SLOTS};                       // Front Left Encoder
Encoder encoderRR{ENC_SLOTS};                       // Rear Right Encoder
Encoder encoderRL{ENC_SLOTS};                       // Rear Left Encoder
Motor motorFR{PWM_CH_FR_1, PWM_CH_FR_2, encoderFR}; // Front Right Motor
Motor motorFL{PWM_CH_FL_1, PWM_CH_FL_2, encoderFL}; // Front Left Motor
Motor motorRR{PWM_CH_RR_1, PWM_CH_RR_2, encoderRR}; // Rear Right Motor
Motor motorRL{PWM_CH_RL_1, PWM_CH_RL_2, encoderRL}; // Rear Left Motor
Car car{CARLENGTH, CARWIDTH,
        WHEELTRACK, WHEELBASE, WHEELDIAMETER,
        MAX_PWM, MIN_PWM, TURN_MULT,
        motorFR, motorFL, motorRR, motorRL}; // The car

/**
 * @brief ISR called by the front right wheel encoder
 * 
 */
void IRAM_ATTR ISR_FR()
{
  // Current RPM value for the Front Right Motor
  int16_t newInput = car.getMotorFR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFR = newInput;
}

/**
 * @brief ISR called by the front left wheel encoder
 * 
 */
void IRAM_ATTR ISR_FL()
{
  // Current RPM value for the Front Left motor
  int16_t newInput = car.getMotorFL().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFL = newInput;
}

/**
 * @brief ISR called by the rear right wheel encoder
 * 
 */
void IRAM_ATTR ISR_RR()
{
  // Current RPM value for the Rear Right motor
  int16_t newInput = car.getMotorRR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmRR = newInput;
}

/**
 * @brief ISR called by the rear left wheel encoder
 * 
 */
void IRAM_ATTR ISR_RL()
{
  // Current RPM value for the Rear Left motor
  int16_t newInput = car.getMotorRL().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmRL = newInput;
}

/**
 * @brief Initialize I/O Pins
 * 
 */
void setupPins()
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
 * @brief Initialize the PWM pins
 * 
 */
void setupPWM()
{
  // Configure LED PWM functionalitites
  ledcSetup(PWM_CH_FR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_FR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_FL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_FL_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_RR_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_RR_2, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_RL_1, FREQ, RESOLUTION);
  ledcSetup(PWM_CH_RL_2, FREQ, RESOLUTION);

  /*
   * Attach the channel to the GPIOs to be controlled 
   * one channel can be attached to any number of pins if they need the same settings
   */
  ledcAttachPin(MOTOR_FR_1, PWM_CH_FR_1);
  ledcAttachPin(MOTOR_FR_2, PWM_CH_FR_2);
  ledcAttachPin(MOTOR_FL_1, PWM_CH_FL_1);
  ledcAttachPin(MOTOR_FL_2, PWM_CH_FL_2);
  ledcAttachPin(MOTOR_RR_1, PWM_CH_RR_1);
  ledcAttachPin(MOTOR_RR_2, PWM_CH_RR_2);
  ledcAttachPin(MOTOR_RL_1, PWM_CH_RL_1);
  ledcAttachPin(MOTOR_RL_2, PWM_CH_RL_2);
}

/**
 * @brief Initialize the interrupt pins
 * 
 */
void setupInterrupts()
{
  // Attach interrupts to the sensors pins
  attachInterrupt(ENC_FR, ISR_FR, RISING);
  attachInterrupt(ENC_FL, ISR_FL, RISING);
  attachInterrupt(ENC_RR, ISR_RR, RISING);
  attachInterrupt(ENC_RL, ISR_RL, RISING);
}

/**
 * @brief Connect to the WiFi Network
 * 
 */
void setupWifi()
{
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFISSID);

  WiFi.begin(WIFISSID, WIFIPASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief Reconnect to MQTT server on connection loss
 * 
 */
void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(MQTTCLIENTID))
    {
      Serial.println("connected");

      mqttClient.subscribe("Mode");
      Serial.println("Mode topic subscribed");
      mqttClient.subscribe("Deviation");
      Serial.println("Deviation topic subscribed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Debug function
void printRpm()
{
  double speed;
  uint16_t avgrpm;
  avgrpm = (int)((rpmFR + rpmFL + rpmRR + rpmRL) / 4);
  speed = car.rpmToMs(avgrpm);
  
  Serial.println("----------------------------");
  Serial.println(elapsedTime);
  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.println(" m/s");
  Serial.print("FR RPM: ");
  Serial.println(rpmFR);
  Serial.print("FL RPM: ");
  Serial.println(rpmFL);
  Serial.print("RR RPM: ");
  Serial.println(rpmRR);
  Serial.print("RL RPM: ");
  Serial.println(rpmRL);
  Serial.println("----------------------------");
  Serial.println();
}

/**
 * @brief Callback function called upon MQTT message received
 * 
 * @param topic char* - The topic where the message was published
 * @param message byte* - The message received
 * @param length unsigned int - The length of the message
 */
void callback(char *topic, byte *message, unsigned int length)
{
  String mString; // Message as a string
  double mValue;  // Message as a number

  // Message as string
  for (int i = 0; i < length; i++)
  {
    mString += (char)message[i];
  }

  if (String(topic) == "Mode" && mString == "s")
  {
    // Stop the car
    car.brake();
    move = false;
  }
  else if (String(topic) == "Mode" && mString == "d")
  {
    // Moving is now allowed
    move = true;

    // Initial car start: straight at the reference speed
    pwmFR = pwmFL = pwmRR = pwmRL = REF_PWM;
    car.forward(pwmFR, pwmFL, pwmRR, pwmRL);
  }
  else if (String(topic) == "Deviation")
  {
    // Deviation value is a double, convert the received message
    mValue = (double)atof((char*) message); // mString.toDouble();

    // If the absolute value of the deviation is between 0 and 1 and the car can move
    if (abs(mValue) >= 0 && abs(mValue) <= 1 && move)
    {
      Serial.print("Deviation: ");
      Serial.println(mValue);
      /*
      * If the value is less than the threshold
      */
      if (abs(mValue) <= min_dev_to_turn)
      {
        /*
        * Reset the speed and go straight.
        */
        pwmFR = pwmFL = pwmRR = pwmRL = REF_PWM;
        car.forward(pwmFR, pwmFL, pwmRR, pwmRL);
      }
      // If the value is positive
      else if (mValue > min_dev_to_turn)
      {
        // Turn right
        car.turnRight(mValue);
      }
      // If the value is negative
      else if (mValue < -min_dev_to_turn)
      {
        // Turn left
        car.turnLeft(mValue);
      }
    }

    // TODO: this is only meant as debugging, to be removed
    // printRpm();
  }

  /*
  Serial.println();
  Serial.print("Received topic: ");
  Serial.println(topic);
  Serial.println("mString: " + mString);
  Serial.print("mValue: ");
  Serial.println(mValue);
  */
}

/**
 * @brief Set up the MQTT server
 * 
 */
void setupMqtt()
{
  mqttClient.setServer(MQTTSERVER, MQTTPORT);
  mqttClient.setCallback(callback);
}

void setup()
{
  Serial.begin(115200);
  setupPins();
  setupPWM();
  setupInterrupts();
  setupWifi();
  setupMqtt();

  // Enable the motor driver
  digitalWrite(MOTORS_EN, HIGH);
}


void loop()
{
  // If the client is not connected to the MQTT server
  if (!mqttClient.connected())
  {
    // Stop the car
    car.brake();
    // Connect to the MQTT server
    reconnect();
  }

  if (mqttClient.connected())
  {
    // Call the MQTT client loop
    mqttClient.loop();
  }
  
  // Get the current timestamp
  nowTime = millis();
}
