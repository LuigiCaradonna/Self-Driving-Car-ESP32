#include <Arduino.h>
#include <WiFi.h>
#include <../.pio/libdeps/esp32dev/PubSubClient/src/PubSubClient.h>
#include <car/car.h>
#include <motor/motor.h>
#include <encoder/encoder.h>
#include <string>

// Debugging variables ------------------------------------------
unsigned long startTime = millis();
unsigned long nowTime = 0; // Current timestamp
unsigned long elapsedTime = 0;
unsigned long debugResTime = 0;
//---------------------------------------------------------------

// Connection data
const char *wifiSsid = "ESP32WiFi";                 // WiFi network SSID
const char *wifiPass = "ESP32test";                 // WiFi Password
const char *mqttServer = "http://192.168.4.1:1883"; // MQTT Broker IP address

WiFiClient wifiClient;               // Client WiFi
PubSubClient mqttClient{wifiClient}; // Client MQTT
long lastMsg = 0;
char msg[50];
int value = 0;

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
const uint16_t CARLENGTH = 256;  // Car's full length (mm)
const uint16_t CARWIDTH = 153;   // Car's full width (mm)
const uint16_t WHEELTRACK = 128; // Distance between the center of the wheels on the same axle (mm)
const uint16_t WHEELBASE = 115;  // Distance between the front and rear axles (mm)
const uint8_t WDIAMETER = 66;    // Wheels' diameter (mm)

// Encoders' pins
const uint8_t ENC_FR = 36; // Front Right Encoder, pin SP
const uint8_t ENC_FL = 39; // Front Left Encoder, pin SN
const uint8_t ENC_RR = 34; // Rear Right Encoder
const uint8_t ENC_RL = 35; // Rear Left Encoder

// Number of slots of the speed encoder
const uint8_t ENC_SLOTS = 20;

// Setting PWM properties
const uint16_t FREQ = 50000;   // Change to uint32_t for frequencies higher than 65535
const uint8_t PWM_CH_FR_1 = 0; // PWM channel front right IN1
const uint8_t PWM_CH_FR_2 = 1; // PWM channel front right IN2
const uint8_t PWM_CH_FL_1 = 2; // PWM channel front left IN1
const uint8_t PWM_CH_FL_2 = 3; // PWM channel front left IN2
const uint8_t PWM_CH_RR_1 = 4; // PWM channel rear right IN1
const uint8_t PWM_CH_RR_2 = 5; // PWM channel rear right IN2
const uint8_t PWM_CH_RL_1 = 6; // PWM channel rear left IN1
const uint8_t PWM_CH_RL_2 = 7; // PWM channel rear left IN2
const uint8_t RESOLUTION = 8;  // 8 bit - 0 to 255
const uint8_t MAX_PWM = 255;   // Max PWM duty-cycle value
const uint8_t MIN_PWM = 80;    // Min PWM duty-cycle value, make sure the motors run

int16_t volatile rpmFR = 0; // FR Motor's RPM
int16_t volatile rpmFL = 0; // FL Motor's RPM
int16_t volatile rpmRR = 0; // RR Motor's RPM
int16_t volatile rpmRL = 0; // RL Motor's RPM
uint16_t pwmFR = 255;       // FR Motor's PWM
uint16_t pwmFL = 255;       // FL Motor's PWM
uint16_t pwmRR = 255;       // RR Motor's PWM
uint16_t pwmRL = 255;       // RL Motor's PWM

// Instantiate car parts
Encoder encoderFR{ENC_SLOTS};                       // FR Encoder
Encoder encoderFL{ENC_SLOTS};                       // FL Encoder
Encoder encoderRR{ENC_SLOTS};                       // RR Encoder
Encoder encoderRL{ENC_SLOTS};                       // RL Encoder
Motor motorFR{PWM_CH_FR_1, PWM_CH_FR_2, encoderFR}; // FR Motor
Motor motorFL{PWM_CH_FL_1, PWM_CH_FL_2, encoderFL}; // FL Motor
Motor motorRR{PWM_CH_RR_1, PWM_CH_RR_2, encoderRR}; // RR Motor
Motor motorRL{PWM_CH_RL_1, PWM_CH_RL_2, encoderRL}; // RL Motor
Car car{CARLENGTH, CARWIDTH,
        WHEELTRACK, WHEELBASE, WDIAMETER,
        MAX_PWM, MIN_PWM,
        motorFR, motorFL, motorRR, motorRL}; // The car

/**
 * ISR called by the front right wheel encoder
 */
void IRAM_ATTR ISR_FR()
{
  // Current RPM value for the FR motor
  int16_t newInput = car.getMotorFR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFR = newInput;
}

/**
 * ISR called by the front left wheel encoder
 */
void IRAM_ATTR ISR_FL()
{
  // Current RPM value for the FL motor
  int16_t newInput = car.getMotorFL().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFL = newInput;
}

/**
 * ISR called by the rear right wheel encoder
 */
void IRAM_ATTR ISR_RR()
{
  // Current RPM value for the RR motor
  int16_t newInput = car.getMotorRR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmRR = newInput;
}

/**
 * ISR called by the rear left wheel encoder
 */
void IRAM_ATTR ISR_RL()
{
  // Current RPM value for the RL motor
  int16_t newInput = car.getMotorRL().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmRL = newInput;
}

/**
 * Initialize I/O Pins
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
 * Initialize the PWM pins
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
 * Initialize the interrupt pins
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
 * Initialize the WiFi connection
 */
void setup_wifi()
{
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifiSsid);

  WiFi.begin(wifiSsid, wifiPass);

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

void reconnect(const char *mqttServer)
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqttServer))
    {
      Serial.println("connected");

      // Subscribe to the topics
      mqttClient.subscribe("Deviation");
      mqttClient.subscribe("Speed");
    }
    else
    {
      Serial.print("failed, rc=");
      mqttClient.subscribe("Deviation");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String messageTemp;
  double value;
  
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  value = messageTemp.toDouble();

  Serial.println();

  // Check to which of the subscripions this message belongs to and take action

  // Deviation says if the car needs to turn to follow the road
  if (String(topic) == "Deviation")
  {
    /*
     * Values close (to decide how close) to 0 -> go straight
     * Positive value -> turn right
     * Negative value -> turn left
     */

  }
  // Speed says the speed in percentage, acceptable range format: -1.0 to 1.0
  else if (String(topic) == "Speed")
  {
    /*
     * 0 -> stop
     * Positive value -> move forward
     * Negative value -> move backward
     */

    int pwm = car.speedRatioToPwm(value);

    // move the car
    if(value == 0)
    {
      car.brake();
    }
    else if (value > 0)
    {
      car.forward(pwm, pwm, pwm, pwm);
    }
    else
    {
      car.reverse(pwm, pwm, pwm, pwm);
    }
  }
}

void setup()
{
  setupPins();
  setupPWM();
  setupInterrupts();
  setup_wifi();

  // Assign the callback function to the MQTT Client
  // mqttClient.setCallback(callback);

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
  double speed;
  double avgrpm;
  avgrpm = (rpmFR + rpmFL + rpmRR + rpmRL) / 4;
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

void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("ESP8266Client"))
    {
      Serial.println("connected");

      // Subscribe
      mqttClient.subscribe("Deviation");
      mqttClient.subscribe("Speed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
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

  // Call the MQTT client loop
  mqttClient.loop();

  // Get the current timestamp
  nowTime = millis();
  elapsedTime = nowTime - startTime;

  // Debug
  // Print the motors' rpm each half a second
  if (nowTime - debugResTime > 500)
  {
    printRpm();

    debugResTime = nowTime;
  }
}
