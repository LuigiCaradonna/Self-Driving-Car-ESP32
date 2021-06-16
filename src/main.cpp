#include <Arduino.h>
#include <config.h>
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

// The car should move or not
bool move = false;

WiFiClient wifiClient;               // Client WiFi
PubSubClient mqttClient{wifiClient}; // Client MQTT

int16_t volatile rpmFR = 0; // Front Right Motor's RPM
int16_t volatile rpmFL = 0; // Front Left Motor's RPM
int16_t volatile rpmRR = 0; // Rear Right Motor's RPM
int16_t volatile rpmRL = 0; // Rear Left Motor's RPM

// Initial PWM values
uint16_t pwmFR = 170; // Front Right Motor's PWM
uint16_t pwmFL = 170; // Front Left Motor's PWM
uint16_t pwmRR = 170; // Rear Right Motor's PWM
uint16_t pwmRL = 170; // Rear Left Motor's PWM

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
        WHEELTRACK, WHEELBASE, WDIAMETER,
        MAX_PWM, MIN_PWM,
        motorFR, motorFL, motorRR, motorRL}; // The car

/**
 * ISR called by the front right wheel encoder
 */
void IRAM_ATTR ISR_FR()
{
  // Current RPM value for the Front Right Motor
  int16_t newInput = car.getMotorFR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFR = newInput;
}

/**
 * ISR called by the front left wheel encoder
 */
void IRAM_ATTR ISR_FL()
{
  // Current RPM value for the Front Left motor
  int16_t newInput = car.getMotorFL().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmFL = newInput;
}

/**
 * ISR called by the rear right wheel encoder
 */
void IRAM_ATTR ISR_RR()
{
  // Current RPM value for the Rear Right motor
  int16_t newInput = car.getMotorRR().getEncoder().isr(nowTime);

  if (newInput != -1)
    rpmRR = newInput;
}

/**
 * ISR called by the rear left wheel encoder
 */
void IRAM_ATTR ISR_RL()
{
  // Current RPM value for the Rear Left motor
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

      // Subscribe to the topics
      mqttClient.subscribe("Move");
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

void callback(char *topic, byte *payload, unsigned int length)
{
  String messageTemp;
  double value;

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)payload[i];
  }

  value = messageTemp.toDouble();

  Serial.println();

  // Check to which of the subscripions this message belongs to and take action

  if (String(topic) == "Move")
  {
    // value must be either 1 or 0, if it is not, set to 0
    move = (value == 0 || value == 1) ? (int)value : 0;

    // If move is false (0)
    if (!move)
      // Stop the car
      car.brake();
  }
  /* 
   * Deviation says if the car needs to turn to follow the road
   * Execute only if the car is moving
   */
  else if (String(topic) == "Deviation" && value >= 0 && value <= 1 && move)
  {
    /*
     * Values close to 0 -> go straight
     * Positive value -> turn right
     * Negative value -> turn left
     */
    if (abs(value) <= 0.1)
    {
      car.forward(pwmFR, pwmFL, pwmRR, pwmRL);
    }
    else if (value > 0)
    {
      car.turnRight(value);
    }
    else
    {
      car.turnLeft(value);
    }
  }
  /* 
   * Speed says the speed in percentage, acceptable range format: -1.0 to 1.0
   * Execute only if the car is moving
   */
  else if (String(topic) == "Speed" && move)
  {
    /*
     * 0 -> stop
     * Positive value -> move forward
     * Negative value -> move backward
     */

    int pwm = car.speedRatioToPwm(value);

    // At the moment the PID control is disabled, thus, the PWM value is the same for all the wheels
    pwmFR = pwm;
    pwmFL = pwm;
    pwmRR = pwm;
    pwmRL = pwm;

    // Move the car
    if (value == 0)
    {
      car.brake();
    }
    else if (value > 0)
    {
      car.forward(pwmFR, pwmFL, pwmRR, pwmRL);
    }
    else
    {
      car.reverse(pwmFR, pwmFL, pwmRR, pwmRL);
    }
  }
}

void setupMqtt()
{
  // Set the server IP address and port
  mqttClient.setServer(MQTTSERVER, MQTTPORT);

  // Assign the callback function to the MQTT Client
  mqttClient.setCallback(callback);

  reconnect();
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
  // nowTime = millis();
  // elapsedTime = nowTime - startTime;

  // Debug
  // Print the motors' rpm each half a second
  // if (nowTime - debugResTime > 500)
  // {
  //   printRpm();

  //   debugResTime = nowTime;
  // }
}
