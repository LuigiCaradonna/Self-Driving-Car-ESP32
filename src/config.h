// Connection data
#define WIFISSID "Home&Life SuperWiFi-133D" // WiFi network SSID
#define WIFIPASS "YPXAM4PQ7QEL44NG"         // WiFi Password
#define MQTTSERVER "192.168.1.250"          // MQTT Server IP address
#define MQTTPORT 1883                       // MQTT Server Port
#define MQTTCLIENTID "ESP32Client"          // MQTT Client ID

// Front Motors
#define MOTOR_FR_1 18 // FR Motor IN1 pin
#define MOTOR_FR_2 19 // FR Motor IN2 pin
#define MOTOR_FL_1 32 // FL Motor IN1 pin
#define MOTOR_FL_2 33 // FL Motor IN2 pin

// Rear Motors
#define MOTOR_RR_1 16 // RR Motor IN1 pin
#define MOTOR_RR_2 17 // RR Motor IN2 pin
#define MOTOR_RL_1 26 // RL Motor IN1 pin
#define MOTOR_RL_2 27 // RL Motor IN2 pin

/** 
 * DRV8833 SLEEP pin
 * Both the drivers are connected to the same pin,
 * at the moment it is not necessary to disable them separately
 */
#define MOTORS_EN 5

// Car's dimensions in mm
#define CARLENGTH 256  // Car's full length (mm)
#define CARWIDTH 153   // Car's full width (mm)
#define WHEELTRACK 128 // Distance between the center of the wheels on the same axle (mm)
#define WHEELBASE 115  // Distance between the front and rear axles (mm)
#define WDIAMETER 66   // Wheels' diameter (mm)

// Encoders' pins
#define ENC_FR 36 // Front Right Encoder, pin SP
#define ENC_FL 39 // Front Left Encoder, pin SN
#define ENC_RR 34 // Rear Right Encoder
#define ENC_RL 35 // Rear Left Encoder

// Number of slots of the speed encoder
#define ENC_SLOTS 20

// Setting PWM properties
#define FREQ 50000    // Change to uint32_t for frequencies higher than 65535
#define PWM_CH_FR_1 0 // PWM channel front right IN1
#define PWM_CH_FR_2 1 // PWM channel front right IN2
#define PWM_CH_FL_1 2 // PWM channel front left IN1
#define PWM_CH_FL_2 3 // PWM channel front left IN2
#define PWM_CH_RR_1 4 // PWM channel rear right IN1
#define PWM_CH_RR_2 5 // PWM channel rear right IN2
#define PWM_CH_RL_1 6 // PWM channel rear left IN1
#define PWM_CH_RL_2 7 // PWM channel rear left IN2
#define RESOLUTION 8  // 8 bit - 0 to 255
#define MAX_PWM 255   // Max PWM duty-cycle value
#define MIN_PWM 80    // Min PWM duty-cycle value, make sure the motors run