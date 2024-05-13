# esp-32-reciver
#include <esp_now.h>
#include <WiFi.h>

// Motor pins
int enableFrontRightMotor = 22;
int frontRightMotorPin1 = 16;
int frontRightMotorPin2 = 17;

int enableFrontLeftMotor = 23;
int frontLeftMotorPin1 = 18;
int frontLeftMotorPin2 = 19;

int enableBackRightMotor = 27;
int backRightMotorPin1 = 26;
int backRightMotorPin2 = 25; // Moved to global scope

int enableBackLeftMotor = 35;
int backLeftMotorPin1 = 32;
int backLeftMotorPin2 = 33; // Moved to global scope

// Motor speed settings
const int MAX_MOTOR_SPEED = 200;
const int PWMFreq = 1000; // 1 KHz
const int PWMResolution = 8;

// PWM channels for front and back motors
const int frontRightMotorPWMSpeedChannel = 0;
const int frontLeftMotorPWMSpeedChannel = 1;
const int backRightMotorPWMSpeedChannel = 2;
const int backLeftMotorPWMSpeedChannel = 3;

// Signal timeout
#define SIGNAL_TIMEOUT 1000 // Signal timeout in milliseconds
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData receiverData;

bool throttleAndSteeringMode = false;

// Callback function when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == 0)
  {
    return;
  }

  memcpy(&receiverData, incomingData, sizeof(receiverData));

  Serial.print("Values: ");
  Serial.print(receiverData.xAxisValue);
  Serial.print(" ");
  Serial.print(receiverData.yAxisValue);
  Serial.print(" ");
  Serial.println(receiverData.switchPressed);

  if (receiverData.switchPressed == true)
  {
    throttleAndSteeringMode = !throttleAndSteeringMode;
  }

  if (throttleAndSteeringMode)
  {
    throttleAndSteeringMovements();
  }
  else
  {
    simpleMovements();
  }

  lastRecvTime = millis();
}

// Function for simple movements mode
void simpleMovements()
{
  int xAxis = receiverData.xAxisValue;
  int yAxis = receiverData.yAxisValue;

  if (yAxis <= 75) // Move car Forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (yAxis >= 175) // Move car Backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (xAxis >= 175) // Move car Right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (xAxis <= 75) // Move car Left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else // Stop the car
  {
    rotateMotor(0, 0, 0, 0);
  }
}

// Function for throttle and steering movements mode
void throttleAndSteeringMovements()
{
  int throttle = map(receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map(receiverData.xAxisValue, 0, 254, -255, 255);

  int motorDirection = (throttle < 0) ? -1 : 1;

  int frontRightMotorSpeed = abs(throttle) - steering;
  int frontLeftMotorSpeed = abs(throttle) + steering;
  int backRightMotorSpeed = abs(throttle) - steering;
  int backLeftMotorSpeed = abs(throttle) + steering;

  frontRightMotorSpeed = constrain(frontRightMotorSpeed, 0, 255);
  frontLeftMotorSpeed = constrain(frontLeftMotorSpeed, 0, 255);
  backRightMotorSpeed = constrain(backRightMotorSpeed, 0, 255);
  backLeftMotorSpeed = constrain(backLeftMotorSpeed, 0, 255);

  rotateMotor(frontRightMotorSpeed * motorDirection, frontLeftMotorSpeed * motorDirection, backRightMotorSpeed * motorDirection, backLeftMotorSpeed * motorDirection);
}

// Function to control motor rotation
void rotateMotor(int frontRightMotorSpeed, int frontLeftMotorSpeed, int backRightMotorSpeed, int backLeftMotorSpeed)
{
  digitalWrite(frontRightMotorPin1, (frontRightMotorSpeed > 0) ? HIGH : LOW);
  digitalWrite(frontRightMotorPin2, (frontRightMotorSpeed < 0) ? HIGH : LOW);
  digitalWrite(backRightMotorPin1, (backRightMotorSpeed > 0) ? HIGH : LOW);
  digitalWrite(backRightMotorPin2, (backRightMotorSpeed < 0) ? HIGH : LOW);

  digitalWrite(frontLeftMotorPin1, (frontLeftMotorSpeed > 0) ? HIGH : LOW);
  digitalWrite(frontLeftMotorPin2, (frontLeftMotorSpeed < 0) ? HIGH : LOW);
  digitalWrite(backLeftMotorPin1, (backLeftMotorSpeed > 0) ? HIGH : LOW);
  digitalWrite(backLeftMotorPin2, (backLeftMotorSpeed < 0) ? HIGH : LOW);

  ledcWrite(frontRightMotorPWMSpeedChannel, abs(frontRightMotorSpeed));
  ledcWrite(frontLeftMotorPWMSpeedChannel, abs(frontLeftMotorSpeed));
  ledcWrite(backRightMotorPWMSpeedChannel, abs(backRightMotorSpeed));
  ledcWrite(backLeftMotorPWMSpeedChannel, abs(backLeftMotorSpeed));
}

// Function to set up pin modes and PWM
void setUpPinModes()
{
  pinMode(enableFrontRightMotor, OUTPUT);
  pinMode(frontRightMotorPin1, OUTPUT);
  pinMode(frontRightMotorPin2, OUTPUT);

  pinMode(enableFrontLeftMotor, OUTPUT);
  pinMode(frontLeftMotorPin1, OUTPUT);
  pinMode(frontLeftMotorPin2, OUTPUT);

  pinMode(enableBackRightMotor, OUTPUT);
  pinMode(backRightMotorPin1, OUTPUT);
  pinMode(backRightMotorPin2, OUTPUT);

  pinMode(enableBackLeftMotor, OUTPUT);
  pinMode(backLeftMotorPin1, OUTPUT);
  pinMode(backLeftMotorPin2, OUTPUT);

  // Set up PWM for motor speed
  ledcSetup(frontRightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(frontLeftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(backRightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(backLeftMotorPWMSpeedChannel, PWMFreq, PWMResolution);

  // Attach PWM channels to motor enable pins
  ledcAttachPin(enableFrontRightMotor, frontRightMotorPWMSpeedChannel);
  ledcAttachPin(enableFrontLeftMotor, frontLeftMotorPWMSpeedChannel);
  ledcAttachPin(enableBackRightMotor, backRightMotorPWMSpeedChannel);
  ledcAttachPin(enableBackLeftMotor, backLeftMotorPWMSpeedChannel);

  rotateMotor(0, 0, 0, 0);
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for data reception
  esp_now_register_recv_cb(OnDataRecv);

  setUpPinModes();
}

void loop()
{
  // Check for signal loss
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT)
  {
    rotateMotor(0, 0, 0, 0);
  }
}
