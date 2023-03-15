#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// ================================================================
// ===                 LAUNCH ANGLE VARIABLES                   ===
// ================================================================

#define ANGLE_SERVO_PIN 6
Servo angle_servo;
const int MAX_LAUNCH_ANGLE = 35;
const int MIN_LAUNCH_ANGLE = 5;
const int MAX_SERVO_ANGLE = 180;
const int MIN_SERVO_ANGLE = 0;
int launch_angle = 0;

// ================================================================
// ===                   ACTUATOR VARIABLES                     ===
// ================================================================

#define TRIGGER_SERVO_PIN 5
Servo trigger_servo;
const int TRIGGER_CLOSED_ANGLE = 100;
const int TRIGGER_OPEN_ANGLE = 80;

// ================================================================
// ===                      MPU VARIABLES                       ===
// ================================================================

Adafruit_MPU6050 mpu;
void MPU_Read();

const int RECORDING_TIME = 1000;
int launch_start_time = 0;
int launch_end_time = 0;

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

// #define LAUNCH_BUTTON_PIN 4
#define LED_PIN 13
bool blinkState = false;
enum State { StandBye, Angle_Adjusting, Launching, Resting } state;

// ================================================================
// ===                          SETUP                           ===
// ================================================================

void MPU_Setup() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //TODO: update calibration values with propper ones
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU Initialized");
}

void Servo_Setup() {
  trigger_servo.attach(TRIGGER_SERVO_PIN);
  trigger_servo.write(TRIGGER_CLOSED_ANGLE);
  angle_servo.attach(ANGLE_SERVO_PIN);
  angle_servo.write(MIN_SERVO_ANGLE);
  Serial.println("Servos Initialized");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause until serial console opens
  }
  Serial.println("Serial Initialized");

  MPU_Setup();
  Servo_Setup();

  // TODO: Add on board button support
  // pinMode(LAUNCH_BUTTON_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(LAUNCH_BUTTON_PIN), ActivateTrigger, FALLING);

  state = State::StandBye;

  pinMode(LED_PIN, OUTPUT);

  Serial.println();
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  switch(state) {
    case State::StandBye:
      Serial.print("Enter any character to begin: ");
      while (Serial.available() == 0) {}
      Serial.println();
      Serial.end();
      Serial.begin(115200);
      state = State::Angle_Adjusting;

    case State::Angle_Adjusting:
      angle_servo.write(MAX_SERVO_ANGLE);
      Serial.print("Enter any character to continue: ");
      while (Serial.available() == 0) {}
      Serial.println();
      launch_start_time = millis();
      launch_end_time = launch_start_time + RECORDING_TIME;
      state = State::Launching;

    case State::Launching:
      trigger_servo.write(TRIGGER_OPEN_ANGLE);
      if (millis() > launch_end_time) break;
      MPU_Read();
      break;
      // state = State::Resting;

    case State::Resting:
      break;
  }
}

void MPU_Read() {
  sensors_event_t a;
  mpu.getAccelerometerSensor()->getEvent(&a);
  Serial.println(-a.acceleration.x);
}
