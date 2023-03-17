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
const int MAX_SERVO_ANGLE = 0;
const int MIN_SERVO_ANGLE = 180;
int desired_launch_angle = 0;

// ================================================================
// ===                   ACTUATOR VARIABLES                     ===
// ================================================================

#define TRIGGER_SERVO_PIN 5
Servo trigger_servo;
const int TRIGGER_CLOSED_ANGLE = 80;
const int TRIGGER_OPEN_ANGLE = 90;

// ================================================================
// ===                      MPU VARIABLES                       ===
// ================================================================

Adafruit_MPU6050 mpu;
void MPU_Read();

const int RECORDING_TIME = 300;
unsigned long launch_start_time = 0;
unsigned long launch_end_time = 0;

float max_angular_velocity = 0;
unsigned long last_pos_time = 0;
float actual_launch_angle = radians(0);  // initial launch angle

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

// #define LAUNCH_BUTTON_PIN 4
#define LED_PIN 13
bool blinkState = false;
int toggle_time = 0;
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
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
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
  digitalWrite(LED_PIN, blinkState);
  toggle_time = millis() + 1000;

  Serial.println();
  Serial.print("Enter any character to begin: ");
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  switch(state) {
    case State::StandBye:
      // wait for charachter before continuing
      if (Serial.available() == 0) break;
      Serial.end();
      Serial.begin(115200);
      Serial.println();

      // Update servo angle
      angle_servo.write(MAX_SERVO_ANGLE);
      state = State::Angle_Adjusting;

      Serial.print("Enter any character to continue: ");

    case State::Angle_Adjusting:
      // wait for character before continuing
      if (Serial.available() == 0) break;
      Serial.end();
      Serial.begin(115200);
      Serial.println();

      // Update trigger servo
      trigger_servo.write(TRIGGER_OPEN_ANGLE);

      // Track launch time
      launch_start_time = millis();
      launch_end_time = launch_start_time + RECORDING_TIME;
      
      state = State::Launching;

    case State::Launching:
      // Read until launch is over
      if (millis() < launch_end_time) {
        MPU_Read();
        break;
      }
      
      // Present data
      Serial.print("Launch Velocity: ");
      Serial.print(max_angular_velocity * 3);  // TODO: add correct length
      Serial.println("in/s");

      Serial.print("Launch Angle: ");
      Serial.print(degrees(actual_launch_angle));
      Serial.println("deg");

      state = State::Resting;
      break;

    case State::Resting:
      // TODO: reset launch angle to top
      break;
  }

  // Blink every second
  if (millis() > toggle_time) {
    blinkState = !blinkState;
    toggle_time = millis() + 1000;
    digitalWrite(LED_PIN, blinkState);
  }
}

void MPU_Read() {
  // Try to get data and return if failed
  sensors_event_t g;
  if (!mpu.getGyroSensor()->getEvent(&g))
    return;

  // Update max velocity
  if (-g.gyro.y > max_angular_velocity)
    max_angular_velocity = -g.gyro.y;

  // Update launch_angle
  actual_launch_angle += -g.gyro.y * (millis() - last_pos_time);

  // Print velocity
  Serial.print(millis());
  Serial.print(" ");
  Serial.println(-g.gyro.y);
}
