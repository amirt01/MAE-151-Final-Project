#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// ================================================================
// ===                 LAUNCH ANGLE VARIABLES                   ===
// ================================================================

#define ANGLE_SERVO_PIN 11
Servo angle_servo;
const int MAX_LAUNCH_ANGLE = 35;
const int MIN_LAUNCH_ANGLE = 5;
const int MAX_SERVO_ANGLE = 0;
const int MIN_SERVO_ANGLE = 180;
double desired_launch_angle = 0.0;
double desired_servo_angle = 0.0;

// ================================================================
// ===                   ACTUATOR VARIABLES                     ===
// ================================================================

#define TRIGGER_SERVO_PIN 10
Servo trigger_servo;
const int TRIGGER_CLOSED_ANGLE = 80;
const int TRIGGER_OPEN_ANGLE = 90;

// ================================================================
// ===                      MPU VARIABLES                       ===
// ================================================================

Adafruit_MPU6050 mpu;
void MPU_Read();

const int RECORDING_TIME = 250;
unsigned long launch_start_time = 0;
unsigned long launch_end_time = 0;

float max_angular_velocity = 0;
unsigned long last_pos_time = 0;
float actual_launch_angle = radians(0);  // FIXME: add initial launch angle

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

const unsigned long LAUNCH_DELAY = 3000;

// #define LAUNCH_BUTTON_PIN 4
#define LED_PIN 13
bool blinkState = false;
unsigned long toggle_time = 0;

#define BUZZER_PIN 9
bool buzzer_state = false;
unsigned long buzzer_time = 0;
const unsigned long BUZZER_INCREMENT = LAUNCH_DELAY / 3 / 2;

enum class State { StandBye, Angle_Adjusting, Launching, Resting } state;

// ================================================================
// ===                  FUNCTION DECLERATIONS                   ===
// ================================================================

void MPU_Setup();
void Servo_Setup();
void Operations_Setup();

void MPU_Read();
void Update_Blink();
void Update_Buzzer();
void Print_Data();

// ================================================================
// ===                          SETUP                           ===
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause until serial console opens
  }
  Serial.println("Serial Initialized");

  MPU_Setup();
  Servo_Setup();
  Operations_Setup();

  Serial.println();
  Serial.print("Enter launch angle: ");
}

// ================================================================
// ===                           RUN                            ===
// ================================================================

void loop() {
  switch(state) {
    case State::StandBye:
      if (Serial.available() == 0) break;
      Serial.println();
      
      // Get user input and convert to servo angle
      // TODO: @PalmerJR Validate that the launch angle can be adjusted with user input and is accurate 
      desired_launch_angle = Serial.read();
      desired_servo_angle = map(MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, desired_launch_angle);
      if (desired_servo_angle > MAX_SERVO_ANGLE) angle_servo.write(MAX_SERVO_ANGLE);
      else if (desired_servo_angle < MIN_SERVO_ANGLE) angle_servo.write(MIN_SERVO_ANGLE);
      else angle_servo.write(desired_servo_angle);

      launch_start_time = millis() + LAUNCH_DELAY;

      state = State::Angle_Adjusting;

    case State::Angle_Adjusting:
      // buzz until launch timer goes off
      if (millis() < launch_start_time) {
        Update_Buzzer();
        break;
      }
      
      // Track launch time
      launch_end_time = millis() + RECORDING_TIME;

      // Update trigger servo
      trigger_servo.write(TRIGGER_OPEN_ANGLE);

      state = State::Launching;

    case State::Launching:
      // Read until launch is over
      if (millis() < launch_end_time) {
        MPU_Read();
        break;
      }
      
      Print_Data();

      state = State::Resting;

    case State::Resting:
      noTone(BUZZER_PIN);
      angle_servo.write(MAX_LAUNCH_ANGLE);
  }

  Update_Blink();
}

// ================================================================
// ===                   FUNCTION DEFINITIONS                   ===
// ================================================================

void MPU_Setup() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // TODO: we may be able to get more accurate results by adjusting this
  Serial.println("MPU Initialized");
}

void Servo_Setup() {
  trigger_servo.attach(TRIGGER_SERVO_PIN);
  trigger_servo.write(TRIGGER_CLOSED_ANGLE);
  angle_servo.attach(ANGLE_SERVO_PIN);
  angle_servo.write(MIN_SERVO_ANGLE);
  Serial.println("Servos Initialized");
}

void Operations_Setup() {
  // TODO: Add on board button support
  // pinMode(LAUNCH_BUTTON_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(LAUNCH_BUTTON_PIN), ActivateTrigger, FALLING);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  toggle_time = millis() + 1000;

  pinMode(BUZZER_PIN, OUTPUT);

  state = State::StandBye;
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
  actual_launch_angle += -g.gyro.y * (millis() - last_pos_time) / 1000;  // FIXME: bug here

  // Print velocity
  Serial.print(millis());
  Serial.print(" ");
  Serial.println(-g.gyro.y);
}

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;
  blinkState = !blinkState;
  toggle_time = millis() + 300;
  digitalWrite(LED_PIN, blinkState);
}

void Update_Buzzer() {
  if (millis() < buzzer_time) return;
  buzzer_state = !buzzer_state;
  if (buzzer_state) tone(BUZZER_PIN, 1000);
  else noTone(BUZZER_PIN);
  buzzer_time += BUZZER_INCREMENT;
}

void Print_Data() {
  Serial.print("Launch Velocity: ");
  Serial.print(max_angular_velocity * 3);  // FIXME: add correct length
  Serial.println("in/s");

  Serial.print("Launch Angle: ");
  Serial.print(degrees(actual_launch_angle));
  Serial.println("deg");
}
