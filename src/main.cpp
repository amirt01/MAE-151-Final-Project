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
constexpr short MAX_LAUNCH_ANGLE = 45;
constexpr short MIN_LAUNCH_ANGLE = 29;
constexpr short MAX_SERVO_ANGLE = 180;
constexpr short MIN_SERVO_ANGLE = 0;
int desired_launch_angle = 0.0;
int desired_servo_angle = 0.0;

// ================================================================
// ===                   ACTUATOR VARIABLES                     ===
// ================================================================

#define TRIGGER_SERVO_PIN 10
Servo trigger_servo;
constexpr int TRIGGER_CLOSED_ANGLE = 80;
constexpr int TRIGGER_OPEN_ANGLE = 90;

// ================================================================
// ===                      MPU VARIABLES                       ===
// ================================================================

Adafruit_MPU6050 mpu;
void MPU_Read();

constexpr int RECORDING_TIME = 250;
unsigned long launch_start_time = 0;
unsigned long launch_end_time = 0;

float max_angular_velocity = 0;
unsigned long last_pos_time = 0;
double actual_launch_angle = radians(225);

float max_linear_velocity = 0.f;
unsigned long last_accel_time = 0.f;

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

constexpr unsigned int LAUNCH_DELAY = 3000;

#define LED_PIN 13
bool blinkState = false;
unsigned long toggle_time = 0;
constexpr unsigned int LED_INCREMENT = 300;

#define BUZZER_PIN 9
bool buzzer_state = false;
unsigned long buzzer_time = 0;
constexpr unsigned int BUZZER_INCREMENT = LAUNCH_DELAY / 3 / 2;

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
  while (!Serial) delay(10); // will pause until serial console opens
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
      desired_launch_angle = Serial.parseInt();
      desired_servo_angle = map(desired_launch_angle, MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
      angle_servo.write(desired_servo_angle);

      state = State::Angle_Adjusting;

      launch_start_time = millis() + LAUNCH_DELAY;

    case State::Angle_Adjusting:
      // buzz until launch timer goes off
      if (millis() < launch_start_time) {
        Update_Buzzer();
        break;
      }

      state = State::Launching;

      // Update trigger servo
      trigger_servo.write(TRIGGER_OPEN_ANGLE);

      // Track launch time
      launch_end_time = millis() + RECORDING_TIME;
      last_accel_time = millis();
      last_pos_time = millis();

    case State::Launching:
      // Read until launch is over
      if (millis() < launch_end_time) {
        MPU_Read();
        break;
      }

      state = State::Resting;

      Print_Data();

    case State::Resting:
      noTone(BUZZER_PIN);
      angle_servo.write(MIN_SERVO_ANGLE);
  }

  Update_Blink();
}

// ================================================================
// ===                   FUNCTION DEFINITIONS                   ===
// ================================================================

void MPU_Setup() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);  // TODO: we may be able to get more accurate results by adjusting this
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  toggle_time = millis() + 1000;

  pinMode(BUZZER_PIN, OUTPUT);

  state = State::StandBye;

  Serial.println("Operations Initialized");
}

void MPU_Read() {
  // Try to get data and return if failed
  sensors_event_t a, g;
  if (!mpu.getGyroSensor()->getEvent(&g))
    return;

  if(!mpu.getAccelerometerSensor()->getEvent(&a))
    return;

  const static float INITIAL_GYRO_READING = -g.gyro.y;
  const float NEW_GYRO_READING = -g.gyro.y - INITIAL_GYRO_READING;

  const static float INITIAL_ACCEL_READING = -a.acceleration.x;
  const float NEW_ACCEL_READING = -a.acceleration.x - INITIAL_ACCEL_READING;

  // Update max velocity
  if (NEW_GYRO_READING > max_angular_velocity)
    max_angular_velocity = NEW_GYRO_READING;

  const float NEW_LINEAR_VELOCITY = NEW_ACCEL_READING * (millis() - last_accel_time);

  if (NEW_LINEAR_VELOCITY > max_linear_velocity)
    max_linear_velocity = NEW_LINEAR_VELOCITY;

  // Update launch_angle
  actual_launch_angle += NEW_GYRO_READING * (millis() - last_pos_time) / 1000;

  last_accel_time = millis();

  // Print angular velocity
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(NEW_GYRO_READING);
  Serial.print(" ");
  Serial.println(NEW_LINEAR_VELOCITY);
}

// Blink every second
void Update_Blink() {
  if (millis() < toggle_time) return;

  // Update LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  toggle_time += LED_INCREMENT;
}

void Update_Buzzer() {
  if (millis() < buzzer_time) return;
  
  // Update buzzer
  buzzer_state = !buzzer_state;
  if (buzzer_state) tone(BUZZER_PIN, 1000);  // 1000Hz
  else noTone(BUZZER_PIN);

  buzzer_time += BUZZER_INCREMENT;
}

void Print_Data() {
  Serial.print("Launch Velocity (gyro): ");
  Serial.print(max_angular_velocity * 3.97);
  Serial.println("in/s");
  
  Serial.print("Launch Velocity (accel): ");
  Serial.print(max_linear_velocity);
  Serial.println("m/s");

  Serial.print("Launch Angle: ");
  Serial.print(degrees(actual_launch_angle));
  Serial.println("deg");
}
