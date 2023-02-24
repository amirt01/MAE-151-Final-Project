#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <Wire.h>
#include <Servo.h>

// ================================================================
// ===                      MPU VARIABLES                       ===
// ================================================================

MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                       OPERATIONS                         ===
// ================================================================

#define LAUNCH_BUTTON_PIN 4
void ActivateTrigger();

#define TRIGGER_SERVO_PIN 5
#define ANGLE_SERVO_PIN 6
Servo trigger_servo;
Servo angle_servo;
const int TRIGGER_CLOSED_ANGLE = 120;
const int TRIGGER_OPEN_ANGLE = 100;
const int LAUNCH_ANGLE = 90;

enum State { StandBye, Launching, Resting } state;

void Servo_Update();

void MPU_Setup();
void MPU_Read();

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Initialized");

  MPU_Setup();

  trigger_servo.attach(TRIGGER_SERVO_PIN);
  angle_servo.attach(ANGLE_SERVO_PIN);
  trigger_servo.write(TRIGGER_CLOSED_ANGLE);
  angle_servo.write(LAUNCH_ANGLE);

  pinMode(LAUNCH_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LAUNCH_BUTTON_PIN), ActivateTrigger, FALLING);

  state = State::StandBye;

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.println();
  delay(3000);
}

void loop() {
  switch(state) {
    case State::StandBye:
      Servo_Update();
      break;
    case State::Launching:
      MPU_Read();
      break;
    case State::Resting:
      break;
  }
}

void Servo_Update() {
  static int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) {
    trigger_servo.write(pos);
    angle_servo.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    trigger_servo.write(pos);
    angle_servo.write(pos);
    delay(15);
  }
}

void MPU_Setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // generate offsets and calibrate our MPU6050
  mpu.setXAccelOffset(-78);
  mpu.setYAccelOffset(-4286);
  mpu.setZAccelOffset(1189);
  mpu.setXGyroOffset (50);
  mpu.setYGyroOffset (-36);
  mpu.setZGyroOffset (-13);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void MPU_Read() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void ActivateTrigger() {
  state = (int)state + 1;
}
