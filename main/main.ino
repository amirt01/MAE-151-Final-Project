#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

enum class State {StandBye, Launching, Resting} state;

void MPU_Setup();
void MPU_Read();

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial initialization
  Serial.println("Serial Initialized");

  MPU_Setup();

  Serial.println();
  delay(100);
}

void loop() {
  switch(state) {
    case State::StandBye:
      break;
    case State::Launching:
      MPU_Read();
      break;
    case State::Resting:
      break;
    default:
      Serial.println("state error");
      while(true);
  }

  Serial.println("");
  delay(500);
}

void MPU_Setup() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (true) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: +-8G");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: +- 500 deg/s");

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: 21 Hz");
}

void MPU_Read() {
  /* Get new sensor events with the readings */
  static sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.println(" m/s^2");

  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.println(" rad/s");
}
