#include "PreMo.h"
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"

const int RC_ADDR = 0x80;
const int LINE_SENSOR_COUNT = 4;
const int SONAR_COUNT = 3;
const double Kp_v_L = 0.04573; // pid for motor velocity
const double Ki_v_L = 0.0;
const double Kd_v_L = 0.00311;
const double Kp_v_R = 0.04573;
const double Ki_v_R = 0.0;
const double Kd_v_R = 0.00327;
const uint32_t QPPS_L = 1006312; // experimentally determined
const uint32_t QPPS_R = 1031625;
const uint32_t BAUD_RATE = 115200;
const uint16_t GYRO_CALIBRATIONS = 100;
const float GYRO_DIGITS = 100.0;


const int line_sensor_pins[LINE_SENSOR_COUNT] = {A8, A9, A6, A7}; //fl, fr, bl, br
int32_t line_sensor_values[LINE_SENSOR_COUNT] = {0, 0, 0, 0};

const int sonar_pins[SONAR_COUNT] = {A0, A1, A3};
int32_t sonar_values[SONAR_COUNT] = {0, 0, 0};

float omega = 0;

float gyro_drift;
SoftwareSerial serial(10, 11);
RoboClaw rc(&Serial1, 10000);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

void pulse_sensors() {
  // pulse line sensors
  for (int32_t i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_sensor_values[i] = analogRead(line_sensor_pins[i]);
  }

  // pulse sonar sensors
  for (int32_t i = 0; i < SONAR_COUNT; i++) {
    sonar_values[i] = analogRead(sonar_pins[i]);
  }

  // pulse gyro
  mpu.getEvent(&a, &g, &temp);
  omega = round((g.gyro.z * GYRO_DIGITS)) / GYRO_DIGITS - gyro_drift; // * 1.075
  if (fabs(omega) <= 0.02) {
    omega = 0.0;
  }
}

void disp_sensors() {
  Serial.println("Line sensors:");
  Serial.print(line_sensor_values[0]); Serial.print(" ");
  Serial.println(line_sensor_values[1]);
  Serial.print(line_sensor_values[2]); Serial.print(" ");
  Serial.println(line_sensor_values[3]);
  Serial.println("");
  
  Serial.println("Sonar sensors:");
  Serial.print(sonar_values[0]); Serial.print(" ");
  Serial.print(sonar_values[1]); Serial.print(" ");
  Serial.println(sonar_values[2]);
  Serial.println("");

  Serial.print("Gyro omega: "); Serial.println(omega);
  Serial.println("");
}

void setup() {
  // Initialize serial
  Serial.begin(BAUD_RATE);
  while (!Serial) {}
  delay(20);
  Serial.println("Initialized serial");

  // Initialize roboclaw
  Serial.print("Initializing roboclaw...");
  rc.begin(BAUD_RATE);
  rc.SetM1VelocityPID(RC_ADDR, Kp_v_L, Ki_v_L, Kd_v_L, QPPS_L);
  rc.SetM2VelocityPID(RC_ADDR, Kp_v_R, Ki_v_R, Kd_v_R, QPPS_R);
  rc.SetM1PositionPID(RC_ADDR, 75.51887, 0, 475.47365, 0, 0, 0, 0);
  rc.SetM2PositionPID(RC_ADDR, 75.51887, 0, 475.47365, 0, 0, 0, 0);

  rc.SpeedM1M2(RC_ADDR, 0, 0);
  rc.ResetEncoders(RC_ADDR);
  Serial.println("Done");

  // Initialize line sensors
  Serial.print("Initalizing line sensors...");
  for(int32_t i = 0; i < LINE_SENSOR_COUNT; i++){
    pinMode(line_sensor_pins[i], INPUT);
  }
  Serial.println("Done");

  // Initialize sonar sensors
  Serial.print("Initalizing sonar sensors...");
  for(int32_t i = 0; i < SONAR_COUNT; i++){
    pinMode(sonar_pins[i], INPUT);
  }
  Serial.println("Done");

  Serial.print("Looking for gyro...");
  gyro_drift = 0;
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(5);
    }
  }
  Serial.println("MPU6050 Found!");

  // Initialize gyro
  Serial.print("Calibrating gyro...");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate gyro
  for (int i = 0; i < GYRO_CALIBRATIONS; i++) {
    mpu.getEvent(&a, &g, &temp);
    gyro_drift += g.gyro.z;
    delay(10);
  }
  
  gyro_drift /= float(GYRO_CALIBRATIONS);
  gyro_drift = round(gyro_drift * GYRO_DIGITS) / GYRO_DIGITS;
  Serial.print("Gyro calibrated... drift = ");
  Serial.println(gyro_drift);

  // Wait til 5s have passed
  Serial.print("Waiting for 5sec mark...");
  while(millis() < 5000) {
    delay(5);  
  }

  // All done! Start main loop
  Serial.println("START");
}

void loop() {
  pulse_sensors();
  disp_sensors();
  delay(1000);
}
