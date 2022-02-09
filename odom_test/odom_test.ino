#include "PreMo.h"
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"

// CONSTANTS
const int TICKS_PER_REV = 20480; //PPR * 4
const int RC_ADDR = 0x80;
const double WHEEL_RADIUS = 20.4; //mm
const double WHEEL_DIST = 134; //mm
const uint32_t MOTOR_SPEED_MAX = 160000;
const uint32_t BAUD_RATE = 115200;
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const uint16_t GYRO_CALIBRATIONS = 100;
const float GYRO_DIGITS = 100.0;

const double Kp_v_L = 0.04573; // pid for motor velocity
const double Ki_v_L = 0.0;
const double Kd_v_L = 0.00311;
const double Kp_v_R = 0.04573;
const double Ki_v_R = 0.0;
const double Kd_v_R = 0.00327;
const uint32_t QPPS_L = 1006312; // experimentally determined
const uint32_t QPPS_R = 1031625;

// PID for path following (for turning when followiung path)
const double KP = 50;
const double KD = 1;
// PID for motors (for twist method)
const double KP_motor = 1.5;
const double KI_motor = 0;

// GLOBAL VARIABLES
volatile long enc_left;
volatile long enc_right;
volatile float omega; // rad/s
bool update_rc;
int32_t speed_left;
int32_t speed_right;
float gyro_drift;
unsigned long start_time;

// OBJECT DECLARATIONS
SoftwareSerial serial(10, 11);
RoboClaw rc(&Serial1, 10000);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

void update_encoders() {
  enc_left = LEFT_DIR * rc.ReadEncM1(RC_ADDR);
  enc_right = RIGHT_DIR * rc.ReadEncM2(RC_ADDR);
}

void update_gyro() {
  mpu.getEvent(&a, &g, &temp);
  omega = round((g.gyro.z * GYRO_DIGITS)) / GYRO_DIGITS - gyro_drift; // * 1.075
  if (fabs(omega) <= 0.02) {
    omega = 0.0;
  }
}

void update_line_sensors() {
  
}

void reset_encoders() {
  rc.ResetEncoders(RC_ADDR);
}


void set_left_forward(int spd) {
  //Serial.print("left forward: "); Serial.println(spd);
//  int32_t resL = MOTOR_SPEED_MAX * spd / 100;
//  uint32_t outputL = LEFT_DIR * resL;
//  rc.SpeedM1(RC_ADDR, outputL);

  if (spd != speed_left) {
    speed_left = spd;
    update_rc = true;
  }
}

void set_left_reverse(int spd) {
  //Serial.print("left reverse: "); Serial.println(spd);
//  int32_t resL = MOTOR_SPEED_MAX * spd / 100;
//  uint32_t outputL = -1 * LEFT_DIR * resL;
//  rc.SpeedM1(RC_ADDR, outputL);

  if (-1 * spd != speed_left) {
    speed_left = -1 * spd;
    update_rc = true;
  }
}

void set_right_forward(int spd) {
  //Serial.print("right forward: "); Serial.println(spd);
//  int32_t resR = MOTOR_SPEED_MAX * spd / 100;
//  uint32_t outputR = RIGHT_DIR * resR;
//  rc.SpeedM2(RC_ADDR, outputR);

  if (spd != speed_right) {
    speed_right = spd;
    update_rc = true;
  }
}

void set_right_reverse(int spd) {
  //Serial.print("right reverse: "); Serial.println(spd);
//  int32_t resR = MOTOR_SPEED_MAX * spd / 100;
//  uint32_t outputR = -1 * RIGHT_DIR * resR;
//  rc.SpeedM2(RC_ADDR, outputR);
  if (-1 * spd != speed_right) {
    speed_left = -1 * spd;
    update_rc = true;
  }
}

void stop_motors() {
  //rc.SpeedM1M2(RC_ADDR, 0, 0);

  if (speed_left != 0 || speed_right != 0) {
    speed_left = 0;
    speed_right = 0;
    update_rc = true;
  }
}

// MOTION PROFILING DECLARATIONS
MotorManager mm(set_left_forward, set_left_reverse, set_right_forward, set_right_reverse, stop_motors);
EncoderManager em(&enc_left, &enc_right, TICKS_PER_REV, update_encoders);
GyroManager gm(&omega, update_gyro);
PreMo pm(WHEEL_RADIUS, WHEEL_DIST, KP, KD, KP_motor, KI_motor, &mm, &em, &gm);

// MAIN AUTONOMOUS FUNCTIONS
void update_state() {
  
}

void handle_state() {
  
}

void set_drive() {
//  int32_t resL = MOTOR_SPEED_MAX * motorL / 100;
//  int32_t resR = MOTOR_SPEED_MAX * motorR / 100;
//  uint32_t outputL = LEFT_DIR * resL;
//  uint32_t outputR = RIGHT_DIR * resR;
//  //Serial.print("motorL: "); Serial.print(motorL); Serial.print(", motorR: "); Serial.println(motorR);
//  //Serial.print("resL: "); Serial.print(resL); Serial.print(", resR: "); Serial.println(resR);
//  rc.SpeedM1M2(RC_ADDR, outputL, outputR);

  if (update_rc) {
    uint32_t vel_left = LEFT_DIR * MOTOR_SPEED_MAX * speed_left / 100;
    uint32_t vel_right = RIGHT_DIR * MOTOR_SPEED_MAX * speed_right / 100;
    
    rc.SpeedM1M2(RC_ADDR, vel_left, vel_right);
    update_rc = false;
  }
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
  
  rc.ResetEncoders(RC_ADDR);
  //rc.SpeedM1M2(RC_ADDR, 0, 0);
  Serial.println("Done");

  // Initialize global variables
  update_rc = false;
  speed_left = 0;
  speed_right = 0;
  gyro_drift = 0;
  
  // Set parameters for motion profiling
  mm.setSpeedLimits(0, 100);
  pm.twistBothMotors(true);
  pm.setPathFollowSpeed(30);
  pm.setTwistSpeed(30);

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

int state = 0;
long last_update = 0;

float angle(float n){
  return 720.0/n;
}

void loop() {
  start_time = millis();
  
  // Update sensor readings and autonomous movements
  update_line_sensors();

  // Use sensor input to determine what to do
  update_state();
  handle_state(); // to cancel movement we should do pm.cancelMovement() and declare what we want it to instead
  pm.loop();
  //set_drive();

  // Print loop time (for debugging)
  // Serial.println(millis() - start_time);

  if (state == 0) {
    pm.startTranslate(100);
    state = 1;
  }

  // Print coordinates (for debugging)
  if (millis() - last_update >= 1000) {
    Serial.print("encL: ");
    Serial.print(enc_left);
    Serial.print(", encR: ");
    Serial.println(enc_right);
    Serial.print("X: ");
    Serial.print(pm.getX());
    Serial.print(", Y: ");
    Serial.print(pm.getY());
    Serial.print(", Alpha: ");
    Serial.println(pm.getHeading());
    last_update = millis();
  }

  /*  
  start_time = millis();
  //delay(50);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float z_reading = round((g.gyro.z*digits))/digits;
  omega = (z_reading-z_drift) * 1.075;
  
  updateEncoders();
  
  
  //dr.computePosition();
  //odom.loop();
  float star_len = 500.0;
  float n = 5;

  if (state == 0) {
    state = 1;
    /*
    int num_pts = 5;
    float xpts[num_pts] = {0, 75, 150, 300, 450, 525};
    float ypts[num_pts] = {0, 0, -150, -60, -150, 60};
    float dx = 500;
    float dy = 150;
    for(int i = 0; i < num_pts; i++){
      //ypts[i] = 100.0 * cos(xpts[i]/100.0)-100.0;
      xpts[i] = i * dx / (num_pts-1);
      ypts[i] = -0.5 * dy * (cos(PI * xpts[i] / dx) - 1);
      Serial.print(xpts[i]);
      Serial.print(", ");
      Serial.println(ypts[i]);
    }
    pm.startPathFollowing(xpts, ypts, num_pts);
    pm.twistDelta(-1*(180-angle(n))/2.0);
  } else if(!pm.isFollowingPath() && !pm.isTwisting() && state == 1){
    pm.forward(star_len);
    state = 2;
  } else if(!pm.isFollowingPath() && !pm.isTwisting() && state == 2){
    pm.twistDelta(angle(n));
    state = 1;
  }
  
//  if (state == 1 && !odom.isTranslating()) {
//    rc.SetM1VelocityPID(RC_ADDR, 0.0, 0.0, 0.0, QPPS_L);
//    rc.SetM2VelocityPID(RC_ADDR, 0.0, 0.0, 0.0, QPPS_R);
//    motorL = 0.0;
//    motorR = 0.0;
//  }

  if (millis() - startUpdate >= 2000) {
    //odom.setTranslating(false);
  }
  
  //setDrive();
  pm.loop();
  //Serial.println("looped");
  
  if (millis() - lastUpdate >= 1000) {
    Serial.print("encL: ");
    Serial.print(encL);
    Serial.print(", encR: ");
    Serial.println(encR);
    Serial.print("X: ");
    Serial.print(pm.getX());
    Serial.print(", Y: ");
    Serial.print(pm.getY());
    Serial.print(", Alpha: ");
    Serial.println(pm.getHeading());
    lastUpdate = millis();
  }
  */
}
