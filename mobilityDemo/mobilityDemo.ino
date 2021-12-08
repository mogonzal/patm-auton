#include "Odom.h"
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include "math.h"

// CONSTANTS
const int TICKS_PER_REV = 20480; //PPR * 4
const int RC_ADDR = 0x80;
const int WHEEL_RADIUS = 20; //mm
const int WHEEL_DIST = 134; //mm
const uint32_t MOTOR_SPEED_MAX = 160000;
const uint32_t BAUD_RATE = 115200;
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const double Kp_v_L = 0.04573; // pid for motor velocity
const double Ki_v_L = 0.0;
const double Kd_v_L = 0.00311;
const double Kp_v_R = 0.04573;
const double Ki_v_R = 0.0;
const double Kd_v_R = 0.00327;
const uint32_t QPPS_L = 1006312; // experimentally determined
const uint32_t QPPS_R = 1031625;

// SERIAL DECLARATIONS
SoftwareSerial serial(10, 11);

// ROBOCLAW DECLARATIONS
RoboClaw rc(&Serial2, 10000);
volatile long encL = 0.0;
volatile long encR = 0.0;
double motorL = 0.0;
double motorR = 0.0;

void stopMotors() {
  //Serial.println("motors stopped");
  rc.SpeedM1(RC_ADDR, 0);
  rc.SpeedM2(RC_ADDR, 0);
}

void updateEncoders() {
   encL = LEFT_DIR * rc.ReadEncM1(RC_ADDR);
   encR = RIGHT_DIR * rc.ReadEncM2(RC_ADDR);
}

void resetEncoders() {
  rc.ResetEncoders(RC_ADDR);
}

void setDrive() {
  int32_t resL = MOTOR_SPEED_MAX * motorL / 100;
  int32_t resR = MOTOR_SPEED_MAX * motorR / 100;
  uint32_t outputL = LEFT_DIR * resL;
  uint32_t outputR = RIGHT_DIR * resR;
  //Serial.print("motorL: "); Serial.print(motorL); Serial.print(", motorR: "); Serial.println(motorR);
  //Serial.print("resL: "); Serial.print(resL); Serial.print(", resR: "); Serial.println(resR);
  rc.SpeedM1M2(RC_ADDR, outputL, outputR);
}

// MOTION PROFILING DECLARATIONS
Odom odom(&encL, &encR, &motorL, &motorR, resetEncoders);

// OTHER VARIABLES
unsigned long lastUpdate;
unsigned long startUpdate;

void setup() {
  // init serial
  Serial.begin(BAUD_RATE);
  while (!Serial) {}
  delay(50);

  // init roboclaw
  rc.begin(BAUD_RATE);
  rc.SetM1VelocityPID(RC_ADDR, Kp_v_L, Ki_v_L, Kd_v_L, QPPS_L);
  rc.SetM2VelocityPID(RC_ADDR, Kp_v_R, Ki_v_R, Kd_v_R, QPPS_R);
  //rc.SetM1PositionPID(RC_ADDR, 75.51887, 0, 475.47365, 0, 0, 0, 0);
  //rc.SetM2PositionPID(RC_ADDR, 75.51887, 0, 475.47365, 0, 0, 0, 0);
  rc.ResetEncoders(RC_ADDR);
  motorL = 0;
  motorR = 0;
  delay(50);

  // init motion profiling library
  odom.configureWheelbase(WHEEL_RADIUS, WHEEL_DIST, TICKS_PER_REV);
  //odom.configureTranslationPID(7.5, 0.0015, 0.55, 0.0);
  //odom.configureTranslationPID(3.2, 0.0, 0.4, 0.0);
  odom.configureTranslationPID(1.0, 0.00115, 0.001, 0.1);
  odom.configureRotationPID(250.0, 0.0, 100.0);
  delay(50);

  delay(3000);
  Serial.println("START");
  lastUpdate = millis();
  startUpdate = millis();
}

int state = 0;

void loop() {
  
  updateEncoders();
  odom.loop();

  if (state == 0) {
    state = 1;
    odom.startTranslate(500.0); // move 500mm forward
    //odom.startRotate(PI/2 * 2.25);
    //motorL = 7.0;
    //motorR = 7.0;
  }
  if (state == 1 && !odom.isTranslating()) {
    rc.SetM1VelocityPID(RC_ADDR, 0.0, 0.0, 0.0, QPPS_L);
    rc.SetM2VelocityPID(RC_ADDR, 0.0, 0.0, 0.0, QPPS_R);
    motorL = 0.0;
    motorR = 0.0;
  }

  if (millis() - startUpdate >= 2000) {
    odom.setTranslating(false);
  }
  
  setDrive();
  
//  if (millis() - lastUpdate >= 1000) {
//    Serial.print("encL: ");
//    Serial.print(encL);
//    Serial.print(", encR: ");
//    Serial.println(encR);
//    lastUpdate = millis();
//  }
}
