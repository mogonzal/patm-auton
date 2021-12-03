#include "PreMo.h"
#include "Odom.h"
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include "math.h"

// CONSTANTS
const int TICKS_PER_REV = 20480; //PPR * 4
const int RC_ADDR = 0x80;
const int WHEEL_RADIUS = 20; //mm
const int WHEEL_DIST = 134; //mm
const uint32_t MOTOR_SPEED_MAX = 100000;
const uint32_t BAUD_RATE = 115200;
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const double Kp_v_L = 0.04573; // pid for motor velocity
const double Ki_v_L = 0.0;
const double Kd_v_L = 0.00311;
const double Kp_v_R = 0.03991;
const double Ki_v_R = 0.0;
const double Kd_v_R = 0.00327;
const uint32_t QPPS_L = 1006312; // experimentally determined
const uint32_t QPPS_R = 1031625;

// SERIAL DECLARATIONS
SoftwareSerial serial(10, 11);

// ROBOCLAW DECLARATIONS
RoboClaw rc(&Serial2, 10000);
volatile long encL;
volatile long encR;
double motorL;
double motorR;

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
  uint32_t resL = MOTOR_SPEED_MAX * motorL / 100;
  uint32_t resR = MOTOR_SPEED_MAX * motorR / 100;

  rc.SpeedM1(RC_ADDR, LEFT_DIR * resL);
  rc.SpeedM2(RC_ADDR, RIGHT_DIR * resR);
}

// MOTION PROFILING DECLARATIONS
Odom odom(&encL, &encR, &motorL, &motorR, resetEncoders);

// OTHER VARIABLES
unsigned long lastUpdate;

void setup() {
  // init serial
  Serial.begin(BAUD_RATE);
  while (!Serial) {}
  delay(50);

  // init roboclaw
  rc.begin(BAUD_RATE);
  rc.SetM1VelocityPID(RC_ADDR, Kp_v_L, Ki_v_L, Kd_v_L, QPPS_L);
  rc.SetM2VelocityPID(RC_ADDR, Kp_v_R, Ki_v_R, Kd_v_R, QPPS_R);
  rc.ResetEncoders(RC_ADDR);
  motorL = 0;
  motorR = 0;
  delay(50);

  // init motion profiling library
  odom.configureWheelbase(WHEEL_RADIUS, WHEEL_DIST, TICKS_PER_REV);
  odom.configureTranslationPID(7.5, 0.0015, 0.55, 0.0);
  delay(50);

  delay(3000);
  Serial.println("START");
  lastUpdate = millis();
}

int state = 0;

void loop() {
  updateEncoders();
  odom.loop();

  if (state == 0) {
    state = 1;
    odom.startTranslate(500.0); // move 500mm forward
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
