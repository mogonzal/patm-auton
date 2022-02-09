#include "RoboClaw.h"
#include "math.h"
#include <SoftwareSerial.h>

// CONSTANTS
const int RC_ADDR = 0x80;
const int IR_PIN = A5;
const int IR_THRESHOLD = 400;
const int LINE_SENSOR_COUNT = 4;
const int LINE_SENSOR_PINS[LINE_SENSOR_COUNT] = {A1, A0, A2, A3};
const int LINE_SENSOR_THRESHOLD = 100;
const uint32_t BAUD_RATE = 115200;
enum State {ATTACK, ROTATE, FORWARD, LINE_FL, LINE_FR, BACK_UP, TURN_AROUND, BACK_TO_CENTER, LINE_BL, LINE_BR, STRAIGHT_TO_CENTER};

// VARIABLES
State state = ROTATE;
boolean line_seen[LINE_SENSOR_COUNT] = {false, false, false, false};
// 0 = front left, 1 = front right, 2 = back left, 3 = back right

// COMPONENTS
SoftwareSerial serial(10, 11);
RoboClaw rc(&Serial2, 10000);

boolean any_line_seen() {
  if (line_seen[0] || line_seen[1] || line_seen[2] || line_seen[3]) {
    return true;
  }
  return false;
}

void check_line_sensors() {
  for (uint16_t i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_seen[i] = analogRead(LINE_SENSOR_PINS[i]) < LINE_SENSOR_THRESHOLD;
  }
}

void update_state() {
  check_line_sensors();
  
  // do logic based on sensor readings
  if (line_seen[0] && line_seen[1]) {
    state = BACK_UP;
    Serial.print("state: "); Serial.println("BACK UP");
  }
  else if (line_seen[0] && state != BACK_UP) {
    state = LINE_FL;
    Serial.print("state: "); Serial.println("LINE FL");
  }
  else if (line_seen[1] && state != BACK_UP) {
    state = LINE_FR;
    Serial.print("state: "); Serial.println("LINE FR");
  }
  else if (line_seen[2] && line_seen[3]) {
    state = STRAIGHT_TO_CENTER;
    Serial.print("state: "); Serial.println("STRAIGHT TO CENTER");
  }
  else if (line_seen[2] && state != STRAIGHT_TO_CENTER) {
    state = LINE_BL;
    Serial.print("state: "); Serial.println("LINE BL");
  }
  else if (line_seen[3] && state != STRAIGHT_TO_CENTER) {
    state = LINE_BR;
    Serial.print("state: "); Serial.println("LINE BR");
  }
  else if (analogRead(IR_PIN) > IR_THRESHOLD) {
    if (state != ATTACK) {
      Serial.print("state: "); Serial.println("ATTACK");
    }
    state = ATTACK;
  }
}


void setup() {
  // set up pins for line sensors and IR sensor
  for (uint16_t i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }
  pinMode(IR_PIN, INPUT);

  // set up serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {}

  // wait until 5sec have passed before beginning operation
  while (millis() < 5000) {}
  Serial.println("STARTING OPERATION");
}

// test, delete later
unsigned long loop_start;

void loop() {
  loop_start = millis();
  update_state();
  //act();

  Serial.print("loop time: "); Serial.println(millis() - loop_start);
}
