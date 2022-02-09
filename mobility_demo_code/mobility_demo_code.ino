#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include "math.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

//SoftwareSerial serial(10,11);

RoboClaw roboclaw(&Serial2,0);

union{long l; uint32_t u;}unl;
union{long l; uint32_t u;}unr;

enum State {attack, start, search_turn, search_forward, online};
State state = start;

boolean done_start = false;

#define PPR 5120
#define TPR 20480
#define WHEEL_DIA 0.04
#define ADDRESS 0x80
#define WHEEL_DISTANCE 0.134
#define LINESENSORCOUNT 4
#define LINESENSORTHRESHOLD 300
#define LIDARPIN A5
#define LIDARTHRESHOLD 400
#define BACK_UP_DIST 0.25
#define BACK_UP_ANGLE 4.712
#define LEFT_DIR 1
#define RIGHT_DIR -1

int32_t frontLineSeen = 0;
int32_t backLineSeen = 0;

int32_t turn_time = random(100,400)/100;

uint8_t depth1 = 0;
uint8_t depth2 = 0;
uint32_t encL = 0;
uint32_t encR = 0;
uint32_t vl_vr[2]={0,0};


float angular_rate = 1.5;
float linear_rate = 0.5;

int32_t state_change = millis();
float turn = 0;
float forward = 0;
float pwm_value;
boolean lineFront = 0;
boolean lineBack = 0;
int count = 0;
boolean is_front_line = false;
int32_t start_of_match = millis();

int lineSensorPins[LINESENSORCOUNT] = {A0,A1,A2,A3};
int32_t lineSensorValues[LINESENSORCOUNT] = {0,0,0,0};

void setup() {

  //Set up radio reading
  //pinMode(2, INPUT);
  //spinMode(3, INPUT);
  
  //set up line sensor reading
  for(int32_t i = 0; i < LINESENSORCOUNT; i++){
    pinMode(lineSensorPins[i], INPUT);
  }
  pinMode(LIDARPIN, INPUT);
 
  // put your setup code here, to run once:  
  Serial3.begin(115200);
  Serial3.println("hello world");
  Serial.begin(115200);  
  while(!Serial){}  
  Serial.println("Serial Init");
  roboclaw.begin(115200);
  
  
//  roboclaw.SetConfig(ADDRESS, 0x00A0);
//  delay(1000);
//  Serial2.end();
//  roboclaw.begin(115200);
  roboclaw.SetM1VelocityPID(ADDRESS, .04573, .00311, 0, 1006312);
  roboclaw.SetM1PositionPID(ADDRESS, 75.51887, 0, 475.47365, 0, 0, 0, 0);
  Serial.println("PID SET");
  delay(50);
  roboclaw.SetM2VelocityPID(ADDRESS, .03991, .00327, 0, 1031625);
  roboclaw.SetM1PositionPID(ADDRESS, 75.51887, 0, 475.47365, 0, 0, 0, 0);
  roboclaw.ResetEncoders(ADDRESS);
  start_of_match = millis();
}

// d = distance(m)
uint32_t to_pulses(float distance){
  float rev = distance/(WHEEL_DIA*M_PI);
  float pulses = rev * PPR * 4;
  pulses = (uint32_t)pulses;
  return(pulses);
}

void linear_angular_velo(float linear, float angular, uint32_t (&vl_vr)[2]){
  float vl;
  float vr;
  float radius = 0.0;
  if(angular == 0){
    vl = linear;
    vr = linear;
  } else {
    radius = linear/angular;
    
    if(radius < (WHEEL_DISTANCE/4)){
     radius = (WHEEL_DISTANCE/4); 
    }
    
    vl = angular*(radius+(WHEEL_DISTANCE/2));
    vr = angular*(radius-(WHEEL_DISTANCE/2));
  }
  vl_vr[0] = to_pulses(vl);
  vl_vr[1] = to_pulses(vr);
}

void checkLineSensors(){
    for(int32_t i = 0; i < LINESENSORCOUNT; i++){
      int32_t sensorReadValue = analogRead(lineSensorPins[i]);
      lineSensorValues[i] = sensorReadValue;
      if((i==0 || i==1) && sensorReadValue<LINESENSORTHRESHOLD){
        state = online;
        if(state != online){
          state_change = millis();
          roboclaw.ResetEncoders(ADDRESS);
        }
        (!done_start? done_start = true: done_start = false);
        lineFront = 1;
        frontLineSeen = millis();
        is_front_line = true;
        //get_off_line(true);
      } else if ((i == 2 || i == 3) && sensorReadValue<LINESENSORTHRESHOLD){
        state = online;
        if(state != online){
          state_change = millis();
          roboclaw.ResetEncoders(ADDRESS);
        }
        (!done_start? done_start = true: done_start = false);
        lineBack = 1;
        backLineSeen = millis();
        is_front_line = false;
        //get_off_line(false);
      } 
    }
}

void get_off_line(boolean is_front_line){
  int32_t multiplier = 1;
  if(is_front_line){
    multiplier = -1;
  }
  
  linear_angular_velo(multiplier * .2, 0, vl_vr);
  unl.u = vl_vr[0];
  unr.u = vl_vr[1];
  int32_t start = millis();
  roboclaw.SpeedM1M2(ADDRESS, unl.u, -unr.u);
  while(millis()-start < 2000){}
  roboclaw.SpeedM1M2(ADDRESS, 0, 0);
}

void sendVelo(uint32_t vl, uint32_t vr){
    unl.u = vl;
    unr.u = vr;
    if(!lineBack && (unl.l <= 0 || unr.l <= 0)){
        roboclaw.SpeedM1M2(ADDRESS, vl, -vr);
    } else if ( unl.l <= 0 || unr.l <= 0){
      //Serial.println("Moving backward would be dangerous");
      //get_off_line(false);
      roboclaw.SpeedM1M2(ADDRESS, 0, 0);
    }
    
    
    // if line in front and we want to go forward, dont!
    if (!lineFront && (unl.l >= 0 || unr.l >= 0)){
      roboclaw.SpeedM1M2(ADDRESS, vl, -vr);
    } else if (unl.l >= 0 || unr.l >= 0){
      //Serial.println("Moving forward would be dangerous");
      //get_off_line(true);
      roboclaw.SpeedM1M2(ADDRESS, 0, 0);
    }
}

void updateEncoders() {
   encL = LEFT_DIR * rc.ReadEncM1(RC_ADDR);
   encR = RIGHT_DIR * rc.ReadEncM2(RC_ADDR);
}

float distTraveled() {
    return radius * M_PI * (encR + encL) / TPR;
}

float diffTraveled() {
  return WHEEL_DIA * M_PI * (encR - encL) / TPR;
}

float alphaTraveled() {
  return 2 * diffTraveled() / WHEEL_DISTANCE;
}

void update_state(){
  if(analogRead(LIDARPIN)>LIDARTHRESHOLD && state != online){
  //if(0==1){
    if(state != attack){
      state_change = millis();
    }
    state = attack;
    
    (!done_start? done_start = true: done_start = false);
  } else if(!done_start && millis()-start_of_match < ((2.0*M_PI*1000)/angular_rate)){
    state = start;
//  } else if((state == online && millis()-state_change > 1000*(0.25/linear_rate)) || (state == search_turn && millis()-state_change < (turn_time*M_PI*1000/angular_rate))){
//    if(state != search_turn){
//      state_change = millis();
//      // yes its random between 1 and 4 pi, sorry mikey
//      turn_time = random(100,400)/100;
//    }
//    state = search_turn;
//    (!done_start? done_start = true: done_start = false);
  } else if ((state == online && fabs(distTraveled()) > BACK_UP_DIST) || (state == search_turn && fabs(alphaTraveled()) < BACK_UP_ANGLE)) {
    if (state != search_turn) {
      state_change = millis();
      roboclaw.ResetEncoders(ADDRESS);
    }
    state = search_turn;
    (!done_start? done_start = true: done_start = false);
  } else if (state == search_turn && millis()-state_change >= (turn_time*M_PI*1000/angular_rate)) {
    if(state != search_forward){
      state_change = millis();
    }
    state = search_forward;
    (!done_start? done_start = true: done_start = false);
  } //else if (state == online && millis()-state_change <= 1000*(0.25/linear_rate)){
    else if (state == online && fabs(distanceTraveled()) > BACK_UP_DIST) {
    //if (state == online && millis()-state_change <= 2000){
    state = online;
  } else {
    state = search_forward;
  }
}

void act(){
  if(state == start){
    Serial.println("start");
    forward = 0.0;
    turn = angular_rate;
  } else if (state == online){
    Serial.println("online");
    int32_t multiplier = 1;
    if(is_front_line){
      multiplier = -1;
    }
    forward = multiplier * linear_rate;
    turn = 0.0;
  } else if (state == attack) {
    Serial.println("attack");
    forward = linear_rate;
    turn = 0.0;
  } else if (state == search_turn){
    Serial.println("search_turn");
    forward = 0.0;
    turn = angular_rate;
  } else if (state == search_forward){
    Serial.println("search_forward");
    forward = linear_rate;
    turn = 0.0;
  }
}

void loop() {
    int32_t start_of_loop = millis();
    // updates all states
    update_state();
    // !!!!!!!!!sets state if line sensor is seen to state = online, so if u change update_state make sure this gets chenged too if it needs to be. =
    checkLineSensors();
    // set forward and backward
    act();
    
    Serial.println(forward);
    Serial.println(turn);
    linear_angular_velo(forward, turn, vl_vr);
    unl.u = vl_vr[0];
    unr.u = vl_vr[1];
    
    roboclaw.SpeedM1M2(ADDRESS, vl_vr[0], -vl_vr[1]);
    //sendVelo(vl_vr[0], vl_vr[1]);
    
    // if line behind and we want to go backwards, dont!
    if(lineFront && millis() - frontLineSeen > 50){
      lineFront = 0;
    }
    if(lineBack && millis() - backLineSeen > 50){
      lineBack = 0;
    }
    int32_t time = millis()-start_of_loop;
    //Serial.println(analogRead(LIDARPIN));
    //Serial.println(time);
}
