/*  
 *  Main header file
 *  Written by Magnus and Ahmed on 3-24-22
 *  For ME401 Spring 22
 *  
 *
 *  
 *  
 *   
*/


#ifndef _MAIN_H_
#define _MAIN_H_
#define MY_ROBOT_ID 2 // CHANGE THIS TO YOUR TEAM NUMBER
#include <SoftwareSerial.h>
#include "navsystem.h"
#include "motors.h"

// These aren't the pin numbers, they are the external interrupt numbers.
// we have to look them up and find out which are available, and then what pin they are found on 
// connect the pins to the COM on the limit switch in question
// connect NO to ground
// connect NO to 3.3v
// the pins will be set as input and will trigger on FALLING, as the limit switch will disconnect V from pin, and connect pin to GND
// ... I think
const int L_LIMIT_EXTINT = 4; // extInt 4 is on pin 35
const int R_LIMIT_EXTINT = 2; // extInt 2 is on pin 7
const int leftServoPin = 32;
const int rightServoPin = 31;
const int GATE_SERVO_PIN = 30; // Servo1: PMD4/RE4   J16 
const int BT_RX = 36;
const int BT_TX = 37;
const int GATE_OPEN_ANGLE =  180;
const int GATE_CLOSED_ANGLE = 90;
volatile bool GATE_STATE = false;
 int ballcaptured = 0;
enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};
enum robotStates robotState = ATTACK;

SoftwareSerial BTSerial(BT_RX, BT_TX);

robotStates prevState;
RobotPose myRobotPose = {true, MY_ROBOT_ID, 500, 500, 0};
NavPoint pn_r;

volatile bool CRASH_FLAG = false;
volatile int CRASH_SIDE = 0; // 0 is left, 1 is right

bool RADIO = true;
bool serialDebug = false; // if true, will output serial debug info
bool btDebug = false; // if false, it will print to serial instead when performing serialDebug



#endif
