/*  
 *  IR Distance Calculation Library
 *  Written by Magnus and Ahmed 
 *  Original 3-4-22, updated to servo usage 4-11-22
 *  For ME401 Spring 22
 *  
 *  Hardware:
 *  Pololu Carrier With Sharp GP2Y0A60SZLF Analog Distance Sensor
 *  micro servo 
 *  
 *  Inspiration on calibrating sensor:
 *  https://www.instructables.com/How-to-setup-a-Pololu-Carrier-with-Sharp-GP2Y0A60S/
 *  https://blogs.ntu.edu.sg/scemdp-201415s2-g12/infrared-sensor-and-its-calibration/
 *  
 *  Operation:
 *  While power is on, IR pulses are constantly pulsing. analogRead(IR_OUT) will get
 *  analog value, larger is closer. Sensor is 50-300mm, but is non-linear and must be calibrated.
 *  Calibration values can be stored in this file, 
 *  
 *  Calibration:
 *  Light falls off with the square of the distance, so by testing intensity at known distances, 
 *  relationship between 1/analogRead(IR_OUT) and the distance should be linear. Use calibrateSensor()
 *  to print out serial measurements for calibration.
 *  
*/

#ifndef _IRDISTANCE_H_
#define _IRDISTANCE_H_
#include <Servo.h>
#include "motors.h"

#define IR_OUT A9
//#define SERVO_PIN 30
#define RADIANS 1 // if 1, angle measures will be returned in radian headings (+-)
#define SERVOLIMIT 60 // max angle of the servo sweep
#define SERVO_INCREMENT 10 // number of increments to divide the servo into for IR distance measurements
#define SAMPLE_NUM 3 // how many IR measurements to average over


// dc motor for IR sensor, in J3
const int EncoderAPin = 2;
const int EncoderBPin = 20;
const int MotorDirectionPin = 4;
const int MotorPWMPin = 3;
int irMotorSpeed = 180;


//// Global variables for quadrature decoder
volatile long irPosition = 0;
static volatile char irLastLeftA;
static volatile char irLastLeftB;
static volatile bool irErrorLeft;
volatile int irAngle;

// Global variables for the timer interrupt handling
int irSampleTime = 10;
long irCounter=1;
 PIDVars irVars = {9, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 255, -255};
  
class IrSensor {
  public:
    void init();
    Servo irMotor;
    float distanceSweep();
    void calibrateIrSensor();
    int getDistance(int);
    double scanAreaForGap();
    double pidCalc(PIDVars *vars, double currentError);
    void moveIrSensor(int);
    
  private:
    float getDegreeHeading(int);
    int calDist[10] = {5, 10, 15, 20, 25, 30, 40, 50, 60, 70};
    int getRawDistance(int);
    
    int polyFit(int);
    int powerFit(int);
    float getRadHeading(int);
};

IrSensor irSensor;
 
#endif
