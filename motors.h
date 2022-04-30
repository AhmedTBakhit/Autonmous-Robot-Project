    /*  
    *  Motors class and functions for controlling the robot movement in space
    *  Written by Magnus and Ahmed  on 3-26-22
    *  Modified 4-6-22
    *  For ME401 Spring 22 miderm robot project
    * 
    * 
    * Structs:
    *   - RobotVars: I was thinking of usign this as a container that could be updated
    *       by functions whenever variables changed, so I could send a single struct
    *       by serial and get a great deal of information out instead of printing out
    *       csv type values. I ahven't finished it yet. 
    * 
    *   - PIDVars: a struct containing useful information about the PID variables, used
    *       to keep track of integral, limits, k gains, etc. Each PID controller needs
    *       its own vars. hVars is heading, vVars is velocity PID controlle vars.
    * 
    * Classes:
    *   - motors, Motors(): the class and the object of the motors. Keeps motor
    *       functions organized.
    *       Functions:
    *           - setHeadingKp(): convenience function for setting Kp for heading
    *               via serial debug. Intended to be used to control the tightness of
    *               turns by dynamically modifying Kp of heading PID controller.
    *               
    *           - setVelocityKp(): same, but for velocity. 
    *           - commandMotors(double, double): Take a -1 to +1 value and turn that
    *               into the 0 - 180 values we feed to the Servo class for each motor.
    *           - pidCalc(PIDVars*, double): returns a double corresponding to PID
    *               output. Does all the calculations, using the variables passed in
    *               as PIDVars, and the input being the error. 
    * 
    *           - update(NavPoint navpoint, bool debug):  This function does too much.
    *               It is the function responsible for keeping the robot moving toward
    *               its goal. It should be refactored. 
    *               Currently it takes in the goal navpoint and then:
    *                   1.  uses navsystem functions to calculate the position, 
    *                   heading and distance of the navpoint relative to the robot
    *                   2. calculates and normalizes error to 0-1 range
    *                   3. feeds that error to the piDCalc() functions
    *                   4. takes output and passes it to commandMotors()
    *                 
    *               
    * 
    */


#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <Arduino.h>
#include "main.h"
#include "navsystem.h"
#include <Servo.h>


#define maxInput 1.0
#define minInput -1.0



struct RobotVars{
  float v;
  float l;
  float r;
  float tD;
  float tA;
  NavPoint pn_r;
  float currentDist;
  float desiredHeading;
  float angleAdj;
} robotVars;


struct PIDVars{
  // PID controller params
  double Kp;
  double Ki;
  double Kd;
  
  // stuff we need to keep track of
  double _integral;
  double _prevError;
  double _dT;
  double _currentError;
  double _setPoint;
  
  // max output limit for the controller
  double maxLimit;
  double minLimit;
};

void openGate(bool);

class Motors {
    public:
        void update(NavPoint, bool);
        void commandMotors(double, double);
        double pidCalc(PIDVars*, double);
        void setHeadingKp(PIDVars*, int);
        void setVelocityKp(PIDVars*, int);
        PIDVars vVars = {1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, -100};
        PIDVars hVars = {2, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, -100};
        Servo leftServo;
        Servo rightServo;
        float distTh = 25;
        float rotTh = 0.01;
        double distError = 0.0;
        double velocity = 0.0;
        double rotError = 0.0;
        double angleAdj = 0.0;
        int leftServoDC;
        int rightServoDC;
        
        bool reverseMotors = false;

        

};

//class IrMotor{
//  public:
//    void init();
//    Servo irMotor;
//    void moveIrSensor(int);
//    
//};

Motors motors;
Servo gateServo; 
//IrMotor irMotor;

#endif
