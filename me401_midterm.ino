#include "main.h"
//#include "ME401_Radio.h"
#include "ME401_PID.h"
#include "btserial.h"
#include "irDistance.h"
#include "navsystem.h"
#include "newRadio.h"


void attack(){
    // this sets the goalpoint we are trying to get to. this is used during the checkPath function to set currentNavPoint
    goalPoint = nav.findNearestBall(); 

    // get the position of the nearest ball
    NavPoint pnr = nav.getPnr(goalPoint , myRobotPose);
   
   if(pnr.x > 0 && pnr.x < 400 && pnr.y > -100 && pnr.y < 100){
      // if the ball is close enough, open the gate
      openGate(true);
  
      // and increment our score counter
      nav.CountBalls();
   }
   else{
    // if there isn't a ball right in front of us, close the gate
      openGate(false);
   }
}

void defend(){
//  Serial.println("DEFEND"); 
  goalPoint = home_base;
}

void capture(){  
  // set our goalPoint to home base
//  int loop = 0;
//  if(loop = 0)
//  {
//   openGate(true);
//  }
  
  goalPoint = home_base;

  // check if we have reached our base
  bool baseReached = nav.closeEnough(myRobotPose, goalPoint);

  if (baseReached == true){
    //drop the payload
    nav.depositTheCash();

    // reset state to ATTACK so we can go for more balls
    robotState = ATTACK;
    ballcaptured = 0;
  }
  
}

void checkStatus(){
    
    // update all robot, ball and global obstacle positions with radio data
    comms.updateRobotPoseAndBallPositions();

    // assign our robot's information to a robotPose so we can use it easily
    myRobotPose = comms.getRobotPose(MY_ROBOT_ID);

    // update the navigation systems data
    //    nav.update(&currentNavPoint);
   
}

void handleState(){
//  Serial.println("handleState()");
   switch (robotState) {
    case 0:
      // Attack
      attack();
      break;
    case 1:
      // Capture, return to base with ball
      capture();
      break;
    case 2:
      // Defend
      defend();
      break;
    case 3:
      // test functionality, not defined at this time
      break;
    default:
      // no state, switch to attack
      robotState = ATTACK;
      break;
  }
}

void changeState(){
if (ballcaptured >= 3)
{
  robotState = CAPTURE;
}
}



void setup() {
  Serial.begin(115200);
  Serial.print("serial begin");
  
  irSensor.init(); // Sharp IR distance sensor initialize
  Serial.print(", ir sensor online");

  if (RADIO == true){
    comms.ME401_Radio_initialize(); // Initialize the RFM69HCW radio  
    checkStatus();
    Serial.print(", radio online");
  } else {
    Serial.print(", offline mode");
  }

  initBtSerial();
  Serial.println(", btserial online");

  // attach motor servo pins so they actually work when we command them
  motors.leftServo.attach(leftServoPin);
  motors.rightServo.attach(rightServoPin);
  gateServo.attach(GATE_SERVO_PIN);
  
  // set the home base coordinates to the corner of the arena we start in
  nav.setHomeBase(robotPoses[MY_ROBOT_ID]);
  Serial.print(", home_base set to (");
  Serial.print(home_base.x);
  Serial.print(", ");
  Serial.print(home_base.y);
  Serial.print(")");
  
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
//  setupPIDandIR();
  
  // serial output every 2s
  Serial.println("Serial output every 2s on ");
  attachCoreTimerService(btDebugCallback);
  
//  // update the goal position every 500 ms
//  attachCoreTimerService(updateCallback);
//  Serial.println("coreTimer attached");

  // attach the external interrupts for the limit switches
  attachInterrupt(L_LIMIT_EXTINT, handleCrashL, FALLING);
  attachInterrupt(R_LIMIT_EXTINT, handleCrashR, FALLING);
  Serial.println("limit switches activated");
}

void handleCrashL(){
  CRASH_FLAG = true;
  CRASH_SIDE = 0;
}

void handleCrashR(){
  CRASH_FLAG = true;
  CRASH_SIDE = 1;
}

void loop() {

  // check the BTSerial for instructions
  while (BTSerial.available())
  {
      process_inc_byte(BTSerial.read());
  }

  // update the environment and change states as required
   if (RADIO == true){
      checkStatus(); // check the status of the game environment
      changeState(); // check to see if a state change is called for
      handleState(); // execute current state function
   }

   // resolve any crashes with local obstacles
   while (CRASH_FLAG == true){      
      CRASH_FLAG = crashState(CRASH_SIDE);
   } 
  
  // check to see if we have a clear path to our goal point
  // if not, we will adjust our course to the side of an obstacle
  nav.checkPathToGoal(&currentNavPoint);

  // update the relative position of the nav point
  pn_r = nav.getPnr(currentNavPoint, myRobotPose);

  // are we close enough to our currentNavPoint? if so, motor output will be zero for this loop
  bool areWeThereYet = nav.closeEnough(myRobotPose, currentNavPoint);

  // use navpoint and robot pose to calculate PID changes needed and modify motor output
  motors.update(pn_r, areWeThereYet); 

   
} // end loop()

uint32_t btDebugCallback(uint32_t currentTime) {
      float currentDist = nav.getDistanceRelRobot(pn_r);
      float desiredHeading = nav.getHeadingRelRobot(pn_r);
      float x = currentNavPoint.x;
      float y = currentNavPoint.y;
      String outputBuf = (String)millis() + "," +
                          "RobotState: " +
                          (String)robotState + "," +
                          (String)x + "," + 
                          (String)y + "," + 
                          (String)currentDist + "," + 
                          (String)motors.velocity  + "," + 
                          (String)desiredHeading  + "," + 
                          (String)motors.angleAdj + ";";
      if (btDebug == true){
        BTSerial.println(outputBuf);
      } 
      if (serialDebug == true){
        Serial.println(outputBuf);
      }

  return (currentTime + CORE_TICK_RATE * 1000);
}

bool crashState(bool crashSide){
  /* Korey's local obstacle avoidance logic, 4-11-22
   * This function is used when a limit switch triggers and the CRASH_FLAG flag is true.
   * If an obstacle occurs via the crash state, the IR sensor is then activated. 
   * Once activated the robot is to back out and scan the area searching for a new safe path.
   */
   int howLongToBackUp = 1500; // ms that we back up to clear obstacle. How long should it be?
   int r_motor_val;
   int l_motor_val;
   Serial.print("crash state, side: ");
   Serial.println(crashSide);
  if (crashSide == 0){
    // that means we collided on the left. how do we use that info?
    // Should we back up a slightly different direction?
    l_motor_val = 0;
    r_motor_val = 1;
  } else {
    // if crashSide is 1, that means we collided on the right. how do we use that info?
    r_motor_val = 0;
    l_motor_val = 1;
  }
  //The robot should back out of the obstacle area
  motors.commandMotors(1,1); // back up according to values from the crashSide check
  delay(howLongToBackUp); // wait for a few ms to get far enough back
  motors.commandMotors(0, 0); // stop motors for scanning
 
  //Scan the area around the obstacle to search for routes around it if any
  //Seek ‘empty’ direction, if two open paths go right
  // so the scanAreaForGap should return a double, corresponding to an angle from the robot that looks more open
  // the code for this function is in irDistance.cpp
//  double emptyDir = -180;
//  emptyDir = irSensor.scanAreaForGap();
//  Serial.print("irSensor:");
//  Serial.println(emptyDir);
//  //If all directions are blocked back out further
//  if (emptyDir == -180){
//    // scanAreaForGap returns -180 if there are no distances greater than a threshold
//    // This means we're still blocked, so we should back up and try again. We do that by leaving the
//    // crashed flag true so this function will be called again
//    return true;
//  }
//
c
//  // now that we have a proper open angle, how should we proceed? Do we pick a navpoint in that direction? or just use 
//  // commandMotors to turn that far, and how do we know how far we turned?
//  // I'm not sure yet how we do this. 
//  
//  // return false to clear CRASH_FLAG so we continue normal movement
  return false;
}
