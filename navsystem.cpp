    /*  
    *  Navigation systems and functions
    *  Written by Magnus and Ahmed on 4-4-22
    *  Modified 4-6-22
    *  For ME401 Spring 22 miderm robot project
    * 
    *  Structs: 
    *  - NavPoint: a pair of xy coordinates
    *  
    *  Variables:
    *   - home_base: a NavPoint corresponding to the robots home base location, used for 
    *              returning a captured ball.
    *   - test_course: an array of NavPoints, for testing a path following behavior. 
    *   - nav: an object of class NavSystem, for utilizing the nav functions.
    *   - navList: an array of NavPoints I am using to store multiple NavPoints to 
    *       define a path. 
    *   - currNav, maxNav: variables that help index the lenght of navList, ensuring
    *       that undefined behavior doesn't occur when iterating through the array. 
    *   - currentNavPoint: a variable that holds the current goal position of the robot.
    *       We modify the robot's destination by assigning new x/y coordinates to this 
    *       single navPoint. 
    * 
    *  Classes:
    *   NavSystem: The NavSystem class holds a number of variables and functions used 
    *   for navigation of the robot, including helper functions for calculating angles, 
    *   and NavPoint edit functions to change coordinates.
    * 
    *  Usage:
    *  - setHomeBase(currentRobotPose): called during setup, changes the coordinates of home_base NavPoint 
    *              to the corner of the quadrant that the robot starts up in. 
    * 
    *  - getPnr(&currentNavTarget): get position of NavPoint wrt robot frame. Uses navpoint (can be ball, 
    *            whatever, just a point in space) and robot pose to calculate the position 
    *          of the navpoint in relation to the robot frame.
    *           Returns navPoint with coordinates of the P in relation to robot frame.
    *
    *   - update(&currentNavTarget): Called every loop as part of checkStatus(), checks if the waypointreached
    *            flag is true. If it is true, then get the next NavPoint coordinates and 
    *            reset flag.
    *
    *   - getHeadingRelRobot(p_nr): Given a position of a NavPoint relative to the 
    *       robot's position, returns the heading in radians wrt the robot.
    * 
    *   - getDistanceRelRobot(p_nr): Given a position of a NavPoint relative to the
    *       robot's position, return the Euclidean distance in millimeters in relation
    *       to the robot. 
    * 
    *   - getNextNavPoint(&currentNavTarget): Given the current NavPoint pointer, 
    *       assign the coordinates of the next NavPoint in the series to it. 
    *   
    *   - convDegRads/ convRadsDeg(): helper functions I used for troubleshooting. Does
    *       simple conversion between rads and degrees and back again for testing.
    * 
    *   - addNavPoint(): stub, doesn't do anything.
    * 
    *   - goToPoint(&currentNavTarget, int): like editNavPoint, it takes the pointer for
    *       the currentNavTarget and modifies the x any y values. However, instead of
    *       geting those values from a NavPoint in an array, it takes an integer and 
    *       turns it into coordinates. This is mostly used in the btserial as a 
    *       convenience, so that destinations can be sent over serial for remote 
    *       control purposes. 
    */



#include "navsystem.h"
#include <Arduino.h>

#include "btserial.h"

NavPoint::NavPoint(float x1, float y1){
    x = x1;
    y = y1;
}

NavPoint::NavPoint(){
    x = 0.0;
    y = 0.0;
}

//
//void NavSystem::update(NavPoint *cnp){
//    /* Check if the robot has reached our destination. If that flag is true, 
//    *  modify the currentnavTarget's coordinates to the next waypoint. 
//    *
//    */
//    if (waypointReached == true){
//        getNextNavPoint(cnp);
//        waypointReached = false;
//    }
//
//}



void NavSystem::editNavPoint(NavPoint *nextPoint, float x, float y){
    /* Function to alter the coordinates of a nav point directly. 
    */

  nextPoint->x = x;
  nextPoint->y = y;
}


NavPoint testCourse[] = {
                          {250.0, 250.0}, 
                          {1750.0, 250.0}, 
                          {1750.0, 1750.0}, 
                          {250.0, 1750.0},
                        };

void NavSystem::setHomeBase(RobotPose myStartPose){
  // automatically set your home base coordinates according to what quadrant your robot starts up in
  
//    if (myStartPose.x < 1100.0){
//        if (myStartPose.y > 1100.0){
//            // home base is in upper left quadrant
//            editNavPoint(&home_base,250.0, 1850.0);
//        } else {
//            // home base is in lower left quadrant
//            editNavPoint(&home_base,250.0, 250.0);
//        }       
//    } else {
//        if (myStartPose.y > 1000.0){
//            // home base is in upper right quadrant
//            editNavPoint(&home_base,1850.0, 1850.0);
//        } else {
//            // home base is in lower right quadrant
//            editNavPoint(&home_base,1850.0, 250.0);
//        }
   // }

} //setHomeBase();

NavPoint NavSystem::getPnw(NavPoint navpoint, RobotPose robot){
      double theta = (float)robot.theta / 1000.0; // convert from the int16_t format to double
      double c = cos(theta);
      double s = sin(theta);
      
      // rotation matrix
      //      R = np.matrix([[c, s], [-s, c]])
      double x1 = c * navpoint.x + s * navpoint.y;
      double y1 = -s * navpoint.x + c * navpoint.y;

      // define new NavPoint
      double px = x1 + robot.x;
      double py = y1 + robot.y;
      NavPoint pnw = {px, py};
      
      return pnw;
}

NavPoint NavSystem::getPnr(NavPoint navpoint, RobotPose robot){
    /* get position of NavPoint wrt robot frame
     *  Uses navpoint (can be ball, whatever, just a point in space) and robot pose to
     *  calculate the position of the navpoint in relation to the robot frame
     *  Returns a navpoint of the navpoint in relation to the robot frame. 
     */
      double theta = (float)robot.theta / 1000.0; // convert from the int16_t format to double
      double c = cos(theta);
      double s = sin(theta);

      // Pnr = Rrw' * Pnw  - Rrw' * Prw
      // rotation matrix
      double x1 = c * navpoint.x + s * navpoint.y;
      double y1 = -s * navpoint.x + c * navpoint.y;

      // translation matrix
      double x1t = c * robot.x + s * robot.y;
      double y1t = -s * robot.x + c * robot.y;

      // define new NavPoint
      double px = x1 - x1t;
      double py = y1 - y1t;
      NavPoint pnr = {px, py};
      
      return pnr;
}

double NavSystem::getHeadingRelRobot(NavPoint pn_r){
      // get heading from robot to relative navpoint in rads
      double hRads = atan2(pn_r.y, pn_r.x);
      return hRads;
}

double NavSystem::getDistanceRelRobot(NavPoint pn_r){
  /* Calculate the distance from a navpoint relative to the robot
   */
   double distance = sqrt(pn_r.x * pn_r.x + pn_r.y * pn_r.y);
   
   return distance;
}


double NavSystem::convDegRads(float degree){
  double radian = degree * (M_PI / 180.0);
  return radian;
}

double NavSystem::convRadDegs(float radian){
  double degree = radian / (M_PI / 180.0);
  return degree;
}


void NavSystem::goToPoint(int xy, NavPoint* currentNavPoint){
   // 1020.0140 / 10000 = 
    float x = xy / 10000;
    float y = xy - (x * 10000);
    
    currentNavPoint->x = x;
    currentNavPoint->y = y;
}



void NavSystem::getNextNavPoint(NavPoint *oldNavPoint){

    currNav += 1;
    if (currNav > maxNav){
      /// just start over again
      currNav = 0;
    }
    NavPoint newNavPoint = navList[currNav];
    oldNavPoint->x = newNavPoint.x;
    oldNavPoint->y = newNavPoint.y;
    if (serialDebug == true){
        BTSerial.print(oldNavPoint->x);
        BTSerial.print(", ");
        BTSerial.println(oldNavPoint->y);
    }

  }
  
void NavSystem::addNavPoint(NavPoint navpoint){
}


NavPoint NavSystem::getClosestObstacleInPath(){
  /*  Goes through all robot positions and find the closest obstacle in the 
   * 
   */
   NavPoint closestObs = {5000.0, 5000.0};  
    int closestID = 255;
    int closestDist = 3000;
    NavPoint Po_r; // position of obstacle relative to robot
    
   for (int i = 0; i < 40; i++){  
      // iterate through the robots
      RobotPose pose = robotPoses[i];

      // check to see if pose is a valid object on the board. ignore self. 
      if (comms.validPose(pose, MY_ROBOT_ID) == true){

        // if pose exists on the board, get its nav point
        NavPoint poseNavPoint = {pose.x, pose.y};

        // get position of object wrt robot frame
        Po_r = nav.getPnr(poseNavPoint, myRobotPose); 
//        Serial.print("pose: ");
//        Serial.print(i);
//        Serial.print(", wrtR (");
//        Serial.print(Po_r.x);
//        Serial.print(", ");
//        Serial.print(Po_r.y);
//        Serial.println(")");

        // if object is in front of us, and close enough to take into account
        if (Po_r.x < WORRYDISTANCE && Po_r.x > 0){
             // check to see if its in our direct path ahead        
             if (Po_r.y > -ROBOBUMPER && Po_r.y < ROBOBUMPER){
              
              // if this is the case, its blocking so find out how far away it is
              int obsDist = getDistanceRelRobot(Po_r);
              if  (obsDist < closestDist){
                 closestID = pose.ID;
                 closestObs.x = Po_r.x;
                 closestObs.y = Po_r.y;
                 closestDist = obsDist; // this is the newest closest obstacle
            }
        }
      } 
  }
}
  return closestObs;
}

void NavSystem::checkPathToGoal(NavPoint* currentNavPoint){
    /* Ahmed and Magnus's path checking algorithm
   *  Checks to see if the path is blocked by looking at the obstacles between robot and goal.
   *  If it finds an obstacle that is in the danger zone, it takes the closest one and tries to avoid it 
   *  by the shortest path. 
   */

  // get NavPoint for goal wrt robot
  NavPoint Pg_r = getPnr(goalPoint, robotPoses[MY_ROBOT_ID]);

  // get navpoint for closest obstacle in path wrt robot
  NavPoint Po_r = getClosestObstacleInPath();

  // get distance to closest obstacle
  double pDist = getDistanceRelRobot(Po_r);

  // if distance to robot is less than a certain value, we need to avoid it
  // so choose the best distance to go
  if (pDist < WORRYDISTANCE){
      // if obstacle is > 0, its on our left. So we go right. 
      if (Po_r.y >= 0){
        Po_r.y = Po_r.y - OBSAVOID_OFFSET;
        Po_r.x = 600; // forward and to the right
      } else {
        // if obstacle is < 0, its on our right. so we go left.
        Po_r.y = Po_r.y + OBSAVOID_OFFSET;
        Po_r.x = 600; // forward, and to the left
      }
  
      // 
      
      // get world coordinates
      NavPoint Po_w = getPnw(Po_r, robotPoses[MY_ROBOT_ID]);
      
      editNavPoint(currentNavPoint, Po_w.x, Po_w.y);
  } else{
    // if there isn't an obstacle all that close, we can move towards our goal point
      editNavPoint(currentNavPoint, goalPoint.x, goalPoint.y);
  }
} // checkPath();

NavPoint NavSystem::getNavPointFromBallPos(BallPosition ballPos){
  // turn a ballPosition into a nav point for ease of coding
  NavPoint ballNavPoint;
  ballNavPoint.x = ballPos.x;
  ballNavPoint.y = ballPos.y;
  return ballNavPoint;
}

NavPoint NavSystem::findNearestBall(){
  // iterate through all the balls, and return the closest one as our target
  NavPoint closestBall;
  double closestBallDist = 3000;

  if (numBalls == 0){
    // if there are no balls, we don't need to go to a ball, so we leave the nav point unchanged
    closestBall.x = myRobotPose.x;
    closestBall.y = myRobotPose.y;
    return closestBall;
  }
  
  for (int i = 0; i < numBalls; i++){
    // iterate through the balls, checking to see if they are within the arena bounds and thus valid targets
    NavPoint ballPos = getNavPointFromBallPos(ballPositions[i]);
    if (ballPos.x > ARENA_MIN && ballPos.x < ARENA_MAX && ballPos.y > ARENA_MIN && ballPos.y < ARENA_MAX){

        // calculate distance from home base so we can check if they are our balls
        double distFromHomeBase = sqrt(pow((ballPos.x -  home_base.x), 2) + pow((ballPos.y - home_base.y), 2));
        
        if (distFromHomeBase > BASE_RADIUS){         
          NavPoint Pb_r = getPnr(ballPos, myRobotPose);
          double ballDist = getDistanceRelRobot(Pb_r);
          
          if (ballDist < closestBallDist){
            closestBallDist = ballDist;
            closestBall.x = ballPos.x;
            closestBall.y = ballPos.y;
          }
        }
    }
  }
  return closestBall;
}

bool NavSystem::closeEnough(RobotPose robot, NavPoint point){
  // if distance between robot and point is close enough(tm), return true
  
  double distFromPoint = sqrt(pow((robot.x -  point.x), 2) + pow((robot.y - point.y), 2));
  
  if (distFromPoint < CLOSE_ENOUGH){
    return true;
  } else{
    return false;
  }
}

void NavSystem::depositTheCash(){
  /* We're in capture mode, and we reached home base. Now open the gate, shake that booty and back up to deposit the balls.
   */
  openGate(true); // open gate
  delay(1000); // wait for gate to open
  motors.commandMotors(1, 1); // back up 
  delay(1500);
  openGate(false);// close the gate
  BTSerial.println("depositTheCash");
}

void NavSystem::CountBalls()
{
  static long prevTime=0 ;
  long CurrentTime = millis();
  if ((CurrentTime - prevTime) > 6000)
  {
    prevTime = CurrentTime;
    ballcaptured ++;
    Serial.print("ballCaptured: ");
    Serial.println(ballcaptured);
}

}
