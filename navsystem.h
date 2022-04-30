    /*  
    *  Navigation systems and functions
    *  Written by Magnus and Ahmed 
    *  4-4-22, modified 4-11-22
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

 
#ifndef _NAVSYSTEM_H_
#define _NAVSYSTEM_H_
// #include "navlist.h"
#include "newRadio.h"
//#include "ME401_Radio.h"

#define M_PI 3.14159
#define CLOSE_ENOUGH 200
//#define OBSAVOID_OFFSET 75 // offset to avoid obstacles in our path
const float OBSAVOID_OFFSET = 75.0;
const int WORRYDISTANCE = 800; // how close should an obstacle be before we worry about avoiding it?
const int ROBOBUMPER = 200; // what is the radius of our robobumper, ie 2*ROBOBUMPER gap should allow robot to pass through without clipping. this include radius of obs.
const int ARENA_MIN = -250;
const int ARENA_MAX = 2500;
const int BASE_RADIUS = 500;

class NavPoint {
    public:
        NavPoint(float, float);
        float x; 
        float y;
        NavPoint();
};

class NavSystem
{  
  public:
    bool closeEnough(RobotPose robot, NavPoint point);
    void update(NavPoint*);
    void CountBalls();
    NavPoint getNavPointFromBallPos(BallPosition ballPos);
    NavPoint findNearestBall();
    void setHomeBase(RobotPose);
    bool checkWaypointStatus();
    void editNavPoint(NavPoint*, float, float);
    NavPoint getPnr(NavPoint, RobotPose);
    double getHeadingRelRobot(NavPoint);
    double getDistanceRelRobot(NavPoint);
    void goToPoint(int, NavPoint*);
    void getNextNavPoint(NavPoint*); //pass currentNavPoint to getNext and it will assign the new coordinates
    void addNavPoint(NavPoint);
    NavPoint getPnw(NavPoint navpoint, RobotPose robot);
    void checkPathToGoal(NavPoint* currentNavPoint);
    void depositTheCash();
    NavPoint getClosestObstacleInPath();

  private:
    int currNav = 0;
    int maxNav = 10;
    bool waypointReached = false;
    double convDegRads(float);
    double convRadDegs(float);
};

NavPoint navList[10] = {{500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0}
                        };

NavPoint currentNavPoint = {500.0, 500.0};

// assign goalPoint a position when have a ball as a target, or the home base when in capture
NavPoint goalPoint = {1000, 1000};
NavPoint home_base = {100.0, 100.0};

NavSystem nav;


#endif /* _NAVSYSTEM_H_ */
