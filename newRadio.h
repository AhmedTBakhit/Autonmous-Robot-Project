
#ifndef _ME401_RADIO_H_
#define _ME401_RADIO_H_

#include <RFM69.h>
#include <SPI.h>

#define NETWORKID     101   // Must be the same for all nodes (0 to 255)
#define MYNODEID      2   // My node ID (0 to 255)
#define TONODEID      0   // Destination node ID (0 to 254, 255 = broadcast)
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define USEACK        false // Request ACKs or not
#define NUM_ROBOTS 40
#define NUM_BALLS 20
#define RADIO_LED 6

const int RF69_CHIPKIT_IRQ = 38;
char lbuf[1024];
int numRobots = 0;
int numBalls = 0;
  
struct RobotPose
{
  boolean valid;
  int16_t ID;
  int16_t x;
  int16_t y;
  int16_t theta;
};

struct BallPosition
{
  int16_t x = 500;
  int16_t y = 500;
};

class Comms {
  public:
    int getNumRobots();
    void updateRobotPoseAndBallPositions();
    RobotPose getRobotPose (int robotID);
    BallPosition getClosestBallPos();
    void ME401_Radio_initialize(void);
    bool validPose(RobotPose, int);
    
  private:
    void setupRadio();
    void PrintHex8(uint8_t *data, uint8_t length);
    void printRFMMessage(int len, uint8_t* buff);     
};

RFM69 radio(RF69_SPI_CS, RF69_CHIPKIT_IRQ, true);
Comms comms;

RobotPose robotPoses[NUM_ROBOTS];
BallPosition ballPositions[NUM_BALLS];

#endif
