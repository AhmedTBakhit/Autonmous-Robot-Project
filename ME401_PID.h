//#include <SoftPWMServo.h>
//#include <PID_v1.h>
//
//// Pin definitions
//// DO NOT CHANGE THESE ONES
//const int EncoderAPin = 2;
//const int EncoderBPin = 20;
//const int MotorDirectionPin = 4;
//const int MotorPWMPin = 3;
////3   46  Enable1: OC1/RD0  
////4   59  Direction1: RF1 
//
//// PID tuning gains
//float ku = 9.0; // ultimate gain
//float Tu = 0.45; // seconds
////double kp=0.8*ku,ki=0.0,kd=0.8*ku*Tu*0.125; // classic PD fomr Ziegler Nichols table
//
////double kp=1.0,ki=0.0,kd=0.0; // stupid, simple, poor proportional controller
//double kp = 0.6*ku,ki = 2.0*kp/Tu,kd = kp*Tu*0.125; //classid pid
//
//// Global variables for quadrature decoder
//static volatile char lastLeftA;
//static volatile char lastLeftB;
//static volatile bool errorLeft;
//volatile long position = 0;
//volatile int angle;
//
//// Global variables for the timer interrupt handling
//int pidSampleTime = 10;
//long counterPID=1;
//
//// Global variables for the PID controller
//double input=0, output=0, setpoint=0;
//PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
//
//
//// Forward declaration of functions to be used that are defined later than their first use
//uint32_t TimerCallback(uint32_t currentTime);
//
//void setupPIDandIR(void)
//{
//  // Set up the quadrature inputs
//  pinMode(EncoderAPin, INPUT);
//  pinMode(EncoderBPin, INPUT);
//
//  errorLeft = false;
//  lastLeftA = digitalRead(EncoderAPin);
//  lastLeftB = digitalRead(EncoderBPin);
//
//  // Set up the motor outputs
//  pinMode(MotorPWMPin, OUTPUT);
//  pinMode(MotorDirectionPin, OUTPUT);
//
//  digitalWrite(MotorPWMPin,0);
//  digitalWrite(MotorDirectionPin,0);
//
//  SoftPWMServoPWMWrite(MotorPWMPin, 0);
//
//
//  //Setup the pid 
//  myPID.SetMode(AUTOMATIC);
//  myPID.SetSampleTime(pidSampleTime);
//  myPID.SetOutputLimits(-255,255);
//
//  // Initialize the timer interrupt that decodes the IR beacon signal

//
//}
//
//
//
