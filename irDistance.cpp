#include "irDistance.h"

uint32_t irCallback(uint32_t currentTime) {
  char irNewLeftA = digitalRead(EncoderAPin);
  char irNewLeftB = digitalRead(EncoderBPin);

  irPosition += (irNewLeftA ^ irLastLeftB) - (irLastLeftA ^ irNewLeftB); 

  if((irLastLeftA ^ irNewLeftA) & (irLastLeftB ^ irNewLeftB))
  {
    irErrorLeft = true;
  }

  irLastLeftA = irNewLeftA;
  irLastLeftB = irNewLeftB;

  if (irCounter % 100*irSampleTime == 0)
  {
    irAngle = irPosition * 0.133;     

    double output = irSensor.pidCalc(&irVars, irAngle);

    if (output > 0)
    {
      digitalWrite(MotorDirectionPin,1);
    }
    else
    {
      digitalWrite(MotorDirectionPin,0);
    }  
    irSensor.moveIrSensor(abs(output));
    
    irCounter = 0;
  }
  irCounter++;
  
  return (currentTime + CORE_TICK_RATE/100);
}

void IrSensor::init(){
  pinMode(MotorPWMPin, OUTPUT); // dc motor PWM using servo library
  pinMode(MotorDirectionPin, OUTPUT); // direction pin
  digitalWrite(MotorPWMPin,0);  // controls speed
  digitalWrite(MotorDirectionPin,0); // controls direction
  irMotor.attach(MotorPWMPin);
  irMotor.write(0);
  pinMode(IR_OUT, INPUT);
  attachCoreTimerService(irCallback);
}

double IrSensor::scanAreaForGap(){
  // Korey's scan algorithm
  //Scan the area around the obstacle to search for routes around it if any
  //Seek ‘empty’ direction, if two open paths go right
  // so the scanAreaForGap should return a double, corresponding to an angle from the robot that looks more open
  int minAngle = 0; // I'm making these up, I haven't used the servo yet
  int maxAngle = 180;
  double bestAngle = 0; // holds the best angle corresponding to the highest distance
  int highestDistance = 0; // the best really sucks to begin with so anything it finds is okay
  int stupidCloseWeAreStillBlocked = 400; // how far away is stupid close? Change it to something sensible
  
  for (int i = minAngle; i < maxAngle; i++){
    // this code sucks because its going to iterate through ALL the angles from -90 to 90. Maybe a set number of angles to check?
    // or have the angle be an i value multiplied by something? i don't know here is i * 30.
    int angle = i * 30;
    Serial.print("irSensorAngle: ");
    Serial.print(angle);
    Serial.print(", distance to obstacle: ");
    int obsDist = getDistance(angle); // get the distance at this angle
    Serial.println(obsDist);
    
    if (obsDist > highestDistance){
      // if the measured distance is more than our current highest, its more open so we mark it as our current bestAngle
      highestDistance = obsDist;
      bestAngle = angle;
    }
  }

  if (highestDistance < stupidCloseWeAreStillBlocked){
    // uh oh we're really close on even the best one, so I guess we're still blocked
    // tell it that the best angle is straight backwards
    Serial.println("stupid close");
    return -180;
  } else {
    // if the best angle is open enough, ie greater than our stupid close variable, 
    Serial.print("best angle is :");
    Serial.println(bestAngle);
    return bestAngle;
  }
};

void IrSensor::calibrateIrSensor(){
  // mean of 10 values for each distance
  for (int i = 0; i < 10; i++){
    int sumVal = 0;
    Serial.print(calDist[i]);
    delay(4000);
    Serial.print(", ");
    delay(1000);
    for (int j = 0; j<10; j++){
      sumVal += getRawDistance(90);
      delay(100);
    }

    Serial.println((float)sumVal / 10.0);
  }
}


void IrSensor::moveIrSensor(int desiredAngle){
  irVars._setPoint = desiredAngle;
}
  
int IrSensor::getRawDistance(int theta){
  moveIrSensor(theta);
  int sumVal = 0;
  for (int i = 0; i < SAMPLE_NUM; i++){
    sumVal += analogRead(IR_OUT); 
  }
  int irVal = sumVal / SAMPLE_NUM;
  return irVal;
}

int IrSensor::getDistance(int theta){
  int irVal = getRawDistance(theta);
  int calDist = polyFit(irVal);
  return calDist;
}

int IrSensor::powerFit(int irVal){
   // y = 13722x-1.106
  float calDist = 13722.0 * pow((float)irVal, -1.106);
  return (int)calDist;
}

int IrSensor::polyFit(int irVal){
  //y = -3E-07x3 + 0.0007x2 - 0.5821x + 173.8
  
  float rawVal = (float)irVal;
  float calDist = -0.0000003 * pow(rawVal, 3) + 0.0007 * pow(rawVal, 2) -0.5821 * rawVal + 173.8;
  return (int)calDist;
}

float IrSensor::getRadHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float radHeading = ((float)bestTheta - (float)SERVOLIMIT / 2.0) * 0.01745;
  return radHeading; 
}

float IrSensor::getDegreeHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float degreeHeading = (float)bestTheta - ((float)SERVOLIMIT / 2.0);
  return degreeHeading;
}



double IrSensor::pidCalc(PIDVars *vars, double currentError){
  vars->_integral = 0;
  double propError = (vars->Kp) * currentError;
  float dampError = (vars->Kd) * ((currentError - (vars->_prevError))); // derivative of last time step
  float intError = (vars->Ki) * vars->_integral;

  double output = propError + dampError + intError;
  vars->_prevError = currentError;
  
  if (output >(vars->maxLimit))
    output = vars->maxLimit;
  else if (output < (vars->minLimit))
    output = vars->minLimit;

  return output;
}
