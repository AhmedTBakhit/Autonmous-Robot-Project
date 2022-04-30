/*  
 *  Bluetooth Serial communication
 *  Written by Magnus and Ahmed on 3-24-22
 *  For ME401 Spring 22
 *  
 *  Usage:
 *  Use an HC-06 bluetooth slave module to send and receive data from a connected device. 
 *  
 *  Serial commands will be interpreted by this module.
 *  
 *  
 *   
*/


#ifndef _SER_DEBUG_
#define _SER_DEBUG_
#include <SoftwareSerial.h>
#include "main.h"
#include "navsystem.h"
#include "motors.h"
// #include "contServo.h"


const int HC06RX_PIN = 0;
const int HC06TX_PIN = 1;
typedef enum
{
    NONE,
    GOT_A,
    GOT_B,
    GOT_C,
    GOT_D,
    GOT_E,
    GOT_F,
    GOT_G,
    GOT_H,
    GOT_I,
    GOT_L,
    GOT_M,
    GOT_N,
    GOT_P,
    GOT_R,
    GOT_S,
    GOT_T,
    GOT_V,
    GOT_W,
    GOT_X,
    GOT_Y,
    GOT_Z
} states;
states state = NONE;

int current_value;

void initBtSerial();
void process_inc_byte(const byte);
void handle_action();

void initBtSerial(){
  BTSerial.begin(9600);
  BTSerial.print("hello");
}

void handle_action()
{
    // NONE, GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
    switch (state)
    {
    case GOT_A:
        break;
    case GOT_B:
        motors.commandMotors(1, 1);
        delay(current_value);
        motors.commandMotors(0, 0);
        break;
        break;
    case GOT_C:
        break;
    case GOT_D:
        break;
    case GOT_E:
        break;
    case GOT_F:
        BTSerial.println("F");
        motors.commandMotors(-1, -1);
        delay(current_value * 100);
        motors.commandMotors(0, 0);
        break;
    case GOT_G:
        nav.goToPoint(current_value, &currentNavPoint);
        BTSerial.print("currentNavPoint set");
        break;
    case GOT_H:
        motors.setHeadingKp(&motors.hVars, current_value);
        break;
    case GOT_I:
        motors.setVelocityKp(&motors.vVars, current_value);
        break;
    case GOT_L:
        BTSerial.println("L");
        motors.commandMotors(-1, 1);
        delay(current_value * 80);
        motors.commandMotors(0, 0);
        break;
    case GOT_M:
        break;
    case GOT_N:
        break;
    case GOT_P:
        
        break;
    case GOT_R:
        BTSerial.println("R");
        motors.commandMotors(1, -1);
        delay(current_value * 80);
        motors.commandMotors(0, 0);
        break;
        break;
    case GOT_S:
        break;
    case GOT_T:
        BTSerial.println("T received");
        break;
    case GOT_V:
        break;
    case GOT_W:
        break;
    case GOT_X:
        break;
    case GOT_Y:
        break;
    case GOT_Z:
        break;
    default:
        break;
    } // end of switch

    current_value = 0; // since we utilized the current_value above, now we reset it to zero for the next variable
    state = NONE;      // set the state to none, as we have used it
}

void process_inc_byte(const byte c)
{
    if (isdigit(c))
    {
        current_value *= 10;
        current_value += c - '0';
    } // end of digit
    else
    {
        // set the new state if we recognize it
        switch (c)
        {
        // GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
        case ';':
            handle_action();
            break;
        case 'a':
            state = GOT_A;
            break;
        case 'b':
            state = GOT_B;
            break;
        case 'c':
            state = GOT_C;
            break;
        case 'd':
            state = GOT_D;
            break;
        case 'e':
            state = GOT_E;
            break;
        case 'f':
            state = GOT_F;
            break;
        case 'g':
            state = GOT_G;
            break;
        case 'h':
            state = GOT_H;
            break;
        case 'i':
            state = GOT_I;
            break;
        case 'l':
            state = GOT_L;
            break;
        case 'm':
            state = GOT_M;
            break;
        case 'n':
            state = GOT_N;
            break;
        case 'p':
            state = GOT_P;
            break;
        case 'r':
            state = GOT_R;
            break;
        case 's':
            state = GOT_S;
            break;
        case 't':
            state = GOT_T;
            break;
        case 'v':
            state = GOT_V;
            break;
        case 'w':
            state = GOT_W;
            break;
        case 'x':
            state = GOT_X;
            break;
        case 'y':
            state = GOT_Y;
            break;
        case 'z':
            state = GOT_Z;
            break;
        default:
            state = NONE;
            break;
        } // end switch
    }     // end of not digit
}


#endif
