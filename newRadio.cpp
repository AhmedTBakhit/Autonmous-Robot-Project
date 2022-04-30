#include "newRadio.h"

void Comms::setupRadio()
{
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW  
}


int Comms::getNumRobots()
{
  return numRobots;
}

RobotPose Comms::getRobotPose (int robotID)
{
  RobotPose retval;  
  retval = robotPoses[robotID];
  return retval;
}


void Comms::PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
}


void Comms::printRFMMessage(int len, uint8_t* buff)
{
  Serial.print("RECV: " );
  Serial.print(len);
  Serial.print("    ");
  PrintHex8(buff, len);
  /*for(int i = 0 ; i < len ; i++)
  {
    PrintHex8(buff[i], HEX);
  }*/
  Serial.println("");
}

union {
   int16_t value;
   byte arr[sizeof(int16_t)];
} int16_byte_converter;

int16_t convert_bytes_to_int16(char* buf)
{
  int16_byte_converter.arr[1] = buf[0];
  int16_byte_converter.arr[0] = buf[1];
  return int16_byte_converter.value;
}

void Comms::updateRobotPoseAndBallPositions (void)
{
  /*   Read from the radio to get a packet
   *  (1) Read packets until a $ is found
   *  (2) read packets until a * is found
   *  (3) copy the data into the current double-buffer memory locations 
   *  (4) iterate through the buffer and pick off the position and rotation data. It is an integer for each field.
   */

  bool msg_start_found = false;
  bool msg_end_found = false;
  int idx = 0;
  int error = 0;
  int counter = 0;
  while (error == 0)
  {
    counter += 1;
    if (counter > 1000 && idx == 0){
      //failure to find radio message
      return;
    }
    if (radio.receiveDone()) // Got one!
    {
//      printRFMMessage(radio.DATALEN, (uint8_t*)radio.DATA);
      
      // If the start of a message hasn't been found, look for the '$'
      if (!msg_start_found)
      {
        char firstByte = (char)radio.DATA[0];
        char secondByte = (char)radio.DATA[1];
        if (firstByte == '$' && secondByte == '$') 
        {
          msg_start_found = true;
          idx = 0;
        }       

        char lastByte = (char)radio.DATA[radio.DATALEN-1];
        char secondLastByte = (char)radio.DATA[radio.DATALEN-2];
        if (msg_start_found ==true && lastByte == '*' && secondLastByte == '*')
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          //Serial.println("FOUND END BYTE SAME PACKET");
          msg_start_found = false;
          break;
        }
        else if (msg_start_found == true)
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          idx += radio.DATALEN;
          continue;
        }
      }    
      else if (msg_start_found && radio.DATALEN >= 2)
      {
        // Check to see if the message has ended
        char lastByte = (char)radio.DATA[radio.DATALEN-1];
        char secondLastByte = (char)radio.DATA[radio.DATALEN-2];
        if (lastByte == '*' && secondLastByte == '*')
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          //Serial.println("FOUND END BYTE OTHER PACKET");
          msg_start_found = false;
          break;
        }
        // If the message hasn't ended, copy all the data into the buffer
        else
        {       
          //Serial.print("MSG RCVD IDX: ");
          //Serial.println(idx);
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          idx += radio.DATALEN;        
        }
      }
    }   
  }
  
  for(int i = 0 ; i < NUM_ROBOTS ; i++)
  {
    robotPoses[i].valid = false;
  }
  
  int start_offset = 2;
  numRobots = convert_bytes_to_int16(&lbuf[start_offset + 0]);
  for(int i = 0 ; i < numRobots ; i++)
  {
    int bidx = start_offset + 2 + i*12;
    int16_t ID = convert_bytes_to_int16(&lbuf[bidx + 0]); //lbuf[bidx + 0] << 8 | lbuf[bidx + 1];
    int16_t  X = convert_bytes_to_int16(&lbuf[bidx + 2]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];
    int16_t  Y = convert_bytes_to_int16(&lbuf[bidx + 4]); //lbuf[bidx + 4] << 8 | lbuf[bidx + 5];
    int16_t  R = convert_bytes_to_int16(&lbuf[bidx + 6]);
    int16_t  Z = convert_bytes_to_int16(&lbuf[bidx + 8]);
    int16_t  H = convert_bytes_to_int16(&lbuf[bidx + 10]);

    robotPoses[ID].valid = true;
    robotPoses[ID].ID = ID;
    robotPoses[ID].x = X;
    robotPoses[ID].y = Y;
    robotPoses[ID].theta = R;     
  }

  int nidx = start_offset + 2 + numRobots*12;
  numBalls = convert_bytes_to_int16(&lbuf[nidx+0]);
  
  for(int i = 0 ; i < numBalls ; i++)
  {
    int bidx = nidx + 2 + i*6;
    int16_t X = convert_bytes_to_int16(&lbuf[bidx + 0]); //lbuf[bidx + 0] << 8 | lbuf[bidx + 1];
    int16_t Y = convert_bytes_to_int16(&lbuf[bidx + 2]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];
    int16_t H = convert_bytes_to_int16(&lbuf[bidx + 4]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];

    ballPositions[i].x = (float)X;
    ballPositions[i].y = (float)Y;
//    ballPositions[i].hue = H;
  }
 
}

void Comms::ME401_Radio_initialize(void)
{
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
}

bool Comms::validPose(RobotPose pose, int myId){
  if (pose.ID == 0 || pose.ID == myId){
    return false;
  } else{
    return true;
  }

}
