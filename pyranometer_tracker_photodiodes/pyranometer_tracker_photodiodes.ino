/*   Closed-loop photodiode-based quadrant solar tracker
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Uses four photodiodes to track the sun.  Intended for coarse tracking for the pyranometer tube.   
 */

#include <zaberx.h>

#include <SoftwareSerial.h>

//    Feedback variables
int topR = 0;       // top right photodiode
int topL = 1;       // top left photodiode
int bottomR = 2;    // bottom right photodiode
int bottomL = 3;    // bottom left photodiode

int vTR;    // voltage from top right photodiode
int vTL;    // voltage from top left photodiode
int vBR;    // voltage from bottom right photodiode
int vBL;    // voltage from bottom left photodiode

int top;      // average of top right and top left voltages
int bottom;   // average of bottom right and bottom left voltages
int right;    // average of top right and bottom right voltages
int left;     // average of top left and bottom left voltages

int dLay = 100;   //time between incremental movement and photodiode voltage read
int iter8 = 100;   //number of reads the photodiode voltage is averaged over

//    Zaber rotational stage variables
byte command[6];
char reply[6];
long replyData;

int azimuth = 1;    // Device ID of azimuth stage
int zenith = 2;     // Device ID of elevation stage

long azimPos = 0;   // Variable which tracks the absolute position of the azimuth stage (in microsteps)
long zeniPos = 0;   // Variable which tracks the absolute position of the elevation stage (in microsteps)

int homer = 1;      // home the stage
int renumber = 2;   // renumber all devices in the chain
int moveAbs = 20;   // move absolute
int moveRel = 21;   // move relative
int stopMove = 23;  // Stop
int speedSet = 42;    // Speed to target = 0.00219727(V) degrees/sec (assuming 64 microstep resolution)
int getPos = 60;      // Query the device for its position
int storePos = 16;    // Position can be stored in registers 0 to 15
int returnPos = 17;   // returns the value (in microsteps) of the position stored in the indicated register
int move2Pos = 18;    // move to the position stored in the indicated register
int reset = 0;        // akin to toggling device power

const unsigned int intervalShort = 1000;   // Period of feedback iterations

unsigned long currentMillis = 0;
unsigned long shortMillis = 0;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXPin = 2;
int TXPin = 3;

SoftwareSerial rs232(RXPin, TXPin);   //RX, TX

void setup() 
{
  //  Open serial connection with computer
  Serial.begin(9600);

  //  Start software serial port with Zaber rotational stage
  rs232.begin(9600);

  //analogReference(EXTERNAL);
  
  delay(1000);

  
  replyData = sendCommand(0, renumber, 0);
  delay(1000);
  replyData = sendCommand(0, speedSet, 4551);   // Set speed to 10 degrees/sec
  delay(1000);
  
}

void loop() 
{
  currentMillis = millis();
  if(currentMillis - previousMillis > intervalShort)
  {
    quadrant(stepsD(0.05));
  }  
}

long sendCommand(int device, int com, long data)
{
   unsigned long data2;
   unsigned long temp;
   unsigned long repData;
   long replyNeg;
   byte dumper[1];
   
   // Building the six command bytes
   command[0] = byte(device);
   command[1] = byte(com);
   if(data < 0)
   {
     data2 = data + quad;
   }
   else
   {
     data2 = data;
   }
   temp = data2 / cubed;
   command[5] = byte(temp);
   data2 -= (cubed * temp);
   temp = data2 / squared;
   command[4] = byte(temp);
   data2 -= (squared * temp);
   temp = data2 / 256;
   command[3] = byte(temp);
   data2 -= (256 * data2);
   command[2] = byte(data2);
   
   // Clearing serial buffer
   while(rs232.available() > 0)
   {
     rs232.readBytes(dumper, 1);
   }
   
   // Sending command to stage(s)
   rs232.write(command, 6);

   delay(20);
   
   // Reading device reply
   if(rs232.available() > 0)
   {
     rs232.readBytes(reply, 6);
   }
   
   repData = (cubed * reply[5]) + (squared * reply[4]) + (256 * reply[3]) + reply[2];

   if(reply[4] == 1)
   {
     repData += 65536;
   }
   
   if(reply[5] > 127)
   {
     replyNeg = repData - quad;
   }

   /*
   // Printing full reply bytes as well as reply data in decimal 
   Serial.print(reply[0]);
   Serial.print(' ');
   Serial.print(reply[1]);
   Serial.print(' ');
   Serial.print(reply[2]);
   Serial.print(' ');
   Serial.print(reply[3]);
   Serial.print(' ');
   Serial.print(reply[4]);
   Serial.print(' ');
   Serial.println(reply[5]);
   Serial.print("\tData:");
   */
   if(reply[5] > 127)
   {
     //Serial.println(replyNeg);
     return replyNeg;
   }
   else
   {
     //Serial.println(repData);  
     return repData;
   }    
}

void quadrant(long increment)
{
  // Read voltages from photodiodes
  vTR = readAnalog(topR, iter8);
  vTL = readAnalog(topL, iter8);
  vBR = readAnalog(bottomR, iter8);
  vBL = readAnalog(bottomL, iter8);

  // Find average values
  top = (vTR + vTL) / 2;
  bottom = (vBR + vBL) / 2;
  right = (vTR + vBR) / 2;
  left = (vTL + vBL) / 2;

  if(top > bottom)
  {
    replyData = sendCommand(zenith, moveRel, increment);
  }
  else if(top < bottom)
  {
    replyData = sendCommand(zenith, moveRel, (-1)*increment);
  }

  if(right > left)
  {
    replyData = sendCommand(azimuth, moveRel, increment)
  }
  else if(right < left)
  {
    replyData = sendCommand(azimuth, moveRel, (-1)*increment);
  }  
}
