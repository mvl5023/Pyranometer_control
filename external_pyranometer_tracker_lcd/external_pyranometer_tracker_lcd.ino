/*   Closed-loop LDR-based quadrant solar tracker
 *    Hitachi HD44780 LCD output
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Uses four CdS photoresistors to track the sun.  Intended for coarse tracking for the pyranometer tube.   
 */
 
#include <LiquidCrystal.h>

#include <zaberx.h>

#include <SoftwareSerial.h>

//    Feedback variables
int topR = 0;       // top right photoresistor
int topL = 1;       // top left photoresistor
int bottomR = 2;    // bottom right photoresistor
int bottomL = 3;    // bottom left photoresistor

int dLay = 200;   //time between incremental movement and photoresistor voltage read
int iter8 = 100;   //number of reads the photoresistor voltage is averaged over

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

const unsigned int intervalShort = 200;   // Period of feedback iterations

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXPin = 4;
int TXPin = 5;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);      // (RS, enable, D4, D5, D6, D7)

SoftwareSerial rs232(RXPin, TXPin);   //RX, TX

void setup() 
{
  // begin communication with LCD
  lcd.begin(16, 2);     // (columns, rows)

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
    previousMillis = currentMillis;
    quadrant(stepsD(0.2));
  }  
}

long sendCommand(int device, int com, long data)
{
   unsigned long data2;
   unsigned long temp;
   unsigned long repData;
   long replyNeg;
   float replyFloat;
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
   
   replyFloat = (cubed * float(reply[5])) + (squared * float(reply[4])) + (256 * float(reply[3])) + float(reply[2]); 
   repData = long(replyFloat);
   
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
  // Find voltages from photoresistor voltage divider
  int vTR = readAnalog(topR, iter8);   // voltage from top right photoresistor
  int vTL = readAnalog(topL, iter8);    // voltage from top left photoresistor
  int vBR = readAnalog(bottomR, iter8);    // voltage from bottom right photoresistor
  int vBL = readAnalog(bottomL, iter8);    // voltage from bottom left photoresistor

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TL:");
  lcd.print(vTL);
  lcd.setCursor(8, 0);
  lcd.print("TR:");
  lcd.print(vTR);
  lcd.setCursor(0, 1);
  lcd.print("BL:");
  lcd.print(vBL);
  lcd.setCursor(8, 1);
  lcd.print("BR:");
  lcd.print(vBR);

  // Find average values
  int top = (vTR + vTL) / 2;      // average of top right and top left voltages
  int bottom = (vBR + vBL) / 2;   // average of bottom right and bottom left voltages
  int right = (vTR + vBR) / 2;    // average of top right and bottom right voltages
  int left = (vTL + vBL) / 2;     // average of top left and bottom left voltages

  if(top > bottom)
  {
    replyData = sendCommand(zenith, moveRel, (-1)*increment);
  }
  else if(top < bottom)
  {
    replyData = sendCommand(zenith, moveRel, increment);
  }

  if(right > left)
  {
    replyData = sendCommand(azimuth, moveRel, increment);
  }
  else if(right < left)
  {
    replyData = sendCommand(azimuth, moveRel, (-1)*increment);
  }  
}

