#include <SoftwareSerial.h>

byte command[6];
byte reply[6];

int homer = 1;
int renumber = 2;
int moveAbs = 20;
int moveRel = 21;
int stopMove = 23;
int speedSet = 42;    // Speed to target = 0.00219727(V) degrees/sec (assuming 64 microstep resolution)
int getPos = 60;
int storePos = 16;    // Position can be stored in registers 0 to 15
int returnPos = 17;
int move2Pos = 18;
int reset = 0;

#define quad 4294967296
const long cubed = 16777216;
const long squared = 65536;

const float resolutionDeg = 0.000234375;
const float resolutionRad = 0.000004091;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 3;
int TXpin = 4;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

void setup() 
{
  Serial.begin(9600);
  rs232.begin(9600);
  delay(2000);
  sendCommand(0, renumber, 0);
  delay(2000);
  sendCommand(0, homer, 0);
  delay(5000);
}

void loop() 
{
  sendCommand(0, moveAbs, stepsD(10));
  delay(2500);
  sendCommand(0, moveAbs, stepsD(20));
  delay(2500);
}

long stepsD(float degr)
{
  long stepValue;
  stepValue = degr / resolutionDeg;
  return stepValue;
}

long stepsR(float radis)
{
  long stepValue;
  stepValue = radis / resolutionRad;
  return stepValue;
}

void sendCommand(int device, int com, long data)
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
     data +=  quad;
   }
   temp = data / cubed;
   command[5] = byte(temp);
   data -= (cubed * temp);
   temp = data / squared;
   command[4] = byte(temp);
   data -= (squared * temp);
   temp = data / 256;
   command[3] = byte(temp);
   data -= (256 * data);
   command[2] = byte(data);

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
     replyData -= quad;
   }

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
   Serial.print(reply[5]);
   Serial.print("\tData:");
   Serial.println(replyData);   
}

