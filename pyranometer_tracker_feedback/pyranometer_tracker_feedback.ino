/*   Pyranometer tracker using feedback only   
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Uses the optimization algorithm to keep the pyranometer pointed at the sun.  Attempts to maximize voltage tied to pinPyro; moves the Zaber T-series rotational
 *   stages in small increments while tracking the change in voltage using a "perturb and observe" style optimization approach.
 */

#include <zaberx.h>

#include <SoftwareSerial.h>

//    Feedback variables
int pinPyro = 0;   //Analog pin used to read voltage from transimpedance amp
int voltage = 0;   //value read from transimpedance amp
int previousVoltage = 0;  //voltage value from previous iteration

int dLay = 1000;   //time between incremental movement and photodiode voltage read
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

  //  Feedback tracking
  if(currentMillis - shortMillis >= intervalShort)
  {
    optimize(azimuth, stepsD(0.02));
    optimize(zenith, stepsD(0.02));
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

void optimize(int axis, long increment)
{ 
  //Get starting conditions before optimizing
  voltage = readAnalog(pinPyro, iter8); 
  
  Serial.println(voltage);
  
  //Move one increment in +phi and get new voltage and position
  replyData = sendCommand(axis, moveRel, increment);
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinPyro, iter8);  

  /*
  Serial.print(voltage);
  Serial.print(' ');
  Serial.print(axis);
  Serial.println('+');
  */
  
  //Start optimizing along axis
  if(voltage > previousVoltage)         
  {
    while(voltage > previousVoltage)
    {
      previousVoltage = voltage;
      replyData = sendCommand(axis, moveRel, increment);
      delay(dLay);
      voltage = readAnalog(pinPyro, iter8); 

      /*
      Serial.print(voltage);
      Serial.print(' ');
      Serial.print(axis);
      Serial.println('+'); 
      */ 
    }
  }
  else if(voltage < previousVoltage)
  {
    previousVoltage = voltage;
    replyData = sendCommand(axis, moveRel, (-2)*increment);
    delay(dLay);
    voltage = readAnalog(pinPyro, iter8); 

    /*
    Serial.print(voltage);
    Serial.print(' ');
    Serial.print(axis);
    Serial.println('-');
    */
  
    while(voltage > previousVoltage)
    {        
      previousVoltage = voltage;
      replyData = sendCommand(axis, moveRel, (-1)*increment);
      delay(dLay);
      voltage = readAnalog(pinPyro, iter8);

      /*
      Serial.print(voltage);
      Serial.print(' ');
      Serial.print(axis);
      Serial.println('-');
      */
  
    }
  }  
}
