/*   Pyranometer tracker using feed-forward only
 *      
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Feed-forward tracking for pyranometer mount pulls data from GPS unit, calculates solar position, then sends move commands to Zaber T-series rotational stages.
 */

#include <zaberx.h>

#include <LiquidCrystal.h>

#include <TinyGPS++.h>

#include <Sun_position_algorithms.h>
#include <translate.h>

#include <SoftwareSerial.h>

//   solar tracking variables
sunpos SunPos;
float phi;    // azimuthal angle
float theta;  // elevation angle

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

const unsigned int intervalLong = 5000;    // Period of feed-forward iterations

unsigned long currentMillis = 0;
unsigned long longMillis = 0;

int GPSBaud = 4800;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXPin = 2;
int TXPin = 3;
int rsRX = 4;
int rsTX = 5;


LiquidCrystal lcd(8, 9, 10, 11, 12, 13);      // (RS, enable, D4, D5, D6, D7)

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);  

SoftwareSerial rs232(rsRX, rsTX);   //RX, TX

void setup() 
{
  //  Open serial connection with computer
  //Serial.begin(9600);
  
  // begin communication with LCD
  lcd.begin(16, 2);     // (columns, rows)
  
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  //  Start software serial port with Zaber rotational stage
  rs232.begin(9600);  
  delay(1000);

  /*
  replyData = sendCommand(0, renumber, 0);
  delay(1000);
  replyData = sendCommand(0, speedSet, 4551);   // Set speed to 10 degrees/sec
  delay(1000);  
  */
}

void loop()
{
  currentMillis = millis();
        SunPos.UT = gps.time.hour() + double(gps.time.minute())/60.0 + double(gps.time.second())/3600.0 + double(gps.time.centisecond())/360000; // UT in hours [decimal]
        SunPos.Day = gps.date.day(); // day [integer]
        SunPos.Month = gps.date.month(); // month [integer]
        SunPos.Year = gps.date.year(); // year [integer]
        SunPos.Dt = 96.4 + 0.567*double(gps.date.year()-2061); // Terrestial time - UT
        SunPos.Longitude = gps.location.lng() * (2*PI/360.0); // State College Longitude and Latitude [radians]      
        SunPos.Latitude = gps.location.lat() * (2*PI/360.0);
        SunPos.Pressure = 1.0; // Pressure [atm]
        //SunPos.Temperature = imu.readTempC(); // Temperature [C], pulled from LSM303C 6DOF sensor     
        SunPos.Temperature = 20.0;

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(gps.date.month());
          lcd.print('/');
          lcd.print(gps.date.day());
          lcd.print('/');
          lcd.print(gps.date.year());
          lcd.print(' ');
          lcd.print(gps.time.hour());
          lcd.print(':');
          lcd.print(gps.time.minute());
          lcd.setCursor(0, 1);
          lcd.print(gps.location.lat());
          lcd.setCursor(8, 1);
          lcd.print(gps.location.lng());
        
        SunPos.Algorithm5();  

        theta = SunPos.Zenith;
        phi = SunPos.Azimuth + PI;

        azimPos = sendCommand(azimuth, moveAbs, stepsR(phi));
        zeniPos = sendCommand(zenith, moveAbs, stepsR(theta));

        delay(5000);

/*
  //  Feed-forward tracking
  if(currentMillis - longMillis >= intervalLong)
  {
    if(gpsSerial.available() > 0)
    {
      if(gps.encode(gpsSerial.read()))
      {
        longMillis = currentMillis;

      }
    }
  }
  */
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
