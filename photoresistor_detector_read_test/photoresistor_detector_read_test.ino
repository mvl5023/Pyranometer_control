/*   Closed-loop LDR-based quadrant solar tracker
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Uses four CdS photoresistors to track the sun.  Intended for coarse tracking for the pyranometer tube.   
 */

//    Feedback variables
int topR = 0;       // top right photoresistor
int topL = 1;       // top left photoresistor
int bottomR = 2;    // bottom right photoresistor
int bottomL = 3;    // bottom left photoresistor

int vTR;    // voltage from top right photoresistor
int vTL;    // voltage from top left photoresistor
int vBR;    // voltage from bottom right photoresistor
int vBL;    // voltage from bottom left photoresistor

int iter8 = 100;   //number of reads the photoresistor voltage is averaged over

void setup() 
{
  //  Open serial connection with computer
  Serial.begin(9600);  
  delay(1000);  
}

void loop() 
{
  vTR = readAnalog(topR, iter8);
  vTL = readAnalog(topL, iter8);
  vBR = readAnalog(bottomR, iter8);
  vBL = readAnalog(bottomL, iter8);

  Serial.print("Top Right: ");
  Serial.print(vTR);
  Serial.print("\tTop Left: ");
  Serial.print(vTL);
  Serial.print("\tBottom Right: ");
  Serial.print(vBR);
  Serial.print("\tBottom Left: ");
  Serial.println(vBL);

  delay(1000);
}

