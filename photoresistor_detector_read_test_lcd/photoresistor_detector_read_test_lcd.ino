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

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);      // (RS, enable, D4, D5, D6, D7)

void setup() 
{
  //  Begin serial connection with LCD
  lcd.begin(16, 2);     // (columns, rows)
  delay(1000);  
}

void loop() 
{
  vTR = readAnalog(topR, iter8);
  vTL = readAnalog(topL, iter8);
  vBR = readAnalog(bottomR, iter8);
  vBL = readAnalog(bottomL, iter8);

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
  
  delay(500);
}
