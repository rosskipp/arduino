#include <LiquidCrystal.h>
#include <HCSR04.h>

// LCD Setup
// Connections:
// rs (LCD pin 4) to Arduino pin 12
// rw (LCD pin 5) to Arduino pin 11
// enable (LCD pin 6) to Arduino pin 10
// LCD pin 15 to Arduino pin 13
// LCD pins d4, d5, d6, d7 to Arduino pins 5, 4, 3, 2
LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);
int backLight = 13;    // pin 13 will control the backlight

// Initialize sensor that uses digital pins 13 and 12.
int triggerPin = 7;
int echoPin = 8;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{

  // LCD Setup
  pinMode(backLight, OUTPUT);
  digitalWrite(backLight, HIGH); // turn backlight on. Replace 'HIGH' with 'LOW' to turn it off.

  // Serial Setup
  Serial.begin(115200);
  Serial.println("Distance sensor raw data test"); Serial.println("");

  delay(1000);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

  // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
  double distance = distanceSensor.measureDistanceCm();

  // Write to serial for debug
  Serial.print("Distance");
  Serial.println(distance);

  // Write our data to the LCD
  lcd.begin(16,2);              // columns, rows.  use 16,2 for a 16x2 LCD, etc.
  lcd.clear();                  // start with a blank screen
  lcd.setCursor(0,0);           // set cursor to column 0, row 0 (the first row)
  lcd.print("Distance");
  lcd.setCursor(0,1);           // set cursor to column 0, row 1
  lcd.print(distance);
  lcd.print(" cm");

  delay(250);
}
