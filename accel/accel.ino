
/*
 ADXL3xx

 Reads an Analog Devices ADXL3xx accelerometer and communicates the
 acceleration to the computer.  The pins used are designed to be easily
 compatible with the breakout boards from Sparkfun, available from:
 http://www.sparkfun.com/commerce/categories.php?c=80

 http://www.arduino.cc/en/Tutorial/ADXL3xx

 The circuit:
 analog 0: accelerometer self test
 analog 1: z-axis
 analog 2: y-axis
 analog 3: x-axis
 analog 4: ground
 analog 5: vcc

 created 2 Jul 2008
 by David A. Mellis
 modified 30 Aug 2011
 by Tom Igoe

 This example code is in the public domain.

*/


const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)

void setup() {
  // initialize the serial communications:
  Serial.begin(9600);
}

void loop() {
  // print the sensor values:
  Serial.print(analogRead(xpin) / 330.0);
  // print a tab between values:
  Serial.print("\t");
  Serial.print(analogRead(ypin) / 330.0);
  // print a tab between values:
  Serial.print("\t");
  Serial.print(analogRead(zpin) / 330.0);
  Serial.println();
  // delay before next reading:
  delay(100);
}
