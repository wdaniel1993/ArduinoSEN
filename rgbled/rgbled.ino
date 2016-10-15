/*
 Fade

 This example shows how to fade an LED on pin 9
 using the analogWrite() function.

 The analogWrite() function uses PWM, so if
 you want to change the pin you're using, be
 sure to use another PWM capable pin. On most
 Arduino, the PWM pins are identified with 
 a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

 This example code is in the public domain.
 */

#define RED 3
#define GREEN 5
#define BLUE 6

int brightnessBlue = 0;
int brightnessGreen = 0;
int brightnessRed = 0;

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  randomSeed(analogRead(0));
}

// the loop routine runs over and over again forever:
void loop() {
  setBrightnessLed(RED,random(255));
  setBrightnessLed(GREEN,random(255));
  setBrightnessLed(BLUE,random(255));
  delay(300);
  
}

void setBrightnessLed(int led, int brightnessAdd){
  int brightness = getBrightness(led);
  int newBrightness = brightness + brightnessAdd;
  analogWrite(led, newBrightness);
}

int getBrightness(int led){
  switch(led){
    case RED: return brightnessRed;
    case GREEN: return brightnessGreen;
    case BLUE: return brightnessBlue;
    default: return 0;
  }
}


