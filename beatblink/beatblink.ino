//www.elegoo.com
//2016.06.13

int  sensorPin  =  A0;     // select the input  pin for  the potentiometer 
int  ledPin  =  13;   // select the pin for  the LED
int  sensorValue =  0;  // variable to  store  the value  coming  from  the sensor

void setup()
{
Serial.begin(9600);
}
void loop(){
  sensorValue =  analogRead(sensorPin);
  Serial.println(sensorValue,  DEC);
}
