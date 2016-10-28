// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=2;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyroIC1(0x68);
MPU6050 accelgyroIC2(0x69);

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

int16_t angleX1, angleY1, angleZ1;
int16_t angleX2, angleY2, angleZ2;

int16_t corrX1, corrY1, corrZ1;
int16_t corrX2, corrY2, corrZ2;

#define LED_PIN 13

#define MPU1 7
#define MPU2 8
#define dt 0.01  //Sampling rate
#define MAX_VAL 32767
#define MIN_VAL -32768

int phase = 0;

bool blinkState = false;

struct accelgyro_values_t
{
  int ax;
  int ay;
  int az;
  int gx;
  int gy;
  int gz;
};

void setup() {

    int dmpStatus1 = 0;
    int dmpStatus2 = 0;
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    //accelgyro.initialize();
    accelgyroIC1.initialize();
    accelgyroIC2.initialize();

    
    

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyroIC1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed");
    Serial.println(accelgyroIC2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed");
    Serial.println(F("Initializing DMP MPU6050 #1..."));
    dmpStatus1 = initializeDmp(accelgyroIC1);
    Serial.println(F("Initializing DMP MPU6050 #2..."));
    dmpStatus2 = initializeDmp(accelgyroIC2);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (phase==0) {
    Serial.println("\nCalculating offsets and calibrating...");
    calibration(accelgyroIC1);
    calibration(accelgyroIC2);
    phase++;
    delay(1000);
  }

  if (phase==1) {
    Serial.println("\nCalibration done.");
    Serial.println("\nStart reading.");
    phase++;
    delay(300);
  }

  if(phase == 2){
    accelgyroIC1.dmpGetQuaternion(&q, fifoBuffer);
    accelgyroIC1.dmpGetEuler(euler, &q);
    Serial.print("euler sensor 1\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);

    accelgyroIC2.dmpGetQuaternion(&q, fifoBuffer);
    accelgyroIC2.dmpGetEuler(euler, &q);
    Serial.print("euler sensor 2\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);
  }
  
  if(phase == 3){
    // read raw accel/gyro measurements from device
    accelgyroIC1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    accelgyroIC2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

   corrX1 = 0.98*(corrX1+gx1*dt)+0.02*(ax1);  //Complimentary filter for MPU6050 sensor 1
   corrY1 = 0.98*(corrY1+gy1*dt)+0.02*(ay1);
   corrZ1 = 0.98*(corrZ1+gz1*dt)+0.02*(az1);

   corrX2 = 0.98*(corrX2+gx2*dt)+0.02*(ax2);  //Complimentary filter for MPU6050 sensor 1
   corrY2 = 0.98*(corrY2+gy2*dt)+0.02*(ay2);
   corrZ2 = 0.98*(corrZ2+gz2*dt)+0.02*(az2);

   angleX1 = readValueToAngle(corrX1);
   angleY1 = readValueToAngle(corrY1);
   angleZ1 = readValueToAngle(corrZ1);

   angleX2 = readValueToAngle(corrX2);
   angleY2 = readValueToAngle(corrY2);
   angleZ2 = readValueToAngle(corrZ2);
   
    // display tab-separated accel/gyro x/y/z values
    Serial.print("MPU1:\t");
    Serial.print(ax1); Serial.print("\t");
    Serial.print(ay1); Serial.print("\t");
    Serial.print(az1); Serial.print("\t");
    Serial.print(gx1); Serial.print("\t");
    Serial.print(gy1); Serial.print("\t");
    Serial.print(gz1);Serial.print("\t");
    Serial.print(angleX1); Serial.print("\t");
    Serial.print(angleY1); Serial.print("\t");
    Serial.println(angleZ1);


    // display tab-separated accel/gyro x/y/z values
    Serial.print("MPU2:\t");
    Serial.print(ax2); Serial.print("\t");
    Serial.print(ay2); Serial.print("\t");
    Serial.print(az2); Serial.print("\t");
    Serial.print(gx2); Serial.print("\t");
    Serial.print(gy2); Serial.print("\t");
    Serial.print(gz2); Serial.print("\t");
    Serial.print(angleX2); Serial.print("\t");
    Serial.print(angleY2); Serial.print("\t");
    Serial.println(angleZ2);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
    
}

int readValueToAngle(int16_t input){
  return map(input,MIN_VAL, MAX_VAL, 0, 180);
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void calibration(MPU6050 accelgyro){
  int ax_offset, ay_offset, az_offset, gx_offset,gy_offset, gz_offset;
  accelgyro_values_t mean_val = meansensors(accelgyro);

  int az_calibration_val = ((MIN_VAL * -1)/2);
  
  ax_offset=-mean_val.ax/8;
  ay_offset=-mean_val.ay/8;
  az_offset=(az_calibration_val-mean_val.az)/8;

  gx_offset=-mean_val.gx/4;
  gy_offset=-mean_val.gy/4;
  gz_offset=-mean_val.gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    mean_val = meansensors(accelgyro);
    Serial.println("...");

    if (abs(mean_val.ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_val.ax/acel_deadzone;

    if (abs(mean_val.ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_val.ay/acel_deadzone;

    if (abs(az_calibration_val-mean_val.az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(az_calibration_val-mean_val.az)/acel_deadzone;

    if (abs(mean_val.gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_val.gx/(giro_deadzone+1);

    if (abs(mean_val.gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_val.gy/(giro_deadzone+1);

    if (abs(mean_val.gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_val.gz/(giro_deadzone+1);

      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_val.ax); 
      Serial.print("\t");
      Serial.print(mean_val.ay); 
      Serial.print("\t");
      Serial.print(mean_val.az); 
      Serial.print("\t");
      Serial.print(mean_val.gx); 
      Serial.print("\t");
      Serial.print(mean_val.gy); 
      Serial.print("\t");
      Serial.println(mean_val.gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset); 
      Serial.print("\t");
      Serial.print(ay_offset); 
      Serial.print("\t");
      Serial.print(az_offset); 
      Serial.print("\t");
      Serial.print(gx_offset); 
      Serial.print("\t");
      Serial.print(gy_offset); 
      Serial.print("\t");
      Serial.println(gz_offset); 
      
    if (ready==6) {  
      Serial.println("\nCalibrating successful!\t");
      break;
    }
    
  }
}

accelgyro_values_t meansensors(MPU6050 accelgyro){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  int ax, ay, az, gx, gy, gz;
  accelgyro_values_t mean_val;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_val.ax=buff_ax/buffersize;
      mean_val.ay=buff_ay/buffersize;
      mean_val.az=buff_az/buffersize;
      mean_val.gx=buff_gx/buffersize;
      mean_val.gy=buff_gy/buffersize;
      mean_val.gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  return mean_val;
}

int initializeDmp(MPU6050 accelgyro){
    int devStatus = accelgyro.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = accelgyro.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false 
    }
}

