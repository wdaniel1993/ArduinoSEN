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
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"

#define dt 0.01
#define MAX_VAL 32767
#define MIN_VAL -32768
//#define ENABLE_CALIBRATION

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 2;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

Quaternion q1;
Quaternion q2;
Quaternion q;
float euler1[3]; 
float euler2[3]; 

int phase = 0;
unsigned long timer = millis();

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

struct accelgyro_t {
  MPU6050 sensor;
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t dmpReady;
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  int interruptNumber;
  void (*interruptFunction)();
} accelGyro1, accelGyro2;
///////////////////////////////////   FUNCTIONS   ////////////////////////////////////

#ifdef ENABLE_CALIBRATION
void calibration(MPU6050 accelgyro) {
  int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
  accelgyro_values_t mean_val = meansensors(accelgyro);

  int az_calibration_val = ((MIN_VAL * -1) / 2);

  ax_offset = -mean_val.ax / 8;
  ay_offset = -mean_val.ay / 8;
  az_offset = (az_calibration_val - mean_val.az) / 8;

  gx_offset = -mean_val.gx / 4;
  gy_offset = -mean_val.gy / 4;
  gz_offset = -mean_val.gz / 4;
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    mean_val = meansensors(accelgyro);
    Serial.println("...");

    if (abs(mean_val.ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_val.ax / acel_deadzone;

    if (abs(mean_val.ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_val.ay / acel_deadzone;

    if (abs(az_calibration_val - mean_val.az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (az_calibration_val - mean_val.az) / acel_deadzone;

    if (abs(mean_val.gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_val.gx / (giro_deadzone + 1);

    if (abs(mean_val.gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_val.gy / (giro_deadzone + 1);

    if (abs(mean_val.gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_val.gz / (giro_deadzone + 1);

    if (ready == 6) {
      Serial.println("\nCalibrating successful!\t");
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
      break;
    }

  }
}

accelgyro_values_t meansensors(MPU6050 accelgyro) {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  int ax, ay, az, gx, gy, gz;
  accelgyro_values_t mean_val;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_val.ax = buff_ax / buffersize;
      mean_val.ay = buff_ay / buffersize;
      mean_val.az = buff_az / buffersize;
      mean_val.gx = buff_gx / buffersize;
      mean_val.gy = buff_gy / buffersize;
      mean_val.gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  return mean_val;
}
#endif

int initializeDmp(accelgyro_t* accelgyro) {
  int devStatus = accelgyro->sensor.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    accelgyro->sensor.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt)..."));
    //attachInterrupt(accelgyro->interruptNumber, accelgyro->interruptFunction, RISING);
    accelgyro->mpuIntStatus = accelgyro->sensor.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    // get expected DMP packet size for later comparison
    accelgyro->packetSize = accelgyro->sensor.dmpGetFIFOPacketSize();
    accelgyro->dmpReady = true;
    return true;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    accelgyro->dmpReady = false;
    return false;
  }
}
int tryReadLastQuaternion(accelgyro_t* accelGyro){
  
  accelGyro->mpuIntStatus = accelGyro->sensor.getIntStatus();
  // get current FIFO count
  accelGyro->fifoCount = accelGyro->sensor.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((accelGyro->mpuIntStatus & 0x10) || accelGyro->fifoCount == 1024 ) {
      // reset so we can continue cleanly
      accelGyro->sensor.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  }else if (accelGyro->mpuIntStatus & 0x02) {
      while (accelGyro->fifoCount < accelGyro->packetSize) {
        accelGyro->fifoCount = accelGyro->sensor.getFIFOCount();
      }
      accelGyro->sensor.getFIFOBytes(accelGyro->fifoBuffer, accelGyro->packetSize);
      accelGyro->sensor.dmpGetQuaternion(&q, accelGyro->fifoBuffer);
      accelGyro->sensor.resetFIFO();
      accelGyro->fifoCount = 0;
      return true;
  }
  return false;
}

void setOffsets(MPU6050 accelgyro, int ax_offset, int ay_offset, int az_offset, int gx_offset, int gy_offset, int gz_offset){
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
}

double calculateAngle(Quaternion q1, Quaternion q2){
  q1.normalize();
  q2.normalize();
  return acos(dotProductQuaternions(q1,q2)) / PI * 180;
}

double dotProductQuaternions(Quaternion left, Quaternion right) {
  return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
}


void setup() {

  accelGyro1.sensor = mpu1;
  accelGyro2.sensor = mpu2;

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  //accelgyro.initialize();
  accelGyro1.sensor.initialize();
  accelGyro2.sensor.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelGyro1.sensor.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed");
  Serial.println(accelGyro2.sensor.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed");

}

void loop() {
  int res = false;
  int newAngle = false;
  
  if (phase == 0) {
    Serial.println("\nCalculating offsets and calibrating...");
    #ifdef ENABLE_CALIBRATION
    calibration(accelGyro1.sensor);
    calibration(accelGyro2.sensor);
    #else
    setOffsets(accelGyro1.sensor, 1422,1897,870,80,50,59);
    setOffsets(accelGyro2.sensor, 927,-1049,586,234,-11,-23);
    #endif
    phase++;
    delay(1000);
  }

  if (phase == 1) {
    Serial.println("\nCalibration done.");
    Serial.println(F("Initializing DMP MPU6050 #1..."));
    initializeDmp(&accelGyro1);
    Serial.println(F("Initializing DMP MPU6050 #2..."));
    initializeDmp(&accelGyro2);
    phase++;
    delay(300);
    accelGyro1.sensor.resetFIFO();
    accelGyro2.sensor.resetFIFO();
  }

  if (phase == 2) {
    
    if (!(accelGyro1.dmpReady && accelGyro2.dmpReady )) return;
      res = tryReadLastQuaternion(&accelGyro1);
      if(res) {
        q1 = q;   
        newAngle = true;
      }
      res = tryReadLastQuaternion(&accelGyro2);
      if(res){
        q2 = q;
        newAngle = true;
      }

      if(newAngle && (millis()-timer > 500)){
        Serial.print("quat1\t");
        Serial.print(q1.w);
        Serial.print("\t");
        Serial.print(q1.x);
        Serial.print("\t");
        Serial.print(q1.y);
        Serial.print("\t");
        Serial.println(q1.z);
        Serial.print("quat2\t");
        Serial.print(q2.w);
        Serial.print("\t");
        Serial.print(q2.x);
        Serial.print("\t");
        Serial.print(q2.y);
        Serial.print("\t");
        Serial.println(q2.z);
        Serial.print("Angle: ");
        Serial.println(calculateAngle(q1,q2));
        newAngle = false;
        timer = millis();
      }
  }
}
