// I2Cdev library collection - MPU9150 I2C device class
// Based on InvenSense MPU-9150 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
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
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MPU9150.h"

#define ACCEL_FILTER_GAIN (float)1/3.414213562
#define C1 (float)0.4142135624

//filter stuff

/** Default constructor, uses default I2C address.
 * @see MPU9150_DEFAULT_ADDRESS
 */
MPU9150::MPU9150() {
    devAddr = MPU9150_DEFAULT_ADDRESS;
    V = 0;
    bias = 0;
    V_Error = 0;
    mh_Error = 0;
    pitch_Error = 0;
    roll_Error = 0;
    for(int i =0;i<3;i++)
    {
      lastG[i] = 0;
      for(int j=0;j<2;j++)
      {
        xA[i][j] = yA[i][j] = 0; //initializing things from 0
      }
    }
}

/** Specific address constructor.
 * @param address I2C address
 * @see MPU9150_DEFAULT_ADDRESS
 * @see MPU9150_ADDRESS_AD0_LOW
 * @see MPU9150_ADDRESS_AD0_HIGH
 */
// MPU9150::MPU9150(uint8_t address) {
//     devAddr = address;
// }

void MPU9150::setAddress(uint8_t address)//default constructor for initializing I2C address.
{
  devAddr = address;
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
bool MPU9150::initialize() //initialize the gyro with the apt scaling factors.
{
    setClockSource(MPU9150_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU9150_GYRO_FS_1000);
    long timeout = micros();
    while(getFullScaleGyroRange() != MPU9150_GYRO_FS_1000)
    {
      setFullScaleGyroRange(MPU9150_GYRO_FS_1000); //keep trying 
      if(micros() - timeout>1000000)
        break;
    }
    setFullScaleAccelRange(MPU9150_ACCEL_FS_2);
    while(getFullScaleAccelRange() != MPU9150_ACCEL_FS_2 )
    {
      setFullScaleAccelRange(MPU9150_ACCEL_FS_2);
      if(micros() - timeout>1000000)
        break;
    }
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    pinMode(MPU_LED,OUTPUT);
    return testConnection();
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU9150::testConnection() {
    return getDeviceID();//== 1;
}

void MPU9150::setClockSource(uint8_t source) {
    I2Cdev::writeBits(devAddr, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CLKSEL_BIT, MPU9150_PWR1_CLKSEL_LENGTH, source);
}

void MPU9150::setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_SLEEP_BIT, enabled);
}

void MPU9150::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU9150_RA_GYRO_CONFIG, MPU9150_GCONFIG_FS_SEL_BIT, MPU9150_GCONFIG_FS_SEL_LENGTH, range);
}

uint8_t MPU9150::getFullScaleGyroRange() {
    I2Cdev::readBits(devAddr, MPU9150_RA_GYRO_CONFIG, MPU9150_GCONFIG_FS_SEL_BIT, MPU9150_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}

void MPU9150::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU9150_RA_ACCEL_CONFIG, MPU9150_ACONFIG_AFS_SEL_BIT, MPU9150_ACONFIG_AFS_SEL_LENGTH, range);
}

uint8_t MPU9150::getFullScaleAccelRange() {
    I2Cdev::readBits(devAddr, MPU9150_RA_ACCEL_CONFIG, MPU9150_ACONFIG_AFS_SEL_BIT, MPU9150_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}

uint8_t MPU9150::getDeviceID() {
    I2Cdev::readBits(devAddr, MPU9150_RA_WHO_AM_I, MPU9150_WHO_AM_I_BIT, MPU9150_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

void MPU9150::readMag()
{
  byte buf[6];
  I2Cdev::writeByte(devAddr,0x37,0x02);
  I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
  I2Cdev::readBytes(0x0C, 0x03, 7, buf); // get 6 bytes of data
  m[1] = (((int16_t)buf[1]) << 8) | buf[0]; // the mag has the X axis where the accelero has it's Y and vice-versa
  m[0] = (((int16_t)buf[3]) << 8) | buf[2]; // so I just do this switch over so that the math appears easier to me. 
  m[2] = (((int16_t)buf[5]) << 8) | buf[4];
}

void MPU9150::readIMU()
{
  Wire.beginTransmission(devAddr);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(devAddr,14); //request 14 bytes from mpu
  //300us for all data to be received. 
  //each value in the mpu is stored in a "broken" form in 2 consecutive registers.(for example, acceleration along X axis has a high byte at 0x3B and low byte at 0x3C 
  //to get the actual value, all you have to do is shift the highbyte by 8 bits and bitwise add it to the low byte and you have your original value/. 
  a[0]=Wire.read()<<8|Wire.read();  
  a[1]=Wire.read()<<8|Wire.read(); 
  a[2]=Wire.read()<<8|Wire.read(); 
  t=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  g[0]=Wire.read()<<8|Wire.read();  
  g[1]=Wire.read()<<8|Wire.read();
  g[2]=Wire.read()<<8|Wire.read();
}

void MPU9150::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2Cdev::readBytes(devAddr, MPU9150_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void MPU9150::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, uint16_t* mx, uint16_t* my, uint16_t* mz) {

    //get accel and gyro
    getMotion6(ax, ay, az, gx, gy, gz);

    //read mag
    I2Cdev::writeByte(devAddr, MPU9150_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
    delay(10);
    I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer);
    *mx = (((uint16_t)buffer[0]) << 8) | buffer[1];
    *my = (((uint16_t)buffer[2]) << 8) | buffer[3];
    *mz = (((uint16_t)buffer[4]) << 8) | buffer[5];
}

void MPU9150::blink(int8_t n)
{
  for(int8_t i=0;i<n;i++)
  {
    digitalWrite(MPU_LED,1);
    delay(500);
    digitalWrite(MPU_LED,0);
    delay(500);
  }
}

void MPU9150::gyro_caliberation()
{
  //just leave the car stationary. the LED will blink once when the process begins, twice when it ends.
  blink(1);
  float dummy[3];
  int i,j;
  for(i = 0;i<1000;i++)
  {
    readIMU();
    for(j = 0;j<3;j++)
    {
      dummy[j] += g[j];
    }
    delayMicroseconds(500); //give it some rest
  }
  for(j=0;j<3;j++)
  {
    offsetG[j] = g[j]/1000;
  }
  delay(1000);
  blink(2);
}

void MPU9150::accel_caliberation()
{
  //place the car on a roughly horizontal surface, wait for the led to blink twice, then thrice, then rotate the car 180 degrees within
  //5 seconds, the process repeats. 
  float dummy[2][3];
  int i,j,k;
  for(k = 0;k<2;k++)
  { 
    blink(2);
    for(i = 0;i<1000;i++)
    {
      readIMU();
      for(j = 0;j<3;j++)
      {
        dummy[k][j] += float(a[j])*0.001;
      }
      delayMicroseconds(500);
    }
    blink(3);
    delay(5000);//should be enough time to change the orientation of the car.
  }
  for(i = 0;i<3;i++)
  {
    offsetA[i] = (dummy[0][i]+dummy[1][i])/2;//got the offsets!
  }
}

void MPU9150::mag_caliberation()
{
  //the mpu LED will blink once.
  //point the nose of the car in the NS direction, rotate the car around the lateral axis of the car for ~8 seconds,
  //point the nose of the car in the EW direction, rotate the car around the longitudenal axis of the car until you see
  //the MPU LED blink twice
  int16_t i,j,oldmax[3],oldmin[3];
  readMag();
  for(j=0;j<3;j++)
  {
    oldmin[j]=oldmax[j] = m[j];
  }
  blink(1);//indicate that the process has started
  delay(1000);
  for(i=0;i<20000;i++) //20 seconds
  {
    readMag();
    for(j=0;j<3;j++)
    {
      if (oldmax[j] < m[j])
      {
        oldmax[j] = m[j];
      }
      if(oldmin[j] > m[j])
      {
        oldmin[j] = m[j];
      }//basically finding the min max values of each field and then taking an average.
    }
    delay(10);//mag needs some delay
  }
  blink(2);
  for(j=0;j<3;j++)
  {
    offsetM[j] = float(oldmin[j]+oldmax[j])/2;
  }
}

void MPU9150::getOffset(int16_t offA[3],int16_t offG[3],int16_t offM[3],int16_t offT) //remember that arrays are passed by address by default.
{
  for(int i=0;i<3;i++)
  {
    offA[i] = offsetA[i];
    offG[i] = offsetG[i];
    offM[i] = offsetM[i];
  }
  offT = offsetT;
}

void MPU9150::setOffset(int16_t offA[3],int16_t offG[3],int16_t offM[3],int16_t offT)
{
  for(int i=0;i<3;i++)
  {
    offsetA[i] = offA[i];
    offsetG[i] = offG[i];
    offsetM[i] = offM[i];
  }
  offsetT = offT;
}

void MPU9150::readAll(bool mag_Read_Karu_Kya)
{
  readIMU();
  for(int i=0;i<3;i++)
  {
    A[i] = float(a[i] - offsetA[i])*ACCEL_SCALING_FACTOR;
    A[i] = filter_accel(i,A[i]);

    G[i] = float(g[i] - offsetG[i])*GYRO_SCALING_FACTOR;
    G[i] = filter_gyro(lastG[i],G[i]);
    lastG[i] = G[i];
  }
  if(mag_Read_Karu_Kya)
  {
    readMag();
    for(int i=0;i<3;i++)
    {
      M[i] = (float)(m[i] - offsetM[i]);
    }
  }
}

float MPU9150::tilt_Compensate(float roll,float pitch) //function to compensate the magnetometer readings for the pitch and the roll.
{
  float heading;
  float cosRoll = my_cos(roll); //putting the cos(roll) etc values into variables as these values are used over and 
  float sinRoll = my_sin(roll); //over, it would simply be a waste of time to keep on calculating them over and over again 
  float cosPitch = my_cos(pitch);//hence it is a better idea to just calculate them once and use the stored values.
  float sinPitch = my_sin(pitch);
  //the following formula is compensates for the pitch and roll of the object when using magnetometer reading. 
  float Xh = -M[0]*cosRoll + M[2]*sinRoll;
  float Yh = M[1]*cosPitch - M[0]*sinRoll*sinPitch + M[2]*cosRoll*sinPitch;
  
  Xh = Xh*0.2 + 0.8*magbuf[0]; //smoothing out the X readings
  magbuf[0] = Xh;

  Yh = Yh*0.2 + 0.8*magbuf[1]; //smoothing out the Y readings
  magbuf[1] = Yh;
  heading = 57.3*atan2(Yh,Xh);
  if(heading<0) //atan2 goes from -pi to pi 
  {
    return 360 + heading; //2pi - theta
  }
  return heading;
}//643us worst case 

void MPU9150::compute_All(bool mag_Read_Hua_Kya)
{ 
  float pitch_Radians, roll_Radians, d_Yaw_Radians;
  float diff;
  float Anet;
  float trust,trust_1;
  
  readAll(mag_Read_Hua_Kya);//read the mag if the condition is true.
  
  roll  += G[1]*dt; 
  roll_Radians = roll*DEG2RAD;
  pitch_Radians = pitch*DEG2RAD;
  float cosRoll = my_cos(roll_Radians);
  float _sinRoll = -my_sin(roll_Radians);

  float cosPitch = my_cos(pitch_Radians);
  float _sinPitch = -my_sin(pitch_Radians);

  d_Yaw_Radians = G[2]*dt*DEG2RAD;
  //if the car is going around a banked turn, then the change in heading is not the same as yawRate*dt. P.S: cos is an even function.
  mh += dt*(G[2]*cosRoll +G[0]*_sinRoll); //compensates for pitch and roll of gyro(roll pitch compensation to the yaw).  
  pitch += dt*(G[0]*cosRoll + G[2]*_sinRoll + roll*d_Yaw_Radians ) ; //compensates for the effect of yaw and pitch on pitch
  roll -= pitch*d_Yaw_Radians ;//compensate for roll-pitch interchange due to pitch.

  pitch_Error += GYRO_VARIANCE; //increment the errors each cycle
  roll_Error += GYRO_VARIANCE;
  mh_Error += GYRO_VARIANCE;

  Anet = (A[0]*A[0] + A[1]*A[1] + A[2]*A[2]); //square of net acceleration.
  trust = spike(G_SQUARED,Anet);
  trust_1 = 1-trust;
 
  if( mod(A[1])<9 ) 
  {
    pitch = trust_1*pitch + trust*0.573*my_asin(A[1]*0.102); //0.102 = 1/9.8
    pitch_Error *= trust_1; //reduce the error everytime you make a correction
  }
  if( mod(A[0])<9 )
  {
    roll  = trust_1*roll  - trust*0.573*my_asin(A[0]*0.102); //using the accelerometer to correct the roll and pitch.
    roll_Error *= trust_1;
  }

  if(mh >= 360.0) // the mh must be within [0.0,360.0]
  {
    mh -= 360.0;
  }
  if(mh < 0)
  {
    mh += 360;
  }
  yawRate = G[2]; // yaw_Rate sent out.

  if( mag_Read_Hua_Kya )//check if mag has been read or not.
  { 
    readMag(); //read magnetometer 1. It takes 143us at 560KHz i2c Clock. 
    if(mh>20.0&&mh<340.0&&yawRate<100) //range of angles where the magnetometer is reliable against the external interference from the motor.
    {//also, if the car is rotating really quickly, the time difference between when i ping the mag and when i get the results is significant, so
      //the mag is only reliable when i m rotating slowly.
      mh = 0.9*mh + 0.1*tilt_Compensate(roll*DEG2RAD, pitch*DEG2RAD); //TODO: make this dynamic depending on how much interference there is.
      mh_Error *= 0.9; //reduce the error.
    }//193us 
  }

  Ha = A[1]*cosPitch - bias; //this bias is taken from outside (sensor fusion with other sensors to correct velocity estimates) 
  V += Ha*dt;
  V_Error += dt*(ACCEL_VARIANCE*cosPitch + Ha*_sinPitch*pitch_Error );
}//1124us worst case

void MPU9150::Setup()//initialize the state of the marg.
{
  readAll(1); //read accel,gyro,mag 
  pitch = RAD2DEG*asin(A[1]*0.102); //0.102 = 1/9.8
  roll  = -RAD2DEG*asin(A[0]*0.102);  //multiply by 57.3 to standardize roll and pitch into degrees 
  mh = tilt_Compensate(roll*DEG2RAD,pitch*DEG2RAD); //roll, pitch have to be in radians
  yawRate = G[2]; //initial YawRate of the car.
}

float MPU9150::filter_gyro(float mean, float x)
{
  float i =  x - mean;//innovation
  i *= (i*i)/(10*GYRO_SCALING_FACTOR + (i*i) ); //notch filter around the mean value.
  return mean + i;
}

float MPU9150::filter_accel(int i,float x)
{
  xA[i][0] = xA[i][1]; 
  xA[i][1] = x*ACCEL_FILTER_GAIN;
  yA[i][0] = yA[i][1]; 
  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1* yA[i][0]);
  return yA[i][1];
}

void MARG_FUSE(MPU9150 marg[2])
{
  Fuse( marg[0].roll, marg[0].roll_Error, marg[1].roll, marg[1].roll_Error);
  Fuse( marg[0].pitch, marg[0].pitch_Error, marg[1].pitch, marg[1].pitch_Error);
  Fuse( marg[0].mh, marg[0].mh_Error, marg[1].mh, marg[1].mh_Error);
  Fuse( marg[0].V, marg[0].V_Error, marg[1].V, marg[1].V_Error); 
  //TODO: figure out how to fuse Accel, gyro without polluting them in case of sensor failure. 
}



//TODO : an NED acceleration and velocity thingy for drone.
//TODO : velocity estimator for car.