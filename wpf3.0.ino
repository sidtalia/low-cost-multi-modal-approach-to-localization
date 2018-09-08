/*  pin allocation table- 
 *  (0,1)-GPS Rx/Tx
 *  (SDA,SCL)-IMU, magneto,slave arduinos
 *  3-throttle servo
 *  4-steering servo
 *  9,10,11,12,13- used for SPI by optical flow
 *  A0 - r/c receiver, A1 ->USR multiplexing.
 *  free pins- 2,5,6,7,8.
*/

/*NOTE: 
 * It would appear to you that a lot of the variables are global even though most would consider such a practice as unsafe. 
 * However on closer inspection, you will notice that the variables I have kept global are the ones that are shared between multiple functions.
 * This makes writing the functions etc easier for me, plus it reduces the function overhead as the variables are already in the working memory.
 * This also gives me a lot of freedom to use intermediate values from one function in another function(like accelerations, gyrations, etc) for filtering purposes.
 * It would be of some help if someone could tell me how i could make some variables read-only outside of a code block so that the system becomes a little more "secure"
 * without creating classes.
 */
 
/* recent updates(latest first)-
 *  Using 2 MARGS instead of 1 + using the motor commands to guesstimate the speed
 *  not converting magnetometer readings to gauss anymore.
 *  gyro offsets calculated during setup. 2s delay removed during startup.
 * replaced TWBR=12 with Wire.setClock(800000) 
 * bezier curve based trajectory planning added.
 * driver module has it's own separate tab
 * the function prototypes of separate tabs has been put into the main tab because the arduino IDE thinks i am writing in avr level and it wont let me define things in other tabs. 
 * usr module added 
 * miscellaneous changes- mh=90 initialization removed 
 *                       - global coordinates updated each cycle  
 *                       - math functions are in a separate tab 
 *                       - k(accel trust) was for some reason not being calculated, so added a statement for that
 * Kalman filter updated to handle gps and opflow instead of gps and imu.
 * i2c communication to slave arduinos added.function takes address of slave and the message array into which the message is stored. currently only for 2 pieces of information. Not in use right now
 * added optical flow code .  it's function is called in the localization tab before calling "callimu()". results from OpFlow are used in accelgyro tab 
 * magnetometer incorporated. function call is in accelgyro tab
 * accelgyro offsets are now fixed. cant use #define stuff because it would not allow the call imu code to work 
 * accelgyro gps kalman tested.
 * accelgyro incorporated. function call is in the localization tab
 * gps update rate increased to 5Hz, now using ublox's own protocol which has much smaller packet size, baud rate kept at 57600, can be increased to 230400.
 * gps used for basic localization. 
*/

/*
 * general architecture - collect data first ,then do all the math, then write all the signals
 * 
 */


 

#include "I2Cdev.h"       //communication libraries
#include <SPI.h>
#include "Wire.h"
#include "MPU9150.h"
#include<Servo.h>

Servo motor,steer;
#define dt 0.0025
#define MAX_ACCELERATION 7 // max sideways acceleration is 10/root(2) m/s*s

//--------------IMU STUFF-------------------------
MPU9150 accelgyro_1;
MPU9150 accelgyro_2(0x69);

uint8_t address[2] = {0x68,0x69}; //keep the AD0 pin low for 0x68, high for 0x69
int16_t a[3], g[3], lastg[2][3]={0,0,0}, m[3];
uint8_t buf[6];
float variance[2][2][3];
bool failure = {0,0};
float A[3][3], G[3][3], M[3], lastA[2][3], lastG[2][3];//2 rows for 2 margs and 1 row for final data
float gain[2][3]; // gain for each IMU
float offsetA[2][3], offsetG[2][3], offsetM[2][3]; //offsets for each MARG
float AccBias=0;
float roll, pitch, yawRate;
float magbuf[2]={0,0};
float del=0;//difference in angle between when the magnetometer was called and when it actually replied.
float T[2];
float Ha,V=0;
//--------------IMU STUFF ENDS--------------------

//--------------GPS STUFF-------------------------
float latitude=0,longitude=0,Hdop;
float iLong,iLat,lastX,lastY,lastLong,lastLat,destLat,destLong;
bool tick=false; 
#define GPSBAUD 115200
#define MAXVARIANCE 100//max hdop allowed during startup
//--------------GPS STUFF ENDS--------------------

//--------------OPTICAL FLOW STUFF----------------
SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3);    // 2 MHz, mode 3
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50
// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30
// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17


#define RESET_PIN PB0
#define SS_PIN PA4
int8_t dx,dy;
uint8_t surfaceQuality;
bool connection=false;
//--------------OPTICLA FLOW STUFF ENDS-----------

//--------------ULTRASONIC STUFF------------------
volatile unsigned long timer[4];
volatile byte last_channel[3]={0,0,0};
volatile int input[3]={0,0,0};
// all this stuff is redundant for the time being until I add the obstacle avoidance unit (which uses a separate microcontroller 
//of it's own to handle the sensor management. 
float retard=0;    //braking
float sonarCorrection=0;  //deviation suggested by the sonar 
int message[2];
#define SONAR_MODULE_ADDRESS 1
#define SONAR_GAIN 3
#define RETARD_GAIN 0.2
#define MAXSONAR_TRIGGER 2
#define MAXSONAR_VCC 5
//--------------ULTRASONIC STUFF ENDS-------------

//--------------GENERAL VARIABLES-----------------
float mh; //ml=slope of line connecting bot to destination, mh= slope of line along heading.
float d; // distance between bot and destination
int l=1;  //l- stopping distance from target. 
float globalX,globalY,X,Y; //globalX and globalY are the final estimates of X and Y. theta is the angle rotated between consequent readings of gps,m is a variable for convenience 
float destX,destY,destSlope; //destination coordinates and heading.
float int1[2],int2[2]; //intermediate points for the bezier curve
float t; // fraction of the path at which the next ROC and Vmax is calculated. I hope I have not redefined it. One of the drawbacks of having a lot of global variables.
float correction=0,Vmax,yaw_Compensation;  //steering angle required by the bot and max velocity of the bot.
float now;  //variables for checking the time of the control loop
float estimateError,KG;  //imu's error in estimation and kalman gain
int point=1,i;
bool accelRequired=false;
float setPoint=1.5,error =0,Serror =0;
int n;//number of coordinates
int last_Throttle = 1500;// to keep a track of change in throttle
//--------------GENERAL VARIABLES END-------------

class coordinates //for storing multiple coordinates as we further progress to multiple waypoint trajectories instead of just 1 waypoint
{                               //this will become the main point of focus once the basic tasks are complete
  float longitude,latitude,X,Y;
  public:
  float slope;
  void setcords(float y,float x)
  {
    latitude=y;
    longitude=x;
  }
  float rety()
  {
    return latitude;
  }
  float retx()
  {
    return longitude;
  }
  float setcordsO(float y,float x)
  {
    X=x;
    Y=y;
  }
  float retYO()
  {
    return Y;
  }
  float retXO()
  {
    return X;
  }
};

coordinates c[8];

void setup()
{
  Serial.begin(GPSBAUD); // initialize UART communication
  //-----------------OPTICAL FLOW Setup-----------------
  SPI.begin();
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset_ADNS();
  while(!connection)
  { 
    uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
    id = spiRead(ADNS3080_PRODUCT_ID); //gotta do it twice because STM32 is a bitch. 
    (id == ADNS3080_PRODUCT_ID_VALUE)? connection=1 : connection=0;  // represents connection break  
  }
  uint8_t configuration = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, configuration | 0x10); // Set resolution to 1600 counts per inch-this is not my comment.
  for(i  =0;i<100;i++)
  {
    updateOpticalFlow();//warm up the sensor.
  }
  
  //-----------------optical flow setup-----------------------
  
  //-------------- IMU setup begins---------------------------
  Wire.begin();//initialize TWI communication
  Wire.setClock(400000); //make sure we have the fastest speed possible.

  IMU_Setup();//initialize IMUs
  //finding initial orientation now.
  ReadAll();

  pitch = 57.3*asin(A[2][1]*0.102); //0.102 = 1/9.8
  roll  = -57.3*asin(A[2][0]*0.102);  //multiply by 57.3 to standardize roll and pitch into degrees 
  float head[2];
  for(k=0;k<2;k++)
  {
    readMagDuringSetup(address[k],k);
    head[k] = tilt_Compensate(roll/57.3,pitch/57.3); //roll, pitch have to be in radians
  }
  mh = (head[0]+head[1])*0.5;
  yawRate = G[2][2];
  //------------IMU SETUP ENDS------------------------
  
  // motor and steering
  motor.attach(3);
  steer.attach(4);
  for(int i=0;i<250;i++)
  {
    motor.writeMicroseconds(1500);//for some reason writing it once and then leaving it there for 5 seconds does not do the trick.
    steer.writeMicroseconds(1500);
    delay(20);
  }
  //----------------GENERAL SETUP BEGINS--------------
  pinMode(2,OUTPUT);
  i=3; // initialize i at 3.
  //do
  for(i = 0;i<50;i++)
  {
    localizer();
    tick =false; //just something i have to do to make sure it doesn't take the last known GPS reading when it actually starts looping.
    iLat=latitude;
    iLong=longitude;
    if(Hdop<1000.0)
    {
      break;
    }
    delay(50);
  }
//  while(1); //while HDOP>10
  lastLat = iLat;
  lastLong = iLong;
  //2/3 parking lot horizontal y/x
  c[0].setcords(iLat,iLong);  //STORE LOCATION OF ORIGIN IN C[0]
  //for now since i m working in a local frame, i can use meters instead of GPS coordinates, This is a little more reliable too as the GPS probably can't
  //tell me the exact initial location in degrees anyway.
  c[0].setcordsO(0.0,0.0);   //the location of origin is stored in an object of class coordinates using a member function called setcords(float x,float y)   
  c[1].setcordsO(4.596,3.857);   //location of destination:-can be anything, is stored in another object of class coordinates
  c[2].setcordsO(7.26,2.747);
  c[3].setcordsO(1.832,-1.555); 
  c[4].setcordsO(3,-7);
  c[5].setcordsO(0.5,-8.5);
  c[6].setcordsO(-2,-3.46);
  c[7].setcordsO(0.0,0.0);//setting the final location as initial location to check error.
//  c[1].setcordsO(1.932,0.517);
//  c[2].setcordsO(2.449,-1.414);
//  c[3].setcordsO(0.517,-1.932);
//  c[4].setcordsO(0.0,0.0);
  
  //setting the first destination coordinates.
  X= lastX = globalX = 0;  //initializing globalX and globalY
  Y= lastY = globalY = 0;

  n = 8;
  generate_Slopes();//math functions tab. find average slopes.
  c[7].slope = c[6].slope = c[0].slope = mh; //initial slope is the same as initial heading.
//  n = 7;
//  generate_Slopes();
//
//  c[0].slope = c[4].slope = 255.0;
//
//  n=5;
//  generate_Slopes();
  
  destX=c[1].retXO();// use (destLong-iLong)*111692.84 if working with degrees
  destY=c[1].retYO();//(destLat-iLat)*111692.84;
  destSlope = c[1].slope;
  //--------------GENERAL SETUP ends-----------------------   

  
  PCICR  |= (1 << PCIE1);                     // Configure pin change interrupts for PCINT1
  PCMSK1 |= (1 << PCINT8);                    // Pin change interrupt for A0
  PCMSK1 |= (1 << PCINT9);                    // Pin change interrupt for A1
  PCMSK1 |= (1 << PCINT10);                   // Pin change interrupt for A2
  PCMSK1 |= (1 << PCINT11);

}

byte cycle = 1;

void loop()
{
  now = micros();
  ComputeAll();
  if(cycle==pow(2,0)||cycle==pow(2,4))
  { 
    readMag(address[0],0);
    if(mh>20.0&&mh<340.0&&yawRate<100) //range of angles where the magnetometer is reliable against the external interference from the motor.
    {//also, if the car is rotating really quickly, the time difference between when i ping the mag and when i get the results is significant, so
      //the mag is only reliable when i m rotating slowly.
      mh = 0.9*mh + 0.1*(del+ tilt_Compensate(roll*0.01745, pitch*0.01745)); //tilt compensation takes angles in radians
    }//643us
  }
  if(cycle==pow(2,2)||cycle==pow(2,6))
  { 
    readMag(address[1],1);
    if(mh>20.0&&mh<340.0&&yawRate<100) //range of angles where the magnetometer is reliable against the external interference from the motor.
    {//also, if the car is rotating really quickly, the time difference between when i ping the mag and when i get the results is significant, so
      //the mag is only reliable when i m rotating slowly.
      mh = 0.9*mh + 0.1*(del+ tilt_Compensate(roll*0.01745, pitch*0.01745)); //tilt compensation takes angles in radians
    }//643us
  }
  
  localization();//663
  d=distancecalcy(globalY,destY,globalX,destX,0); //returns distance in meters. look into math_functions tab. 70us
  t = get_T(V,d); //40us finds the t parameter for the bezier curve. the t corresponds to the point where the car will be 2 cycles from now.  
  get_Intermediate_Points(mh,destSlope,globalX,destX,globalY,destY,d);//248us. 4th degree bezier curve needs 4 points. I have
    //the current position and the final position as the 2 extreme points but i still need the points in the middle. This function does that.
  Curvature(globalX,globalY,int1[0],int1[1],int2[0],int2[1],destX,destY,t,MAX_ACCELERATION);//540us. the change in globalX can't be more than 5cm even at 20m/s
    //the function calculates the steering angle required and the max Velocity according to the max allowed lateral acceleration and yaw_Compensation
  driver();//140us
 
  if(cycle==pow(2,3))
  {
    uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
    if(id != ADNS3080_PRODUCT_ID_VALUE) 
    {
      reset_ADNS();
    }
  }//2397
  
  cycle <<= 1;//bit shift cycle variable 
  if(cycle==0)
  {
    cycle=1;
  }
  while(micros()-now<=2500); //fixing loop time.
}

ISR(PCINT1_vect)
{
  timer[0]=micros();
  //channel 1 ----
  
  if(last_channel[0]==0&& PINC & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[0]=1;
    timer[1]=timer[0];           
  }
  else if(last_channel[0]==1 && !(PINC & B00000001))
  {
    last_channel[0]=0;
    input[0]=timer[0]-timer[1];
  }

  //channel 2---                  
  if(last_channel[1]==0 && PINC & B00000010) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[1]=1;
    timer[2]=timer[0];          
  }
  else if(last_channel[1]==1 && !(PINC & B00000010))
  {
    last_channel[1]=0;
    input[1]=timer[0]-timer[2];
  }
  
  //channel 3-- 
  if(last_channel[2]==0&& PINC & B00000100) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[2]=1;
    timer[3]=timer[0];          
  }
  else if(last_channel[2]==1 && !(PINC & B00000100))
  {
    last_channel[2]=0;
    input[2]=timer[0]-timer[3];
  }
}
