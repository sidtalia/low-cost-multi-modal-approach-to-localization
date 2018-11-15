#ifndef _SIDMATH_H_
#define _SIDMATH_H_

#include"Arduino.h"
#include"math.h"

#define DEG2RAD (float)0.0174533
#define RAD2DEG (float)57.2958

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

float anglecalcy(float x1,float x2,float y1,float y2);  

float distancecalcy(float y1,float y2,float x1,float x2,int i);

float mod(float a);  //taking mod of a number
//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
float gpsOpFlowKalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate);

void Fuse(float &A, float &A_Error, float &B, float &B_Error);

float depress(float a,float k);

float my_asin(float a);

float my_cos(float a);

float my_sin(float a);

float get_T(float V,float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float d,float dt);
 
float ROC(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t,float max_Acceleration);

void get_Intermediate_Points(float slope1,float slope2,float X1,float X2,float Y1,float Y2,float d);

void generate_Slopes(coordinates c[],int n) ;

float spike(float mean, float x);

#endif 
