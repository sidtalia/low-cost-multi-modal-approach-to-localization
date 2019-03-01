#include"SIDMATH.h"


float anglecalcy(float x1,float x2,float y1,float y2)  //everything is inline because fuck you thats why
{
  float angle = RAD2DEG*atan2((y2-y1),(x2-x1));
  if(angle<0)
  {
    angle += 360;
  }
  return angle;
}//215us


//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
float Kalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate)   //used in localization tab
{
  float estimateError=(50.0/float(trustInEstimate));     //biasing was changed to 30 on 1/4/17 (not the american date standard.)
  //estimateError*=estimateError; //squaring the estimate error because i want the trust to rise very quickly if surface quality is good and fall very quickly if it is lower than 20)
  float KG=(estimateError/(estimateError+gpsError));
  return (KG*gpscord+(1-KG)*estimate);  
}//95us


void Fuse(float &A, float &A_Error, float &B, float &B_Error)
{
  float KG = A_Error/(A_Error + B_Error);
  A_Error *= (1-KG);
  B_Error *= (1-KG); //reduce the error.
  A = (1-KG)*B + KG*A;
  B = A;
}

float depress(float a,float k)               //used to depress accelgyro values
{
  return (a*a*a*a)/(k+a*a*a*a);
}//74.5us 
