#include"OPFLOW.h"
#include"Arduino.h"
#include<SPI.h>

//TOCHECK : replace camera lens, change clock speed, try a different sensor.
SPISettings spiSettings(1e6, MSBFIRST, SPI_MODE3);    // 2 MHz, mode 3

OPFLOW::OPFLOW()
{
	CALIBERATION = DEFAULT_CALIB;
}

void OPFLOW::caliberation(float height, float angle)//distance measured by a rangefinder, angle made by the object with the vertical
{
  float true_height = height/cosf(angle*0.001745); //from degree to radians. when looking at a far away point, moving at the same speed, it appears that the point
  //is moving slower than it should
  CALIBERATION = true_height/128.0f;//new caliberation.
}

float OPFLOW::error_calc()
{
  float x = SQ*NORMALIZE;
  float x2 = x*x;
  float x3 = x2*x;
  return K0*x3 + K1*x2 + K2*x + K3;
}

void  OPFLOW::updateOpticalFlow() //ma-ma-ma-ma-moneeeeyyyy shooooooot
{
  // Read sensor
	uint8_t buf[6];
	spiRead(ADNS3080_MOTION_BURST, buf, 7);
	uint8_t motion = buf[0];
	if (motion & 0x01) 
	{  
		int8_t dx = buf[1];   //caliberation for conversion to meters.   
		int8_t dy = buf[2];
		uint8_t surfaceQuality = buf[3];
    int shutter = buf[4]<<8|buf[5];
    uint8_t pix = buf[6];
    X = dx;
    X *= CALIBERATION;
    Y = dy;
    Y *= CALIBERATION;
    SQ = surfaceQuality;
    shutter_Speed = float(shutter);
    max_pix = float(pix);
    if(SQ<10)
    {
      SQ = 10.0f; //sanity check
    }
    V_Error = fabs(error_calc());
    P_Error = V_Error*dt; //replace with 1/(1+(9-SQ*0.11)) if it takes too much processing.
    if(SQ<70)
    {
      V_Error = 1e3;
      P_Error = 1e3;
    }

    X += omega[1]*ride_height*dt*0.1f; //the sign was flipped on 11/2/19 3:28pm
    X = LPF(0,X);
    Y -= omega[0]*ride_height*dt*0.1f; //compensation for rotations ya know. this sign was also flipped. please run a test.
    Y = LPF(1,Y);
  } 
	else if(motion & 0x10)  //buffer overflow
	{
		uint8_t surfaceQuality = 1;		
    X = LPF(0,X);
    Y = LPF(1,Y);
    SQ = float(surfaceQuality);
	  P_Error = 1e3; //some very large value that the optical flow sensor would never actually have.
    V_Error = 1e3;//ridiculous values to represent that optical flow is unreliable.
  }

  V_x = X*LOOP_FREQUENCY;
  V_x = LPF(2,V_x);
  V_y = Y*LOOP_FREQUENCY;
  V_y = LPF(3,V_y);  
//sensor health check.
  // TODO : insert (in the main code) a method to get the sensor health for all sensors (please?)
  health += fabs(V_y-omega[2])>=1.0f? 1:-1; //increase health for every glitch that we get, decrease it for every nice reading
  health = health<=0 ? 0 : health; //ensure the health isn't less than -1
  failure = health>50? true:false;
}

void OPFLOW::reset_ADNS(void)              //reset. used almost never after the setup.
{
  digitalWrite(RESET_PIN, HIGH); // Set high
  delayMicroseconds(20);
  digitalWrite(RESET_PIN, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
}


float OPFLOW::LPF(int i,float x)
{
  xA[i][0] = xA[i][1]; 
  xA[i][1] = x*LPF_GAIN_OPFLOW;
  yA[i][0] = yA[i][1]; 
  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1_OPFLOW* yA[i][0]); // first order LPF to predict new speed.
  return yA[i][1];
}

void OPFLOW::set_6469(void)
{
  // set frame rate to manual
  uint8_t regVal = spiRead(ADNS3080_EXTENDED_CONFIG);
  regVal = (regVal & ~0x01) | 0x01;
  delayMicroseconds(75);  // small delay
  spiWrite(0x0B, regVal);
  delayMicroseconds(75);  // small delay
  // set frame period for 6469 fps
  spiWrite(0x19,0x7E);
  delayMicroseconds(75);  // small delay
  spiWrite(0x1A,0x0E);

}

bool OPFLOW::initialize(void)
{
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  bool connection=false;
  reset_ADNS(); //reset ADNS3080
  if(!connection) //TODO : Send out alert on Xbee.
  { 
    uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
    id = spiRead(ADNS3080_PRODUCT_ID); //gotta do it twice because STM32 is a bitch. 
    (id == ADNS3080_PRODUCT_ID_VALUE)? connection=1 : connection=0;  // represents connection break  
  }
  uint8_t configuration = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, configuration | 0x10); // Setting resolution.

  return connection;
}

void OPFLOW::spiWrite(uint8_t reg, uint8_t data) 
{
  spiWrite(reg, &data, 1);
}

void OPFLOW::spiWrite(uint8_t reg, uint8_t *data, uint8_t len) 
{
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg | 0x80); // Indicate write operation
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  SPI.transfer(data, len); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t OPFLOW::spiRead(uint8_t reg) 
{
  uint8_t buf;
  spiRead(reg, &buf, 1);
  return buf;
}

void OPFLOW::spiRead(uint8_t reg, uint8_t *data, uint8_t len) 
{
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);          //telling the optical flow sensor that we want to read it (MISO mode)

  SPI.transfer(reg); // Send register address
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  memset(data, 0, len); // Make sure data buffer is 0
  SPI.transfer(data, len); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}
