
#include"OPFLOW.h"
#include"Arduino.h"
#include<SPI.h>

#define Frequency 400 //dt is defined in PARAMS.h


SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3);    // 2 MHz, mode 3

OPFLOW::OPFLOW()
{
	CALIBERATION = DEFAULT_CALIB;
}

void OPFLOW::caliberation(float height,float angle)//distance measured by a rangefinder, angle made by the object with the vertical
{
  float true_height = height/cos(angle*0.001745); //from degree to radians. when looking at a far away point, moving at the same speed, it appears that the point
  //is moving slower than it should
  CALIBERATION = true_height/128.0f;//new caliberation.
}

void  OPFLOW::updateOpticalFlow() //ma-ma-ma-ma-moneeeeyyyy shooooooot
{
  // Read sensor
	uint8_t buf[4];
	spiRead(ADNS3080_MOTION_BURST, buf, 4);
	uint8_t motion = buf[0];
	if (motion & 0x01) 
	{  
		int8_t dx = buf[1];   //caliberation for conversion to meters.   
		int8_t dy = buf[2];
		uint8_t surfaceQuality = buf[3];
    X = float(dx)*CALIBERATION;
    Y = float(dy)*CALIBERATION;
    SQ = surfaceQuality;

    if(SQ>40)
    {
      P_Error = CALIBERATION/SQ;//the smallest distance it can measure divided by surface Quality.
                                        //more surface quality = more reliable least count.
      V_Error = P_Error*Frequency; //least count/smallest time division
    }
    else
    {
      P_Error = 1000; //some very large value that the optical flow sensor would never actually have.
      V_Error = 1000;//ridiculous values to represent that optical flow is unreliable
    }
  } 
	else if(motion & 0x10)  //buffer overflow
	{
		int8_t dx = 0;   //caliberation for conversion to meters.   
		int8_t dy = 0;
		uint8_t surfaceQuality = 1;		
    X = 0.0;
    Y = 0.0;
    SQ = float(surfaceQuality);
	  P_Error = 1000; //some very large value that the optical flow sensor would never actually have.
    V_Error = 1000;//ridiculous values to represent that optical flow is unreliable.
  }

  V_x = X*Frequency;
  V_y = Y*Frequency;
}

void OPFLOW::reset_ADNS(void)              //reset. used almost never after the setup.
{
  digitalWrite(RESET_PIN, HIGH); // Set high
  delayMicroseconds(20);
  digitalWrite(RESET_PIN, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
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