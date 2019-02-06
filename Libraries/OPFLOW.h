//I found the code for sensor communication in some dark corner of the internet. I'm not saying it's mine but I don't know whom
//to give credit so ¯\_(ツ)_/¯ 

#ifndef _OPFLOW_H_
#define _OPFLOW_H_

#include"Arduino.h"
#include"SPI.h"
#include"PARAMS.h"


#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_SQUAL                 0x05
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

#define DEFAULT_CALIB (float)ride_height/128.0f
#define DEFAULT_DT dt
#define DEFAULT_FREQ 1/dt
/*
usage : 
	OPFLOW obj;
	//after initializing SPI and all that.
	obj.initialize();
	obj.caliberation(height,angle);
	//
	obj.updateOpticalFlow(data);//get that data baby
*/

#define LPF_GAIN_OPFLOW (float)1/13.70620474
#define C1_OPFLOW (float)0.8540806855

class OPFLOW
{
public:
	OPFLOW();
	void caliberation(float height,float angle); //use this in aerial vehicles or if the car's ride height changes. 
	void updateOpticalFlow();
	void reset_ADNS(void);
	bool initialize(void);
	void spiWrite(uint8_t reg, uint8_t data);
	void spiWrite(uint8_t reg, uint8_t *data, uint8_t len);
	uint8_t spiRead(uint8_t reg);
	void spiRead(uint8_t reg, uint8_t *data, uint8_t len); 
	float LPF(int i,float x);
	float CALIBERATION;
	float X, Y, V_x, V_y, SQ, P_Error, V_Error;
	float omega[2];
	float xA[4][2],yA[4][2];//for low pass filter
};

#endif
