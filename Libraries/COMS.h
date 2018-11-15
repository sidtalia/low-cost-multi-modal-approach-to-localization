#ifndef _COMS_H_
#define _COMS_H_

#include"Arduino.h"

void write_To_Port(int16_t a,int bytes)
{
  uint8_t b[4];
  for(int i = 0;i<bytes;i++)
  {
    b[i] = a>>(8*(bytes -1 -i)); //last 8 bits
    Serial.write(b[i]);
  }
}

/*
packet start sign (0xFE)
payload length
packet sequence(?)
System ID
Component ID
Message ID
Data
Checksum
*/

/*
Position (LLH),
Velocity,
Heading
rp,
acceleration
sensor health
temp

*/

#define OFFSET_ID 0x0001
#define COMMAND_ID 0X0002

#define WP_ID 0x0005
#define STATE_ID 0x0006

#define GYRO_CAL 0xF000
#define ACCEL_CAL 0xE000
#define MAG_CAL 0xD000

#define ERROR_CODE 0xFFFF

bool GCS_Get_Offsets(int16_t A1[3], int16_t A2[3], int16_t G1[3], int16_t G2[3], int16_t M1[3], int16_t M2[3], int16_t T1, int16_t T2)
{
	uint8_t i;
	int16_t ID;
	write_To_Port(OFFSET_ID,2);//tell the GCS that I want them sweet sweet offsets.

	if(Serial.available())
	{
		ID = Serial.read()<<|Serial.read();
		if(ID==OFFSET_ID)//confirm that you are getting the offsets and nothing else.
		{
			for(i=0;i<3;i++)//computer has offsets
			{
				A1[i] = Serial.read()<<|Serial.read();
				A2[i] = Serial.read()<<|Serial.read();
				G1[i] = Serial.read()<<|Serial.read();
				G2[i] = Serial.read()<<|Serial.read();
				M1[i] = Serial.read()<<|Serial.read();
				M2[i] = Serial.read()<<|Serial.read();
			}
		}
		else
		{
			return 0; //if computer has no offsets
		}
	}
	T1 = Serial.read()<<|Serial.read();
	T2 = Serial.read()<<|Serial.read();
	return 1;
} //42 bytes received.

void GCS_Send_Offsets(int16_t A1[3], int16_t A2[3], int16_t G1[3], int16_t G2[3], int16_t M1[3], int16_t M2[3], int16_t T1, int16_t T2)
{
	uint8_t i;
	write_To_Port(OFFSET_ID,2);
	for(i=0;i<3;i++)
	{
		write_To_Port(A1[i],2);
		write_To_Port(A2[i],2);
		write_To_Port(G1[i],2);
		write_To_Port(G2[i],2);
		write_To_Port(M1[i],2);
		write_To_Port(M2[i],2);	
	}
	write_To_Port(T1,2);
	write_To_Port(T2,2);
}//42 bytes sent

uint8_t GCS_Get_Command()
{
	if(Serial.available())
	{
		return Serial.read()<<|Serial.read(); //0x0 for STOP 0x01-> full manual, 0x03 ->partial manual
							//0x04 autonomous. (requires waypoints)
	}
	return 0;
}

void GCS_Send_Command(int id)//use this for asking for things during loop. request response has to be asynchronous.
{
	write_To_Port(id,2);
}

void GCS_Get_WP(float X[20], float Y[20])
{
	int16_t ID = Serial.read()<<|Serial.read();
	if(ID == WP_ID)
	{
		int16_t len = Serial.read()<<|Serial.read();//number of waypoints in the incoming data
		for(int i=0;i<len;i++) //test this please.
		{
			X[i] = float(long(Serial.read()<<|Serial.read()<<|Serial.read()<<|Serial.read()))*1e-8; //coordinates transfered wrt to origin, converted 
		}
		for(int i=0;i<len;i++) //test this please.
		{
			Y[i] = float(long(Serial.read()<<|Serial.read()<<|Serial.read()<<|Serial.read()))*1e-8; //coordinates transfered wrt to origin, converted 
		}
	}
}

void GCS_Send_WP(float X, float Y)
{
	long x = long(X*1e8);
	long y = long(Y*1e8);
	write_To_Port(WP_ID,2);
	write_To_Port(x,4);
	write_To_Port(y,4);
}//10 bytes 

void GCS_Send_Calib_Command(uint8_t id)
{
	if(id == 1)
		write_To_Port(GYRO_CAL,2);
	if(id == 2)
		write_To_Port(ACCEL_CAL,2);
	if(id == 3)
		write_To_Port(MAG_CAL,2);
	else
		write_To_Port(ERROR_CODE,2)
}//2 bytes

// void send_heartbeat(); 
void GCS_Send_State(float message[6])//position(2), speed(1), heading(1), acceleration(1), Position Error
{
	long out[6];
	out[0] = message[0]*1e8;
	out[1] = message[1]*1e8;//hopefully this is correct
	out[2] = message[2]*100;
	out[3] = message[3]*100;
	out[4] = message[4]*100;
	out[5] = message[5]*100;

	write_To_Port(STATE_ID,2);
	for(int i=0;i<6;i++)
	{
		write_To_Port(out[i],4);
	}
}//26bytes

void GCS_Send_Health(bool health[3])
{
	int16_t message;
	message = 4*health[2] + 2*health[1] + health[0];
	write_To_Port(message,2); 
}//2 bytes


#endif
