#ifndef _COMS_H_
#define _COMS_H_



#include"Arduino.h"
#include"PARAMS.h"


class GCS
{
public:

	long transmit_stamp, received_stamp;
	GCS()
	{
		received_stamp = transmit_stamp = millis();
	}
	
	void write_To_Port(int16_t a,int bytes)
	{
	  uint8_t b[4];
	  for(int i = 0;i<bytes;i++)
	  {
	    b[i] = a>>(8*(bytes -1 -i)); //last 8 bits
	    Serial.write(b[i]);
	  }
	}

	bool Get_Offsets(int16_t A1[3], int16_t A2[3], int16_t G1[3], int16_t G2[3], int16_t M1[3], int16_t M2[3], int16_t T1, int16_t T2)
	{
		uint8_t i;
		int16_t START_ID, message_ID, len;
		
		write_To_Port(START_SIGN,2);//start sign
		write_To_Port(6,2); 		//length of payload
		write_To_Port(OFFSET_ID,2); //tell the GCS that I want them sweet sweet offsets.

		delay(100);//wait 100 second for the data to come in.

		if(Serial.available())
		{
			START_ID = Serial.read()<<8|Serial.read(); //start sign
			len = Serial.read()<<8|Serial.read(); //length of packet 40 bytes
			message_ID = Serial.read()<<8|Serial.read();
			if( message_ID==OFFSET_ID && len==40 )//confirm that you are getting the offsets and nothing else.
			{
				for(i=0;i<3;i++)//computer has offsets
				{
					A1[i] = Serial.read()<<8|Serial.read();
					A2[i] = Serial.read()<<8|Serial.read();
					G1[i] = Serial.read()<<8|Serial.read();
					G2[i] = Serial.read()<<8|Serial.read();
					M1[i] = Serial.read()<<8|Serial.read();
					M2[i] = Serial.read()<<8|Serial.read();
				}
			}
			else
			{
				return 0; //if computer has no offsets
			}
		}
		T1 = Serial.read()<<8|Serial.read();
		T2 = Serial.read()<<8|Serial.read();
		return 1;
	} //

	void Send_Offsets(int16_t A1[3], int16_t A2[3], int16_t G1[3], int16_t G2[3], int16_t M1[3], int16_t M2[3], int16_t T1, int16_t T2)
	{
		uint8_t i;
		write_To_Port(START_SIGN,2);
		write_To_Port(40,2);
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

	void Get_WP(float X[20], float Y[20])
	{
		delay(100);
		uint8_t len = Serial.available();
		for(int i=0;i<len;i++) //test this please.
		{
			X[i] = float( long( Serial.read()<<24|Serial.read()<<16|Serial.read()<<8|Serial.read() ) )*1e-8; //coordinates transfered wrt to origin, converted 
		}
		for(int i=0;i<len;i++) //test this please.
		{
			Y[i] = float( long( Serial.read()<<24|Serial.read()<<16|Serial.read()<<8|Serial.read() ) )*1e-8; //coordinates transfered wrt to origin, converted 
		}
	}

	void Send_WP(float X, float Y)
	{
		long x = long(X*1e8);
		long y = long(Y*1e8);

		write_To_Port(START_SIGN,2);
		write_To_Port(8,2);
		write_To_Port(WP_ID,2);
		write_To_Port(x,4);
		write_To_Port(y,4);
	}//10 bytes 

	void Send_Calib_Command(uint8_t id)
	{
		write_To_Port(START_SIGN,2);
		write_To_Port(2,2);
		if(id == 1)
			write_To_Port(GYRO_CAL,2);
		if(id == 2)
			write_To_Port(ACCEL_CAL,2);
		if(id == 3)
			write_To_Port(MAG_CAL,2);
		else
			write_To_Port(ERROR_CODE,2);
	}//2 bytes

	// void send_heartbeat(); 
	void Send_State(byte mode,float lon, float lat, float vel, float heading)//position(2), speed(1), heading(1), acceleration(1), Position Error
	{
		if(millis() - transmit_stamp > 100)
		{
			transmit_stamp = millis();
			long out[4];
			out[0] = lon*1e8;
			out[1] = lat*1e8;//hopefully this is correct
			out[2] = vel*100;
			out[3] = heading*100;

			write_To_Port(START_SIGN,2);
			write_To_Port(18,2);
			write_To_Port(STATE_ID,2);
			write_To_Port(int16_t(mode),2);
			for(int i=0;i<4;i++)
			{
				write_To_Port(out[i],4);
			}
		}
	}// 30 bytes

	uint16_t check()
	{
		uint16_t START_ID,len,message_ID;

		if(millis() - received_stamp > 100) //10 Hz 
		{
			received_stamp = millis();
			if(Serial.available())
			{
				START_ID = Serial.read()<<8|Serial.read(); //start sign
				len = Serial.read()<<8|Serial.read(); //length of packet 40 bytes
				message_ID = Serial.read()<<8|Serial.read();
				
				return message_ID;
			}
		}
		return 0xFF;//no message
	}

	uint8_t get_Mode()
	{
		uint8_t mode = Serial.read();
		mode = Serial.read(); //its the last 8 bytes that matter
		return mode;
	}


};

#endif
