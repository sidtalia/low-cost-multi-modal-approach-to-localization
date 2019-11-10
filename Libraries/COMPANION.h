#ifndef _COMPANION_H_
#define _COMPANION_H_


#include"Arduino.h"
#include"PARAMS.h"


class JEVOIS
{
public:
	int8_t msg_len;
	uint8_t mode;
	long transmit_stamp, received_stamp,failsafe_stamp;
	bool failsafe = false;
	
	JEVOIS()
	{
		received_stamp = transmit_stamp = failsafe_stamp = millis();
		mode = 0x01;
		msg_len=0;
	}
	
	void write_To_Port(int16_t a,int bytes)
	{
	  uint8_t b[2];
	  for(int i = 0;i<bytes;i++)
	  {
	    b[i] = a>>(8*(i)); //last 8 bits
	    Serial2.write(b[i]);
	  }
	}

	// void send_heartbeat(); 
	void Send_State(byte MODE, float X, float Y, float heading, float dest_X, float dest_Y, float dest_Slope, float pitch, float roll, float yawRate, float velocity)
	{
		if(millis() - transmit_stamp > 100)
		{
			transmit_stamp = millis();
			int16_t out[11];
			out[0] = int16_t(X*1e2);
			out[1] = int16_t(Y*1e2);
			out[2] = int16_t(heading*1e2);
			out[3] = int16_t(dest_X*1e2);
			out[4] = int16_t(dest_Y*1e2);
			out[5] = int16_t(dest_Slope*1e2);
			out[6] = int16_t(velocity*1e2);
			out[7] = int16_t(yawRate);
			out[8] = int16_t(pitch*1e2);
			out[9] = int16_t(roll*1e2);
			out[10] = int16_t(MODE);
			Serial2.write('c');
			Serial2.write('a');
			Serial2.write('r');
			Serial2.write(' ');
			for(uint8_t i=0;i<11;i++)
			{
				write_To_Port(out[i],2);
			}
			Serial2.write(0x0d);
			Serial2.write(0x0a); //CRLF end of line.
		}
	}// 30 bytes

	uint16_t check()
	{
		uint16_t START_ID,message_ID;

		if(millis() - received_stamp > 100) //10 Hz 
		{
			received_stamp = millis();
			if(Serial2.available())
			{
				failsafe_stamp = millis();
				START_ID = Serial2.read()|int16_t(Serial2.read()<<8); //start sign
				if(START_ID == START_SIGN)
				{
					msg_len = Serial2.read()|int16_t(Serial2.read()<<8); //length of packet
					message_ID = Serial2.read()|int16_t(Serial2.read()<<8);	
					mode = Serial2.read()|int16_t(Serial2.read()<<8); 
					failsafe = false;
					return message_ID;
				}
				else
				{
					while(Serial2.available())
					{
						Serial2.read();
					}
				}
			}
			if(millis() - failsafe_stamp > 1000)
			{
				failsafe = true;
			}
		}
		return 0xFF;//no message
	}

	uint8_t get_Mode()
	{
		return mode;
	}


};

#endif
