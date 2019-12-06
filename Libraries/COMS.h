#ifndef _COMS_H_
#define _COMS_H_



#include"Arduino.h"
#include"PARAMS.h"


class GCS
{
public:
	int16_t msg_len;
	uint8_t mode;
	long transmit_stamp, received_stamp,failsafe_stamp;
	bool failsafe = false;
	GCS()
	{
		received_stamp = transmit_stamp = failsafe_stamp = millis();
		mode = 0x01;
	}
	
	void write_To_Port(int32_t a,int bytes)
	{
	  uint8_t b[4];
	  for(int i = 0;i<bytes;i++)
	  {
	    b[i] = a>>(8*(i)); //last 8 bits
	    Serial.write(b[i]);
	  }
	}

	bool Get_Offsets(int16_t A[3], int16_t G[3], int16_t M[3], int16_t &T,int16_t gain[3])
	{
		uint8_t i;
		int16_t START_ID, message_ID, len;
		
		write_To_Port(START_SIGN,2);//start sign
		write_To_Port(8,2); 		//length of payload
		write_To_Port(OFFSET_ID,2); //tell the GCS that I want them sweet sweet offsets.
		write_To_Port(0x01,2);

		delay(1000);//wait 1 second for the data to come in
		if(Serial.available())
		{
			START_ID = Serial.read()|int16_t(Serial.read()<<8); //start sign
			if(START_ID == START_SIGN)
			{
				len = Serial.read()|int16_t(Serial.read()<<8); //length of packet 40 bytes
				message_ID = Serial.read()|int16_t(Serial.read()<<8);
				Serial.read()|int16_t(Serial.read()<<8);//waste
				if(message_ID==OFFSET_ID && len == 28)//confirm that you are getting the offsets and nothing else.
				{
					for(i=0;i<3;i++)//computer has offsets
					{
						A[i] = Serial.read()|int16_t(Serial.read()<<8);
						G[i] = Serial.read()|int16_t(Serial.read()<<8);
						M[i] = Serial.read()|int16_t(Serial.read()<<8);
						gain[i] = Serial.read()|int16_t(Serial.read()<<8);
					}
					T = Serial.read()|int16_t(Serial.read()<<8);
					return 1;
				}
				else
				{
					return 0;
				}
			}
			else
			{
				return 0; //if computer has no offsets
			}
		}
		return 0;
	} //

	void Send_Offsets(int16_t A[3], int16_t G[3], int16_t M[3], int16_t T,int16_t gain[3])
	{
		uint8_t i;
		write_To_Port(START_SIGN,2);
		write_To_Port(28,2);
		write_To_Port(OFFSET_ID,2);
		write_To_Port(0x01,2);//mode
		for(i=0;i<3;i++)
		{
			write_To_Port(A[i],2);
			write_To_Port(G[i],2);
			write_To_Port(M[i],2);
			write_To_Port(gain[i],2);
		}
		write_To_Port(T,2);
	}//42 bytes sent

	void Send_Config(int16_t params[20])
	{
		uint8_t i;
		write_To_Port(START_SIGN,2);
		write_To_Port(28,2);
		write_To_Port(CONFIG_ID,2);
		write_To_Port(0x01,2);//mode
		for(i=0;i<20;i++)
		{
			write_To_Port(params[i],2);
		}
	}

	bool Get_Config(int16_t params[20])
	{
		uint8_t i;
		int16_t START_ID, message_ID, len;
		
		write_To_Port(START_SIGN,2);//start sign
		write_To_Port(8,2); 		//length of payload
		write_To_Port(CONFIG_ID,2); //tell the GCS that I want them sweet sweet configs.
		write_To_Port(0x01,2);

		delay(1000);//wait 1 second for the data to come in
		if(Serial.available())
		{
			START_ID = Serial.read()|int16_t(Serial.read()<<8); //start sign
			if(START_ID == START_SIGN)
			{
				len = Serial.read()|int16_t(Serial.read()<<8); //length of packet 40 bytes
				message_ID = Serial.read()|int16_t(Serial.read()<<8);
				Serial.read()|int16_t(Serial.read()<<8);//waste
				if(message_ID==CONFIG_ID && len == 40)//confirm that you are getting the offsets and nothing else.
				{
					for(i=0;i<20;i++)//computer has offsets
					{
						params[i] = Serial.read()|int16_t(Serial.read()<<8);
					}
					return 1;
				}
				else
				{
					return 0;
				}
			}
			else
			{
				return 0; //if computer has no offsets
			}
		}
		return 0;
	}

	void Get_WP(float &X, float &Y, float &slope, int16_t &point)
	{
		received_stamp = millis();
		X = float(int16_t(Serial.read()|int16_t(Serial.read()<<8) ) )*1e-2; //coordinates transfered wrt to origin, converted 
		Y = float(int16_t(Serial.read()|int16_t(Serial.read()<<8) ) )*1e-2; //coordinates transfered wrt to origin, converted 
		slope = float(int16_t(Serial.read()|int16_t(Serial.read()<<8) ) )*1e-2;
		point = int16_t(Serial.read()|int16_t(Serial.read()<<8) );	
	}

	bool Send_WP(float X, float Y, float slope,int16_t point)
	{
		if(millis() - transmit_stamp > 100)
		{
			transmit_stamp = millis();
			int x = int16_t(X*1e2);
			int y = int16_t(Y*1e2);
			int m = int16_t(slope*1e2);

			write_To_Port(START_SIGN,2);
			write_To_Port(8,2);
			write_To_Port(WP_ID,2);
			write_To_Port(0x01,2);
			write_To_Port(x,2);
			write_To_Port(y,2);
			write_To_Port(m,2);
			write_To_Port(point,2);
			return 1;
		}
		return 0;
	}//10 bytes 

	void Send_Calib_Command(uint8_t id)
	{
		write_To_Port(START_SIGN,2);
		write_To_Port(8,2);
		if(id == 1)
			write_To_Port(GYRO_CAL,2);
		if(id == 2)
			write_To_Port(ACCEL_CAL,2);
		if(id == 3)
			write_To_Port(MAG_CAL,2);
		if(id == 4)
			write_To_Port(DONE,2);
		else
			write_To_Port(ERROR_CODE,2);
		
		write_To_Port(0x01,2); //mode
	}//2 bytes

	// void send_heartbeat(); 
	void Send_State(byte mode,double lon, double lat,double gps_lon, double gps_lat, float vel, float heading, float pitch, float roll,float Accel, float opError, float pError, float head_Error, float VelError, float Time, float Hdop, int16_t comp_status)//position(2), speed(1), heading(1), acceleration(1), Position Error
	{
		if(millis() - transmit_stamp > 100)
		{
			transmit_stamp = millis();
			long out[15];
			out[0] = lon*1e7;
			out[1] = lat*1e7;//hopefully this is correct
			out[2] = gps_lon*1e7;
			out[3] = gps_lat*1e7;
			out[4] = vel*1e2;
			out[5] = heading*1e2;
			out[6] = pitch*1e2;
			out[7] = roll*1e2;
			out[8] = Accel*1e2;
			out[9] = opError*1e3;
			out[10] = pError*1e3;
			out[11] = head_Error*1e3;
			out[12] = VelError*1e3;
			out[13] = Time;
			out[14] = int32_t(Hdop*1e3)<<16|int32_t(comp_status);

			write_To_Port(START_SIGN,2);
			write_To_Port(68,2);
			write_To_Port(STATE_ID,2);
			write_To_Port(int16_t(mode),2);
			for(int i=0;i<15;i++)
			{
				write_To_Port(out[i],4);
			}
		}
	}// 30 bytes

	uint16_t check()
	{
		uint16_t START_ID,message_ID;

		if(millis() - received_stamp > 100) //10 Hz 
		{
			received_stamp = millis();
			if(Serial.available())
			{
				failsafe_stamp = millis();
				START_ID = Serial.read()|int16_t(Serial.read()<<8); //start sign
				if(START_ID == START_SIGN)
				{
					msg_len = Serial.read()|int16_t(Serial.read()<<8); //length of packet
					message_ID = Serial.read()|int16_t(Serial.read()<<8);	
					mode = Serial.read()|int16_t(Serial.read()<<8); 
					failsafe = false;
					return message_ID;
				}
				else
				{
					while(Serial.available())
					{
						Serial.read();
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
