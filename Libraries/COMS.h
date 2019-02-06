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
		int16_t START_ID, message_ID, len,mode;
		
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
				mode = Serial.read()|int16_t(Serial.read()<<8);
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

	void Get_WP(double &X, double &Y)
	{

		X = float(int32_t( Serial.read()|int32_t(Serial.read()|int32_t(Serial.read()|int32_t(Serial.read()<<8)<<8)<<8)<<8 ) )*1e-7; //coordinates transfered wrt to origin, converted 
		Y = float(int32_t( Serial.read()|int32_t(Serial.read()|int32_t(Serial.read()|int32_t(Serial.read()<<8)<<8)<<8)<<8 ) )*1e-7; //coordinates transfered wrt to origin, converted 
	}

	void Send_WP(double X, double Y)
	{
		long x = long(X*1e7);
		long y = long(Y*1e7);

		write_To_Port(START_SIGN,2);
		write_To_Port(8,2);
		write_To_Port(WP_ID,2);
		write_To_Port(0x01,2);
		write_To_Port(x,4);
		write_To_Port(y,4);
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
	void Send_State(byte mode,double lon, double lat, float vel, float heading, double pitch, double roll,float Accel, float opError, float pError, float head_Error, float VelError, float Time, float Hdop)//position(2), speed(1), heading(1), acceleration(1), Position Error
	{
		if(millis() - transmit_stamp > 100)
		{
			transmit_stamp = millis();
			long out[13];
			out[0] = lon*1e7;
			out[1] = lat*1e7;//hopefully this is correct
			out[2] = vel*1e2;
			out[3] = heading*1e2;
			out[4] = pitch*1e7;
			out[5] = roll*1e7;
			out[6] = Accel*1e2;
			out[7] = opError*1e3;
			out[8] = pError*1e3;
			out[9] = head_Error*1e3;
			out[10] = VelError*1e3;
			out[11] = Time;
			out[12] = Hdop*1e3;

			write_To_Port(START_SIGN,2);
			write_To_Port(56,2);
			write_To_Port(STATE_ID,2);
			write_To_Port(int16_t(mode),2);
			for(int i=0;i<13;i++)
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
