#ifndef _GPS_H
#define _GPS_H

#include"Arduino.h"
#include"PARAMS.h"
#include"SIDMATH.h"

#define GPS_BAUD 230400 
/*
 * I call this tab the "fast_GPS" because it's faster than using the "tinyGPS" library.
 * I m using UBX-POSLLH protocol which, as the name suggests, gives me the POSition in Longitude,Latitude and Height
 * as well as some other useful data. The reason for using this is that it contains the bare minimum information 
 * the bot would need, therefore reducing the time required to transfer the data to ~5ms down from approximately 20ms 
 * when using the standard protocols (forgot their names, I guess they were called GNSS,GLONAS etc, I could be wrong
 * so don't quote me anywhere).
 * ALSO, full disclosure, I did not write this code, I picked it up from the internet from the guy that made the video of 
 * "10Hz update rate on ublox gps". I did makes some changes but they are just for my convenience. All credit for this stuff goes to him.
 */


const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };  //header of the incoming signal

struct NAV_POSLLH    //structure in which all the data will be stored
{
  unsigned char cls;  //class
  unsigned char id;   //id
  unsigned short len;  //length of packet
  unsigned long iTOW;   //time in microseconds since first fix
  long lon;              //longitude 
  long lat;               //latitude
  long height;          //height
  long hMSL;            //height above mean sea level
  unsigned long hAcc;     //horizontal accuracy 
  unsigned long vAcc;     //vertical accuracy 
};

class GPS
{
public:
	NAV_POSLLH posllh;
	// long iTOW;
	// float VelNED[3];
	   //object of structure NAV_POSLLH
 	double longitude,latitude,Hdop;//,last_longitude,last_latitude,height,last_height;
 	bool tick,configured;
 	uint8_t config_msg_POSLLH[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47};
 	uint8_t config_msg_rate[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
 	uint8_t config_msg_baud[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
 					   0x00, 0x84, 0x03, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0xE8};
 	uint8_t config_msg_save[21] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
					   0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF}; 					   

 	GPS()
 	{
 		tick = false;
 		Hdop = 100000; //initial Hdop. This helps me differentiate whether data came in from gps or if gps has not even initialized yet.
 	}

	void sendPacket(uint8_t *packet, byte len) //send commands to gps for configuration purposes
	{
	    for (byte i = 0; i < len; i++)
	    {
	        Serial1.write(packet[i]);
	    }
	}

 	void disableNmea() //diable the god awful NMEA messages. This code was copied from https://github.com/1oginov/UbxGps
	{
	    // Array of two bytes for CFG-MSG packets payload.
	    byte messages[][2] = {
	        {0xF0, 0x0A},
	        {0xF0, 0x09},
	        {0xF0, 0x00},
	        {0xF0, 0x01},
	        {0xF0, 0x0D},
	        {0xF0, 0x06},
	        {0xF0, 0x02},
	        {0xF0, 0x07},
	        {0xF0, 0x03},
	        {0xF0, 0x04},
	        {0xF0, 0x0E},
	        {0xF0, 0x0F},
	        {0xF0, 0x05},
	        {0xF0, 0x08},
	        {0xF1, 0x00},
	        {0xF1, 0x01},
	        {0xF1, 0x03},
	        {0xF1, 0x04},
	        {0xF1, 0x05},
	        {0xF1, 0x06},
	    };

	    // CFG-MSG packet buffer.
	    byte packet[] = {
	        0xB5, // sync char 1
	        0x62, // sync char 2
	        0x06, // class
	        0x01, // id
	        0x03, // length
	        0x00, // length
	        0x00, // payload (first byte from messages array element)
	        0x00, // payload (second byte from messages array element)
	        0x00, // payload (not changed in the case)
	        0x00, // CK_A
	        0x00, // CK_B
	    };
	    byte packetSize = sizeof(packet);

	    // Offset to the place where payload starts.
	    byte payloadOffset = 6;

	    // Iterate over the messages array.
	    for (byte i = 0; i < sizeof(messages) / sizeof(*messages); i++)
	    {
	        // Copy two bytes of payload to the packet buffer.
	        for (byte j = 0; j < sizeof(*messages); j++)
	        {
	            packet[payloadOffset + j] = messages[i][j];
	        }

	        // Set checksum bytes to the null.
	        packet[packetSize - 2] = 0x00;
	        packet[packetSize - 1] = 0x00;

	        // Calculate checksum over the packet buffer excluding sync (first two) and checksum chars (last two).
	        for (byte j = 0; j < packetSize - 4; j++)
	        {
	            packet[packetSize - 2] += packet[2 + j];
	            packet[packetSize - 1] += packet[packetSize - 2];
	        }

	        sendPacket(packet, packetSize);
	    }
	}


	void calcChecksum(unsigned char* CK)    //function to calculate expected checksum 
	{   
	  memset(CK, 0, 2);
	  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) 
	  {
	    CK[0] += ((unsigned char*)(&posllh))[i];
	    CK[1] += CK[0];
	  }
	}

	bool processGPS()    //bool function to tell us whether all the data has come in or not
	{
	  static int fpos = 0;                //variable to keep a track of where we are in the structure
	  static unsigned char checksum[2];       //variables for storing checksum given in the message
	  const int payloadSize = sizeof(NAV_POSLLH);     //size of payload

	  while ( Serial1.available() ) //while there is something on the UART/serial port. 
	  {
	    byte c = Serial1.read();  //put data read from serial port into c
	    if ( fpos < 2 )              //checking for first 2 bits of data 
	    {
	      if ( c == UBX_HEADER[fpos] )  //if the data in the first 2 bits matches up with the known standard first 2 bits, increment fpos
	        fpos++;
	      else
	        fpos = 0;
	    }
	    
	    else                            //when fpos>2 
	    {
	      if ( (fpos-2) < payloadSize )                   //when the first 2 bits of data have been read   
	      {  
	        ((unsigned char*)(&posllh))[fpos-2] = c;  //put the next bits of data into the object posllh of structure NAV_POSLLH by simply incrementing the position at which the data 
	      }                                           //is being written rather than mentioning the name of the data etc    
	      fpos++;                                     //i feel this should be fpos+=sizeof(c) because this didn't run properly in turbo c++
	                                                  //it could be possible that in arduino IDE, incrementing the position implicitly means going to the next variable space.
	                                                    
	      if ( fpos == (payloadSize+2) )  //when payload has been read, calculate the expected checksum 
	      {
	        calcChecksum(checksum);
	      }
	      else if ( fpos == (payloadSize+3) ) //compare our checksum with the given checksum
	      {
	        if ( c != checksum[0] )
	          fpos = 0;                   //if checksum fails, the data can't be trusted, hence, move fpos back to 0
	      }
	      else if ( fpos == (payloadSize+4) )   // compare given checksum with our checksum
	      {
	        fpos = 0;
	        if ( c == checksum[1] ) 
	        {
	          return true;     //data is valid, return true to indicate that we should now look at the data 
	        }
	      }
	      else if ( fpos > (payloadSize+4) )  //when the entire sentence has been parsed, move fpos back to 0 
	      {
	        fpos = 0;
	      }
	    }
	  }
	  return false;
	}
	inline void updategps()  //make sure that you have while(!processGPS()){} before calling this if you are relying on an update from this function
	{
	    longitude = double(posllh.lon)*1e-7;
	    latitude = double(posllh.lat)*1e-7;
	    Hdop= double(posllh.hAcc)*1e-3; //HAcc in meters.
	    //stuff below this point is for benchmarking purposes and can be removed if not using bolderflight uNav
	 //    height = double(posllh.height)*1e-3;
	 //    iTOW = posllh.iTOW;
	 //    VelNED[0] = (latitude - last_latitude)*DEG2METER; //yes I know gps velocity is based on doppler effect but thats only for high speeds. for speeds less than 5m/s, i doubt doppler would help.
		// VelNED[1] = (longitude- last_longitude)*DEG2METER;
		// VelNED[2] = (last_height - height);
		// last_longitude = longitude;
		// last_latitude = latitude;
		// last_height = height;
	}
	void localizer() //function to figure out our original location. not inline because it is called only once
	{
	  tick = false;//falsify tick so that we know a new message was actually received
	  if(processGPS())
	  {
	    updategps();
	    tick = true; // this tick is used to signify that new gps data has arrived.
	  }
	}

	bool initialize()
	{
		long timeout = millis();
		while(millis()-timeout < 300)
		{
			localizer();
			if(tick)
			{
				break;
			}
		}
		if(!tick)
		{
			Serial1.flush();
			Serial1.begin(9600);
			disableNmea();
			sendPacket(config_msg_POSLLH,sizeof(config_msg_POSLLH));
			delay(100);
			sendPacket(config_msg_rate,sizeof(config_msg_rate));
			delay(100);
			sendPacket(config_msg_baud,sizeof(config_msg_baud));
			Serial1.flush();
			Serial1.begin(GPS_BAUD);//reset baud.
			
			sendPacket(config_msg_save,sizeof(config_msg_save));
			delay(100);
			localizer();
		}
		return tick;
	}

	int8_t fix_type()
	{
		while(!tick)
		{	
			localizer();
			delay(5);
		}
		if(Hdop > 100)
		{
			return 0;//not useful
		}
		if(Hdop > 10)
		{
			return 1;//meh
		}
		if(Hdop < 2.5)
		{
			return 2; //good enough
		}
		if(Hdop < 1.0)
		{
			return 3;//usefully accurate
		}
		if(Hdop < 0.6)
		{
			return 4;//very accurate
		}
		if(Hdop < 0.55)
		{
			return 5;//dead accurate
		}
		return -1; //out of bound kind of answer
	}

	bool fix_initial_position()//get the averaged out value of position over a period of 1 second.
	{

		double estimate_HDOP;
		double estimate_lat,estimate_long;
		double gain;
		uint16_t timer,timeout;

		localizer();
		timer = timeout = millis();
		estimate_HDOP = Hdop; 
		estimate_long = longitude;
		estimate_lat = latitude;
		if(Hdop>100000)
		{
			return 0; //break if gps unavailable.
		}
		//this process should take about 16 seconds tops after the gps fix has been received..
		while(estimate_HDOP>0.0001) 
		{
			localizer();
			if( millis() - timer > 1500 ) //1.5 seconds
			{
				timer = millis(); //reset timer 

				gain = estimate_HDOP/(estimate_HDOP + 0.1*Hdop);

				estimate_lat = gain*latitude + (1-gain)*estimate_lat ; //assuming the vehicle isn't moving
				estimate_long = gain*longitude + (1-gain)*estimate_long ;
				
				estimate_HDOP *= (1-gain); //reduce error.
			}
			if( millis() - timeout > 15000 )//15 second time out.
			{
				return 0;
			}
		}

		latitude = estimate_lat;
		longitude = estimate_long;

		return 1;
	}
};



#endif
