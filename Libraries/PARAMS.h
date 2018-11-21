#ifndef _PARAMS_H_
#define _PARAMS_H_

#define dt (float)0.0025
#define dt_micros 2500

#define MODE_STOP 0x00
#define MODE_STANDBY 0X01
#define MODE_MANUAL 0x02
#define MODE_PARTIAL 0X03
#define MODE_AUTO 0x04
#define MODE_NO_GPS 0x05

#define START_SIGN 0X00FE
#define OFFSET_ID 0x0001
#define COMMAND_ID 0X0002


#define WP_ID 0x0005
#define STATE_ID 0x0006
#define MODE_ID 0x0007

#define GYRO_CAL 0xF000
#define ACCEL_CAL 0xE000
#define MAG_CAL 0xD000

#define ERROR_CODE 0xFFFF

#define COM_BAUD 115200
#define GPS_BAUD 230400
#define JEVOIS_BAUD 9600

#endif