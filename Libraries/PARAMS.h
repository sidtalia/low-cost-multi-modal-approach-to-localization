#ifndef _PARAMS_H_
#define _PARAMS_H_

#define dt (float)0.0025
#define LOOP_FREQUENCY 400
#define dt_micros 2500

#define ride_height (float)0.05 //6.2cms height of the sensor.
#define DIST_BW_ACCEL_AXLE (float)0.225// distance between rear axle and accelerometer
#define THROTTLE_OFFSET 1535
#define DECLINATION (float)-1.5

#define MODE_STOP 0x00
#define MODE_STANDBY 0X01
#define MODE_MANUAL 0x02
#define MODE_PARTIAL 0X03
#define MODE_AUTO 0x04
#define MODE_AUTO_LUDICROUS 0x05
#define MODE_NO_GPS 0x06

#define FIX_TIMEOUT 1000

#define LUDICROUS 0x05 //interchangable with MODE_AUTO_LUDICROUS
#define CRUISE 0x04 //interchangable with MODE_AUTO

#define START_SIGN 0xFE
#define OFFSET_ID 0x01
#define COMMAND_ID 0X02

#define CONTROL_FREQUENCY LOOP_FREQUENCY/2
#define FUTURE_TIME (float)2/CONTROL_FREQUENCY
#define CONTROL_TIME (float)1000/CONTROL_FREQUENCY //control time in ms

#define WP_CIRCLE 0.5 //1/2 meter radius around waypoint.

#define WP_ID 0x0005
#define STATE_ID 0x0006
#define MODE_ID 0x0007
#define CLEAR_ID 0x0008
#define MARK_ID 0x0009
#define CALIB_ID 0x000A

#define GYRO_CAL 0x10
#define ACCEL_CAL 0x20
#define MAG_CAL 0x30
#define DONE 0x40

#define ERROR_CODE 0xFF

#define COM_BAUD 230400
#define GPS_BAUD 230400
#define JEVOIS_BAUD 9600

#endif
