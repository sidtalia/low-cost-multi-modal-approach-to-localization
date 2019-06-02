#ifndef _MEMORY_H_
#define _MEMORY_H_

#include"Arduino.h"
#include<EEPROM.h>

void store_memory(int j, int16_t offA[3], int16_t offG[3], int16_t offM[3], int16_t offT, int16_t axis_gain[3] )
{
	uint16_t A[3],G[3],M[3],T,gain[3];
	uint16_t add = j*26;
	for(int i = 0;i<3;i++)
	{
		A[i] = uint16_t(offA[i]);
		EEPROM.write(add,A[i]);
		add += 2;

		G[i] = uint16_t(offG[i]);
		EEPROM.write(add,G[i]);
		add += 2;

		M[i] = uint16_t(offM[i]);
		EEPROM.write(add,M[i]);
		add += 2;

		gain[i] = uint16_t(axis_gain[i]);
		EEPROM.write(add,gain[i]);
		add += 2;
	}
	T = uint16_t(offT);
	EEPROM.write(add,T);
}

bool check_memory()
{
	uint16_t data[2];
	EEPROM.read(0,&data[0]);
	EEPROM.read(2,&data[1]);
	if(data[0]!=data[1])
	{
		return 1;
	}
	return 0;
}

void read_memory(int j, int16_t offA[3], int16_t offG[3], int16_t offM[3], int16_t &offT, int16_t axis_gain[3])
{
	uint16_t A[3],G[3],M[3],T,gain[3];
	uint16_t add = j*26;
	for(int i = 0;i<3;i++)
	{
		EEPROM.read(add, &A[i]);
		offA[i] = int16_t(A[i]);
		add += 2;
		EEPROM.read(add, &G[i]);
		offG[i] = int16_t(G[i]);
		add += 2;
		EEPROM.read(add, &M[i]);
		offM[i] = int16_t(M[i]);
		add += 2;
		EEPROM.read(add, &gain[i]);
		axis_gain[i] = int16_t(gain[i]);
		add += 2;
	}
	EEPROM.read(add, &T);
	offT = int16_t(T);
}

void store_config(int16_t param[20])
{
	uint16_t config;
	uint16_t add = 52; //this is the offset to prevent memory overlap between imu offsets and config stuff.
	for(int i = 0;i<20;i++)
	{
		config = uint16_t(param[i]);
		EEPROM.write(add,config);
		add += 2;
	}
}

bool check_config()
{
	uint16_t data[2];
	EEPROM.read(52,&data[0]);
	EEPROM.read(53,&data[1]);
	if(data[0]!=data[1])
	{
		return 1;
	}
	return 0;
}

void read_config(int16_t param[20])
{
	uint16_t config;
	uint16_t add = 52;
	for(int i = 0;i<20;i++)
	{
		EEPROM.read(add, &config);
		param[i] = int16_t(config);
		add += 2;
	}
}

#endif
