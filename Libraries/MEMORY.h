#ifndef _MEMORY_H_
#define _MEMORY_H_

#include"Arduino.h"
#include<EEPROM.h>

void store_memory(int i, int16_t offA[3], int16_t offG[3], int16_t offM[3], int16_t offT)
{
	uint16_t A[3],G[3],M[3],T;
	uint16_t add = i*20;
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
	}
	T = uint16_t(offT);
	EEPROM.write(add,T);
}

bool check_memory()
{
	uint16_t data;
	EEPROM.read(0,&data);
	if(data)
	{
		return 1;
	}
	return 0;
}

void read_memory(int i, int16_t offA[3], int16_t offG[3], int16_t offM[3], int16_t offT)
{
	uint16_t A[3],G[3],M[3],T;
	uint16_t add = i*20;
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
	}
	EEPROM.read(add, &T);
	offT = int16_t(T);
}

#endif