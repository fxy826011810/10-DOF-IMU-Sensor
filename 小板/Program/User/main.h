#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "monitor.h"
#include "ahrs.h"
typedef struct
{
	uint32_t heart;
	struct 
	{
	Icm20602Datadef 		Data;
	System_Monitor_t 		monitor;	
	}Icm20602;
	struct 
	{
		uint8_t						status;
		magDatadef				Data;
		System_Monitor_t 	monitor;	
	}Ist8310;
	struct 
	{
		Ms5611DataDef			Data;
		Ms5611Status			Status;
		System_Monitor_t 	monitor;
	}Ms5611;
	ahrs_t ahrs;
}cmd_t;

extern cmd_t cmd;


#endif

