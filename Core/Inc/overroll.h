#ifndef __OVERROLL_H
#define __OVERROLL_H

#include "main.h"
extern uint16_t StateFlag;
extern volatile float global_angle;
typedef enum{ 
	STATE_Tast = 99,
	STATE_A_Start=0,
	STATE_A1  =1,
	STATE_A2  =2,
	STATE_A3  =3,
	STATE_A4 = 4,
	STATE_A_1 =5,
	STATE_B_Start = 6,
	STATE_B1  =7,
	STATE_B2  =8,
	STATE_B3  =9,
	STATE_B4  =10,
	STATE_C_Start,
	STATE_C,
	STATE_C_1,
	STATE_idle = -1
}State;
extern State MyPos;

#endif
