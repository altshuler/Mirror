
#ifndef __MOTION_TASK_H
#define __MOTION_TASK_H

#define VELOCITY_CALC_PERIOD 	200 // Velocity calculation frequency =10 Hz in quantities of  0.5 ms  (as period of  TMR3- CCR4)
#define OVERSHOOT_FACTOR		4
#define EMERGENCY_NOT_ACTIVE	0
#define EMERGENCY_ACTIVE		1
#define MAX_EMRG_DEBOUNCE_DELAY 12 // 6ms delay in motion_task (messages received any 0.5 ms on queue)



struct sEmergencyStatus
{
	uint8_t  Emergency1Flag;
	uint8_t  Emergency2Flag;
	uint8_t  Emerg1DebounceTmr;
	uint8_t  Emerg2DebounceTmr;
	uint8_t  EmergencyFlagsState;
};



void SetNextPosition(uSSI Encoder);
void PedestalPositionCmd(void);


#endif
