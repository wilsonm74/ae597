// state_machine.h

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum {
	ACQUISITION = 0,
	TRACKING = 1
} viewerStates;

typedef enum {
	STANDBY = 0,
	FOLLOWING = 1,
	RETURNING = 2
} leaderStates;

void leaderStateMachineInit(void);
void viewerStateMachineInit(void);

void leaderStateMachineUpdate(viewerStates viewerState, unsigned char trajectoryComplete, unsigned char boundsExceeded);
void viewerStateMachineUpdate(float trackingError, unsigned char trackingErrorValid);

leaderStates leaderStateMachineGetState(void);
viewerStates viewerStateMachineGetState(void);

#endif