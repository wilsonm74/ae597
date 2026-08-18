#include "state_machine.h"
#include "system.h"
#include "spheres_constants.h"
#include <math.h>

#define TRACKING_ERROR_THRESHOLD_DEG 5.0

static viewerStates currentViewerState;
static leaderStates currentLeaderState;

void leaderStateMachineInit(void) {
	currentLeaderState = STANDBY;
}

void viewerStateMachineInit(void) {
	currentViewerState = ACQUISITION;
}

void leaderStateMachineUpdate(viewerStates viewerState, unsigned char trajectoryComplete, unsigned char boundsExceeded) {
	switch (currentLeaderState) {
		case STANDBY:
			if (viewerState == TRACKING) {
				currentLeaderState = FOLLOWING;
			}
			break;
		case FOLLOWING:
			if (trajectoryComplete != 0 || boundsExceeded != 0) {
				currentLeaderState = RETURNING;
			}
			break;
	}
}

leaderStates leaderStateMachineGetState(void) {
	return currentLeaderState;
}

void viewerStateMachineUpdate(float trackingError, unsigned char trackingErrorValid) {
	switch (currentViewerState) {
		case ACQUISITION:
			if (fabsf(trackingError) < TRACKING_ERROR_THRESHOLD_DEG && trackingErrorValid != 0) {
				currentViewerState = TRACKING;
			}
			break;
	}
}

viewerStates viewerStateMachineGetState(void) {
	return currentViewerState;
}
