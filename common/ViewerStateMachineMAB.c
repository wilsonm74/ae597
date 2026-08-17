//Author: Matthew Bennett

// This file holds functions for tracking updating the state viewer state machine
// The static viewerCurrentState parameter will last throughout runtime
// Functions that need the current state of the state machine can check it's state via CheckStateMachine()
// By isolating this value we can avoid mistaken reassignments in other areas in the coe. 


#include "control.h"
#include "gsp.h"
#include "spheres_types.h"
#include "spheres_physical_parameters.h"
#include "math.h"

//create a static variable to track the state
static int viewerCurrentState = 1;
static int acquisitionThreshold = 5;

/// @brief Takes in the current tracking error and updates the viewer state after acquisition
/// @param trackingError 
void UpdatViewerStateMachine(float trackingError){
    if(viewerCurrentState == 1 && abs(trackingError) < acquisitionThreshold){  
        viewerCurrentState = 2;
        UpdateLeaderStateMachine(viewerCurrentState); //Update the leader to begin trajectory
    }

}

int CheckStateMachine(){
    return viewerCurrentState;
}