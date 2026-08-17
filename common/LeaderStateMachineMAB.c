//Author: Matthew Bennett

#include "control.h"
#include "gsp.h"
#include "spheres_types.h"
#include "spheres_physical_parameters.h"
#include "math.h"
#include "LeaderStateMachineMAB.h"

//create a static variable to track the state
static int leaderCurrentState = 1;
static int acquisitionThreshold = 5;

/// @brief Updatees the current
/// @param newManeuver 
///     The new maneuver the leader should transition to. It is the callers responsibility to call the corrrect maneuverupdate. 
void UpdateLeaderStateMachine(int newManeuver){
    leaderCurrentState = newManeuver;
}

int CheckLeaderStateMachine(){
    return leaderCurrentState;
}