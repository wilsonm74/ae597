//Author: Matthew Bennett

#include "control.h"
#include "gsp.h"
#include "spheres_types.h"
#include "spheres_physical_parameters.h"
#include "math.h"

//create a static variable to track the state
static int viewerCurrentState = 1;
static int acquisitionThreshold = 5;

void UpdatViewerStateMachine(float trackingError){
    if(viewerCurrentState == 1 && abs(trackingError) < acquisitionThreshold){  
        viewerCurrentState = 2;
    }

    // We need this to also call the update to the Leader State Machine

}

int CheckStateMachine(){
    return viewerCurrentState;
}