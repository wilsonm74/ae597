//Author: Matthew Bennett

//We need to be able to generate a set of thruster commands based on the
//incoming trackingError.
    //  We can use the ctrlAttitudeNLPDwie method which will need the 6 gains
    // We can plan for this to use the default values for the gains for now but
    //  create two sets controlled by the viewer maneuver number in case we need
    //  different gains for that phase. We can have that check be it's own helper function to keep things nice and neat.



#include "control.h"
#include "gsp.h"
#include "spheres_types.h"
#include "spheres_physical_parameters.h"
#include "ViewerControllerMAB.h"
#include "ctrl_attitude.h"
#include "ctrl_mix.h"
#include "prop.h"
#include "trackingError.h"


extern const float KPattitudePD, KDattitudePD;

/// @brief  
///
///  Inputs:
///     Called from main: handles the needed calls to other functions to generate proper thrust commands to the viewer sphere
void ViewerController(int maneuverNumber, state_vector viewCurState, state_vector leadCurState, control_vector * viewControl){

    // Create arrays hope we won't need to alloc at any point
    float gains[6] = {0}; 
    prop_time firing_times;
    state_vector targetVector;
    state_vector trackingError;
    
    SelectGains(gains, maneuverNumber);    // Determine proper gains based on maneuver number
    GetTargetVector(viewCurState, leadCurState, &targetVector); // Calculate the target vector
    findStateError(trackingError ,targetVector, viewCurState); // Calculate the state error between the target and current state
    
    
    // Call the provided controller function with selected gains
    ctrlAttitudeNLPDwie(gains[0],
                        gains[1],
                        gains[2],
                        gains[3],
                        gains[4],
                        gains[5],
                        trackingError,
                        *viewControl);
}

/// @brief  
///
///  Inputs:
///     
void SelectGains(float gains[6], int maneuverNumber){

    switch (maneuverNumber)
    {
    case  1: // Acquisition
        gains[0] = KPattitudePD; // 
        gains[1] = KDattitudePD; //
        gains[2] = KPattitudePD; // 
        gains[3] = KDattitudePD; // 
        gains[4] = KPattitudePD; // 
        gains[5] = KDattitudePD; // 
        break;

    case 2: // Tracking 
        gains[0] = KPattitudePD; // 
        gains[1] = KDattitudePD; //
        gains[2] = KPattitudePD; // 
        gains[3] = KDattitudePD; // 
        gains[4] = KPattitudePD; // 
        gains[5] = KDattitudePD; // 
        break;
    default: // Have default use the initially provided values
        gains[0] = KPattitudePD; // 
        gains[1] = KDattitudePD; //
        gains[2] = KPattitudePD; // 
        gains[3] = KDattitudePD; // 
        gains[4] = KPattitudePD; // 
        gains[5] = KDattitudePD; // 
        break;
    }


}

