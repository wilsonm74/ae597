
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



/// @brief  
///
///  Inputs:
///     Called from main: handles the needed calls to other functions to generate proper thrust commands to the viewer sphere
void ViewerController(int maneuverNumber, state_vector trackingError){

    // Create arrays hope we won't need to alloc at any point
    float gains[6] = {0}; 
    control_vector resultant_command;

    // Determine proper gains
    SelectGains(gains, maneuverNumber);

    
    
    // Call the provided controller function with selected gains
    ctrlAttitudeNLPDwie(gains[0],
                        gains[1],
                        gains[2],
                        gains[3],
                        gains[4],
                        gains[5],
                        trackingError,
                        resultant_command);

    ctrlMixWLoc(); // Generate mix to get firing times
    
    propSetThrusterTimes();


}

/// @brief  
///
///  Inputs:
///     float* gains used to pass gains back to viewerController when we call the 
void SelectGains(float* gains, int maneuverNumber){

    switch (maneuverNumber)
    {
    case  1: // Acquisition
        gains[0] = 1; // Replace with real values
        gains[1] = 1; // Replace with real values
        gains[2] = 1; // Replace with real values
        gains[3] = 1; // Replace with real values
        gains[4] = 1; // Replace with real values
        gains[5] = 1; // Replace with real values
        break;

    case 2: // Tracking 
        gains[0] = 1; // Replace with real values
        gains[1] = 1; // Replace with real values
        gains[2] = 1; // Replace with real values
        gains[3] = 1; // Replace with real values
        gains[4] = 1; // Replace with real values
        gains[5] = 1; // Replace with real values
    default: // Have default use the initially provided values
        gains[0] = 1; // Replace with real values
        gains[1] = 1; // Replace with real values
        gains[2] = 1; // Replace with real values
        gains[3] = 1; // Replace with real values
        gains[4] = 1; // Replace with real values
        gains[5] = 1; // Replace with real values
        break;
    }


}

