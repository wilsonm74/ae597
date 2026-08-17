#include "spheres_types.h"


void ViewerController(int maneuverNumber, state_vector viewCurState, state_vector leadCurState, control_vector * viewControl);
void SelectGains(float gains[6], int maneuverNumber);
