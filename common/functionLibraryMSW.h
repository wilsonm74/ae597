#ifndef FUNCTIONLIBRARYMSW_H
#define FUNCTIONLIBRARYMSW_H

#include "spheres_types.h"

void trajectoryMixAndQuantize(prop_time *firing_times, float control[6],
    float state[13], float pulse_demand_ms[12]);

state_vector trajectory_origin;

#endif
