#ifndef FUNCTIONLIBRARYMSW_H
#define FUNCTIONLIBRARYMSW_H

#include "spheres_types.h"

void trajectoryMixAndQuantize(prop_time *firing_times, float control[6], float state[13], float pulse_demand_ms[12]);

void generateTrajectory(state_vector target, float acceleration[3], unsigned int maneuver_time);

void trajectoryPhase(float t_s, float period_s, float *phase, float *phase_rate, float *phase_accel);

state_vector trajectory_origin;

#endif
