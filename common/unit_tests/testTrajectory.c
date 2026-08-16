
#include <stdio.h>
#include "functionLibraryMSW.h"

#define CIRCLE_PERIOD_S 120.0f

/* Mock external variables */
const float VEHICLE_MOMENT_ARM = 0.1f;
const float VEHICLE_THRUST_FORCE = 0.5f;
const float VEHICLE_MASS = 4.43f;
const float KPattitudePD = 0.0036f;
const float KDattitudePD = 0.0180f;
const float KPpositionPD = 0.0200f;
const float KDpositionPD = 0.1880f;
static unsigned int control_period_ms = 100;

/* Mock function */
unsigned int ctrlPeriodGet(void) {
    return control_period_ms;
}

int main(void) {

	float dt = 0.01f;
	float dur = 100.0f;
	float t;

	state_vector target;
	float acceleration[3];

	FILE *file = fopen("trajectoryPhaseSimple_output.csv", "w");

	fprintf(file, "t, state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9], state[10], state[11], state[12]\n");

	for (t = 0.0f; t < dur; t += dt) {
		float phase, phase_rate, phase_accel;
		generateTrajectory(target, acceleration, t * 1000.0f);
		fprintf(file, "%f, ", t);
		for (int i = 0; i < 13; i++) {
			fprintf(file, "%f, ", target[i]);
		}
		fprintf(file, "\n");
	}

	fclose(file);
}



