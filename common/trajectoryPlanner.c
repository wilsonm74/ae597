/*
 * trajectoryPlanner.c
 *
 * Stage 1: Planned Trajectory. See trajectoryPlanner.h.
 */

#include "trajectoryPlanner.h"
#include <math.h>

void plannedTrajectoryGenerate(trajectory_path_t *path)
{
	unsigned int i;
	float t_ms;
	float angle;

	path->numPoints = TRAJ_NUM_POINTS;
	for (i = 0; i < TRAJ_NUM_POINTS; i++) {
		t_ms = (float)(i * TRAJ_CTRL_PERIOD_MS);
		angle = (CIRCLE_TWO_PI * t_ms) / (float)CIRCLE_PERIOD_MS;

		path->pos[i][0] = CIRCLE_RADIUS_X * cosf(angle);  // X
		path->pos[i][1] = CIRCLE_CENTER_Y;                // Y
		path->pos[i][2] = CIRCLE_RADIUS_Z * sinf(angle);  // Z
	}
}
