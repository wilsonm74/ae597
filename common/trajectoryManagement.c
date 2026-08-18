/*
 * trajectoryManagement.c
 *
 * Stage 2: Trajectory Management. See trajectoryManagement.h.
 */

#include "trajectoryManagement.h"
#include <math.h>
#include <stdbool.h>

#define SQUARE(x) ((x)*(x))

void trajectoryManagement(const trajectory_path_t *path, const float leaderPos[3],
						   unsigned char *trajectoryComplete, unsigned char *boundsExceeded, unsigned char following_started, unsigned int maneuver_time)
{
	*boundsExceeded = 0;
	if (leaderPos[0] >  BOUND_X || leaderPos[0] < -BOUND_X ||
		leaderPos[1] >  BOUND_Y || leaderPos[1] < -BOUND_Y ||
		leaderPos[2] >  BOUND_Z || leaderPos[2] < -BOUND_Z) {
		*boundsExceeded = 1;
	}

	float finalDist = sqrtf(SQUARE(leaderPos[0] - path->pos[path->numPoints - 1][0]) +
							SQUARE(leaderPos[1] - path->pos[path->numPoints - 1][1]) +
							SQUARE(leaderPos[2] - path->pos[path->numPoints - 1][2]));

	*trajectoryComplete = following_started && (maneuver_time >= 0.8f*TRAJ_TOTAL_MS) && finalDist <= TRAJ_COMPLETE_TOLERANCE;
}
