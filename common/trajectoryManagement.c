/*
 * trajectoryManagement.c
 *
 * Stage 2: Trajectory Management. See trajectoryManagement.h.
 */

#include "trajectoryManagement.h"
#include <math.h>

void trajectoryManagement(const trajectory_path_t *path, const float leaderPos[3],
						   unsigned char *trajectoryComplete, unsigned char *boundsExceeded)
{
	static unsigned int targetIdx = 0;   // furthest waypoint reached so far
	unsigned int i;
	float dx, dy, dz, dist;

	// --- Output 2: does the planned path stay within the workspace bounds? ---
	*boundsExceeded = 0;
	for (i = 0; i < path->numPoints; i++) {
		if (path->pos[i][0] >  BOUND_X || path->pos[i][0] < -BOUND_X ||
			path->pos[i][1] >  BOUND_Y || path->pos[i][1] < -BOUND_Y ||
			path->pos[i][2] >  BOUND_Z || path->pos[i][2] < -BOUND_Z) {
			*boundsExceeded = 1;
			break;
		}
	}

	// --- Output 1: has the leader progressed through the whole path? ---
	dx = leaderPos[0] - path->pos[targetIdx][0];
	dy = leaderPos[1] - path->pos[targetIdx][1];
	dz = leaderPos[2] - path->pos[targetIdx][2];
	dist = sqrtf(dx*dx + dy*dy + dz*dz);

	// Once close enough to the current waypoint, advance to the next one
	if (dist <= TRAJ_COMPLETE_TOLERANCE && targetIdx < path->numPoints - 1) {
		targetIdx++;
	}

	*trajectoryComplete = (targetIdx >= path->numPoints - 1 &&
							dist <= TRAJ_COMPLETE_TOLERANCE) ? 1 : 0;
}
