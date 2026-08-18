/*
 * trajectoryManagement.h
 *
 * Stage 2: Trajectory Management.
 *
 * Takes the planned trajectory produced by trajectoryPlanner.h/.c (Stage 1)
 * and the leader's current position, and reports:
 *   1) whether the trajectory has been completed
 *   2) whether the planned trajectory path exceeds the workspace bounds
 */

#ifndef TRAJECTORY_MANAGEMENT_H
#define TRAJECTORY_MANAGEMENT_H

#include "trajectoryPlanner.h"

/* Workspace limits (given test-volume bounds) */
#define BOUND_X 0.8f   // [m] +/- limit on X
#define BOUND_Y 1.2f   // [m] +/- limit on Y
#define BOUND_Z 0.8f   // [m] +/- limit on Z

#define TRAJ_COMPLETE_TOLERANCE 0.05f

/*
 * Inputs:
 *   path      - the planned trajectory from Stage 1
 *   leaderPos - current [X,Y,Z] position of the leader (SPHERE1)
 * Outputs:
 *   *trajectoryComplete - 1 once the leader has advanced through and
 *                         arrived at the final waypoint, 0 otherwise
 *   *boundsExceeded     - 1 if ANY waypoint in the planned path falls
 *                         outside the +/-BOUND_X/Y/Z workspace, 0 otherwise
 *
 * Note: because this trajectory is a closed loop (it returns to its start
 * point after each revolution), simply checking "is the leader near the
 * last waypoint?" would also be true near t=0. Instead this function tracks
 * a monotonically-advancing "furthest waypoint reached" index, so
 * completion can only be signaled after the leader has actually progressed
 * through the whole path and arrived back at the final waypoint.
 */
void trajectoryManagement(const trajectory_path_t *path, const float leaderPos[3],
						   unsigned char *trajectoryComplete, unsigned char *boundsExceeded,
						   unsigned char following_started, unsigned int maneuver_time);

#endif
