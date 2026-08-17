/*
 * trajectoryPlanner.h
 *
 * Stage 1: Planned Trajectory.
 *
 * Generates the planned circular trajectory (X-Z plane) that the leader
 * (SPHERE1) is commanded to fly. Produces a trajectory_path_t: a set of
 * waypoints that is handed off to the Trajectory Management stage
 * (trajectoryManagement.h/.c).
 */

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

/*----------------------------------------------------------------------------*/
/* Circular trajectory parameters for SPHERE1                                 */
/*                                                                            */
/* SPHERE1 flies a circle in the X-Z plane (Y is held at 0, well inside the   */
/* +/-0.2m bound). The circle has radius 0.8m along X and Z, matching the     */
/* +/-0.8m bounds on those axes.                                             */
/*----------------------------------------------------------------------------*/
#define CIRCLE_RADIUS_X   0.6f      // [m] radius of travel along X
#define CIRCLE_RADIUS_Z   0.6f      // [m] radius of travel along Z
#define CIRCLE_CENTER_Y   0.0f      // [m] Y held here (within +/-0.2m bound)
#define CIRCLE_PERIOD_S   600.0f    // [s] time to complete one full revolution
#define CIRCLE_PERIOD_MS  600000u    // [ms] time to complete one full revolution
#define CIRCLE_NUM_REVS   1u        // number of revolutions to fly before ending test
#define CIRCLE_TWO_PI     6.283185307f

/* One planned-trajectory waypoint is generated per control cycle */
#define TRAJ_CTRL_PERIOD_MS 100u
#define TRAJ_TOTAL_MS        (CIRCLE_PERIOD_MS * CIRCLE_NUM_REVS)
#define TRAJ_NUM_POINTS      ((TRAJ_TOTAL_MS / TRAJ_CTRL_PERIOD_MS) + 1u)

/* A trajectory_path_t holds the full set of waypoints (one per control
   cycle) for the maneuver. This is the object handed off to the Trajectory
   Management stage. */
typedef struct {
	float pos[TRAJ_NUM_POINTS][3];   // planned [X,Y,Z] waypoints, in order
	float accel[TRAJ_NUM_POINTS][3]; // planned [X,Y,Z] accelerations, in order
	float vel[TRAJ_NUM_POINTS][3];   // planned [X,Y,Z] velocities, in order
	unsigned int numPoints;          // number of valid waypoints in pos[]
} trajectory_path_t;

// Fills in a circular trajectory path (X-Z plane circle, as configured above).
void plannedTrajectoryGenerate(trajectory_path_t *path);

#endif /* TRAJECTORY_PLANNER_H */
