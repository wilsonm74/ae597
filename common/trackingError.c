/*
 * trackingError.c
 *
 * See trackingError.h.
 */

#include "trackingError.h"
#include "math_quat.h"
#include "functionLibraryMSW.h"
#include <math.h>
#include "spheres_constants.h"
#define RAD_TO_DEG 57.29577951f

float calculateTrackingError(const float viewerPos[3], const float leaderPos[3],
							  const float viewerQuat[4])
{
	float qx, qy, qz, qw;
	float xb[3];
	float los[3];
	float losNorm;
	float lHat[3];
	float dot;

	// Viewer attitude quaternion, matching the SPHERES state_vector layout:
	// [QUAT_1, QUAT_2, QUAT_3, QUAT_4] = [qx, qy, qz, qw] (vector part first,
	// scalar part last). Callers can pass &ctrlState[QUAT_1] directly.
	qx = viewerQuat[0];
	qy = viewerQuat[1];
	qz = viewerQuat[2];
	qw = viewerQuat[3];

	// x_b_hat = R(q) * [1,0,0]^T
	// (first column of the body->inertial rotation matrix built from q)
	xb[0] = 1.0f - 2.0f*(qy*qy + qz*qz);
	xb[1] = 2.0f*(qx*qy + qw*qz);
	xb[2] = 2.0f*(qx*qz - qw*qy);

	// l_hat = (p_leader - p_viewer) / || p_leader - p_viewer ||
	los[0] = leaderPos[0] - viewerPos[0];
	los[1] = leaderPos[1] - viewerPos[1];
	los[2] = leaderPos[2] - viewerPos[2];

	losNorm = sqrtf(los[0]*los[0] + los[1]*los[1] + los[2]*los[2]);
	if (losNorm < 1e-6f) {
		// Viewer and leader are (numerically) at the same position, so
		// the line-of-sight direction is undefined. Report zero error
		// rather than dividing by ~0.
		return 0.0f;
	}
	lHat[0] = los[0] / losNorm;
	lHat[1] = los[1] / losNorm;
	lHat[2] = los[2] / losNorm;

	// theta = acos( x_b_hat . l_hat )
	dot = xb[0]*lHat[0] + xb[1]*lHat[1] + xb[2]*lHat[2];

	// Clamp before acos: floating-point rounding can push |dot| slightly
	// past 1.0, which would otherwise make acosf() return NaN.
	if (dot >  1.0f) dot =  1.0f;
	if (dot < -1.0f) dot = -1.0f;

	return acosf(dot) * RAD_TO_DEG;
}

void GetTargetQuat(const float viewerPos[3], const float leaderPos[3],
								   float targetQuat[4])
{
	float los[3];
	float losNorm;
	float lHat[3];
	float bodyXAxis[3] = {1.0f, 0.0f, 0.0f};

	// l_hat = (p_leader - p_viewer) / || p_leader - p_viewer ||
	los[0] = leaderPos[0] - viewerPos[0];
	los[1] = leaderPos[1] - viewerPos[1];
	los[2] = leaderPos[2] - viewerPos[2];

	losNorm = sqrtf(los[0]*los[0] + los[1]*los[1] + los[2]*los[2]);

	if (losNorm < 1e-6f) {
		// Viewer and leader are at same position, return identity quaternion
		targetQuat[0] = 0.0f;
		targetQuat[1] = 0.0f;
		targetQuat[2] = 0.0f;
		targetQuat[3] = 1.0f;
		return;
	}

	// Normalize line-of-sight
	lHat[0] = los[0] / losNorm;
	lHat[1] = los[1] / losNorm;
	lHat[2] = los[2] / losNorm;

	// Create quaternion that rotates body X-axis [1,0,0] to point toward leader
	quatVec2Vec(targetQuat, bodyXAxis, lHat);
}

void GetTargetVector(state_vector viewerState, state_vector leaderState, state_vector *targetVector){
	float los[3];
	float losNorm;
	float lHat[3];
	float bodyXAxis[3] = {1.0f, 0.0f, 0.0f};
	float targetQuat[4];


	// l_hat = (p_leader - p_viewer) / || p_leader - p_viewer ||
	los[0] = leaderState[0] - viewerState[0];
	los[1] = leaderState[1] - viewerState[1];
	los[2] = leaderState[2] - viewerState[2];

	losNorm = sqrtf(los[0]*los[0] + los[1]*los[1] + los[2]*los[2]);

	if (losNorm < 1e-6f) {
		// Viewer and leader are at same position, return identity quaternion
		targetQuat[0] = 0.0f;
		targetQuat[1] = 0.0f;
		targetQuat[2] = 0.0f;
		targetQuat[3] = 1.0f;

	}
	else{
		// Normalize line-of-sight
		lHat[0] = los[0] / losNorm;
		lHat[1] = los[1] / losNorm;
		lHat[2] = los[2] / losNorm;

		quatVec2Vec(targetQuat, bodyXAxis, lHat);
	}

		(*targetVector)[POS_X] = trajectory_origin[POS_X];
		(*targetVector)[POS_Y] = trajectory_origin[POS_Y];
		(*targetVector)[POS_Z] = trajectory_origin[POS_Z];
		(*targetVector)[QUAT_1] = targetQuat[0];
		(*targetVector)[QUAT_2] = targetQuat[1];
		(*targetVector)[QUAT_3] = targetQuat[2];
		(*targetVector)[QUAT_4] = targetQuat[3];

		return;
}

