/*
 * trackingError.h
 *
 * Computes the pointing error between a viewer vehicle's boresight and a
 * leader vehicle's position, per the tracking-error diagram:
 *
 *      theta = acos( x_b_hat . l_hat )
 *
 *      x_b_hat = R(q) * [1, 0, 0]^T   (viewer's body +X axis, i.e. its
 *                                       boresight, rotated into the
 *                                       inertial frame by its attitude)
 *      l_hat   = (p_leader - p_viewer) / || p_leader - p_viewer ||
 */

#ifndef TRACKING_ERROR_H
#define TRACKING_ERROR_H
#include "spheres_types.h"
#define VIEWER_INIT_X_M 0.0
#define VIEWER_INIT_Y_M -0.3
#define VIEWER_INIT_Z_M 0.0
/*
 * Inputs:
 *   viewerPos  - [X,Y,Z] position of the viewer (controller vehicle), meters
 *   leaderPos  - [X,Y,Z] current position of the leader, meters
 *   viewerQuat - viewer's attitude quaternion, in the SAME layout as the
 *                SPHERES state_vector: [QUAT_1, QUAT_2, QUAT_3, QUAT_4] =
 *                [qx, qy, qz, qw] (vector part first, scalar part last, per
 *                the official GSP state vector table). You can pass
 *                &ctrlState[QUAT_1] directly, since those 4 elements are
 *                contiguous in state_vector.
 *
 * Returns:
 *   Pointing error in degrees, in the range [0, 180].
 */
float calculateTrackingError(const float viewerPos[3], const float leaderPos[3],
							  const float viewerQuat[4]);


void GetTargetVector(state_vector viewerState, state_vector leaderState, state_vector *targetVector);

#endif /* TRACKING_ERROR_H */
