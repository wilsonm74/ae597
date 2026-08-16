/*
Copyright (c) 2014, Massachusetts Institute of Technology Space Systems Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Massachusetts Institute of Technology, the MIT Space Systems Laboratory, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* 
 * gsp.c
 *
 * SPHERES Guest Scientist Program custom source code template.
 *
 * MIT Space Systems Laboratory
 * SPHERES Guest Scientist Program
 * http://ssl.mit.edu/spheres/
 * 
 * Copyright 2005 Massachusetts Institute of Technology
 */


/*----------------------------------------------------------------------------*/
/*                         Do not modify this section.                        */
/*----------------------------------------------------------------------------*/

#include "comm.h"
#include "commands.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "pads.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"

/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "ctrl_mix.h"
#include <string.h>
#include "trajectoryPlanner.h"
#include "trajectoryManagement.h"
#include "leaderPositionComm.h"
#include "trackingError.h"
#include "functionLibraryMSW.h"

#define MINIMUM_PULSE_MS 10

void gspIdentitySet()
{
   // set the logical identifier (SPHERE#) for this vehicle
   sysIdentitySet(SPHERE_ID);
}


void gspInitProgram()
{
   // set the unique program identifier (to be assigned by MIT)
   sysProgramIDSet(2110);

   // set up communications TDMA frames
   commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), NUM_SPHERES);
   commTdmaStandardInit(COMM_CHANNEL_STS, sysIdentityGet(), NUM_SPHERES);

   // enable communications channels
   commTdmaEnable(COMM_CHANNEL_STL);
   commTdmaEnable(COMM_CHANNEL_STS);   
   
   // allocate storage space for IMU samples
   padsInertialAllocateBuffers(50);

   // inform system of highest beacon number in use
   padsInitializeFPGA(NUM_BEACONS);

   /* custom program initialization goes below this point */
}


void gspInitTest(unsigned int test_number)
{
	extern state_vector initState;
    memcpy(trajectory_origin, initState, sizeof(state_vector));
	if (sysIdentityGet() == SPHERE1){
		padsEstimatorInitWaitAndSet(initState, 50, 200, 105, PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
	} else {
		padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER, PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
	}
	ctrlPeriodSet(100);
}


void gspInitTask()
{
}


void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro, unsigned int num_samples)
{
}


void gspPadsGlobal(unsigned int beacon, beacon_measurement_matrix measurements)
{
}


void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data)
{
}


void gspControl(unsigned int test_number, unsigned int test_time, unsigned int maneuver_number, unsigned int maneuver_time)
{
	state_vector ctrlState;
	state_vector ctrlStateTarget;
	state_vector ctrlStateError;
	float acceleration[3] = {0.0f, 0.0f, 0.0f};
	float ctrlControl[6];
	prop_time firing_times;
	const int min_pulse = 10;
    int metrology_cycle = 0;
    static float pulse_demand_ms[12] = {0.0f};

	static unsigned int next_log_time = 0;

	extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD, VEHICLE_MASS;

	//Clear all uninitialized vectors
	memset(ctrlControl,0,sizeof(float)*6);
	memset(ctrlStateTarget,0,sizeof(state_vector));
	memset(ctrlStateError,0,sizeof(state_vector));

	padsStateGet(ctrlState);

	switch(maneuver_number) {
		case 1: //Estimator initialization
			if (test_time >= 10000) {
				ctrlManeuverTerminate();
			}
			break;
		case 2:
		{
			static trajectory_path_t plannedPath;   // Stage 1 output, built once
			static unsigned char pathGenerated = 0;
			unsigned char trajectoryComplete = 0;
			unsigned char boundsExceeded = 0;

			if (sysIdentityGet()==SPHERE1) {
				float leaderPos[3];
				unsigned int idx;

				metrology_cycle = (maneuver_time % 1000U) < ctrlPeriodGet();
				padsGlobalPeriodSet(SYS_FOREVER);

				// Stage 1: generate the planned trajectory once, the first
				// time we enter this maneuver.
				if (!pathGenerated) {
					plannedTrajectoryGenerate(&plannedPath);
					pathGenerated = 1;
				}

				// Current leader (SPHERE1) position, from the state estimator
				leaderPos[0] = ctrlState[POS_X];
				leaderPos[1] = ctrlState[POS_Y];
				leaderPos[2] = ctrlState[POS_Z];

				// Stage 2: trajectory management
				trajectoryManagement(&plannedPath, leaderPos, &trajectoryComplete, &boundsExceeded);

				if (boundsExceeded) {
					// Planned path violates the workspace bounds: hold the
					// current position rather than command further motion.
					ctrlStateTarget[POS_X] = leaderPos[0];
					ctrlStateTarget[POS_Y] = leaderPos[1];
					ctrlStateTarget[POS_Z] = leaderPos[2];
				} else {
					idx = maneuver_time / TRAJ_CTRL_PERIOD_MS;
					if (idx >= plannedPath.numPoints) {
						idx = plannedPath.numPoints - 1;
					}
					ctrlStateTarget[POS_X] = plannedPath.pos[idx][0];
					ctrlStateTarget[POS_Y] = plannedPath.pos[idx][1];
					ctrlStateTarget[POS_Z] = plannedPath.pos[idx][2];

					#if (0)
						if (idx > 0 && idx < plannedPath.numPoints - 1) {
							float dt = (float)TRAJ_CTRL_PERIOD_MS / 1000.0f;
							for (int jdx = 0; jdx < 3; jdx++) {
								acceleration[jdx] = (plannedPath.pos[idx+1][jdx] - 2*plannedPath.pos[idx][jdx] + plannedPath.pos[idx-1][jdx]) / (dt * dt);
								}
						}
						else {
							acceleration[0] = 0.0f;
							acceleration[1] = 0.0f;
							acceleration[2] = 0.0f;
						}
					#else
						acceleration[0] = plannedPath.accel[idx][0];
						acceleration[1] = plannedPath.accel[idx][1];
						acceleration[2] = plannedPath.accel[idx][2];
					#endif
				}
				// Broadcast our position so the viewer (SPHERE2) can compute
				// its pointing error relative to us.
				leaderPositionBroadcast(leaderPos);

                metrology_cycle = ((maneuver_time % 1000U) < ctrlPeriodGet());
				padsGlobalPeriodSet(SYS_FOREVER);				

			} else {
				float viewerPos[3];
				float receivedLeaderPos[3];
				float pointingErrorDeg;

				ctrlStateTarget[POS_X] = -0.5f;

				// Viewer (SPHERE2): compute pointing error toward the
				// leader, using the most recently received leader position.
				viewerPos[0] = ctrlState[POS_X];
				viewerPos[1] = ctrlState[POS_Y];
				viewerPos[2] = ctrlState[POS_Z];

				if (leaderPositionGet(receivedLeaderPos)) {
					// ctrlState[QUAT_1..QUAT_4] are contiguous floats, and
					// (per the existing code's use of ctrlStateTarget[QUAT_1]
					// = 1.0f for identity attitude) QUAT_1 is the scalar
					// component - so this is scalar-first [qw,qx,qy,qz],
					// matching calculateTrackingError()'s expected order.
					pointingErrorDeg = calculateTrackingError(viewerPos, receivedLeaderPos,
															   &ctrlState[QUAT_1]);
					// pointingErrorDeg is available here for logging, or for
					// driving a "point-at-leader" attitude controller later.
					(void)pointingErrorDeg;
				}
			}					
			ctrlStateTarget[QUAT_1] = 1.0f;
			//find error
			findStateError(ctrlStateError,ctrlState,ctrlStateTarget);
			//call controllers
			// feedforward acceleration compoent
			ctrlControl[FORCE_X] += acceleration[0] * VEHICLE_MASS;
			ctrlControl[FORCE_Y] += acceleration[1] * VEHICLE_MASS;
			ctrlControl[FORCE_Z] += acceleration[2] * VEHICLE_MASS;

			ctrlPositionPDgains(KPpositionPD, KDpositionPD, 
								KPpositionPD, KDpositionPD, 
								KPpositionPD, KDpositionPD, ctrlStateError, ctrlControl);
			ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,
								KPattitudePD,KDattitudePD,
								KPattitudePD,KDattitudePD,
								ctrlStateError,ctrlControl);

			//mix forces/torques into thruster commands
			// ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 20.0f, FORCE_FRAME_INERTIAL);
            trajectoryMixAndQuantize(&firing_times, ctrlControl, ctrlState, pulse_demand_ms);
			
            if (metrology_cycle) {
                memset(&firing_times, 0, sizeof(prop_time));
            }

			//Set firing times
			propSetThrusterTimes(&firing_times);

			if (test_time >= next_log_time) {
				float debug_values[8] = {0};
				debug_values[0] = (float)maneuver_time / 1000.0f;
				debug_values[1] = (float)boundsExceeded;
				debug_values[2] = (float)ctrlStateTarget[POS_X];
				debug_values[3] = (float)ctrlStateTarget[POS_Y];
				debug_values[4] = (float)ctrlStateTarget[POS_Z];
				debug_values[5] = (float)ctrlControl[FORCE_X];
				debug_values[6] = (float)ctrlControl[FORCE_Y];
				debug_values[7] = (float)ctrlControl[FORCE_Z];

				commSendPacket(
					COMM_CHANNEL_STL,
					GROUND,
					sysIdentityGet(),
					COMM_CMD_DBG_FLOAT,
					(unsigned char *)debug_values,
					0
				);

				next_log_time = test_time + 1000U;
			}

			if (metrology_cycle && sysIdentityGet() == SPHERE1) {
				padsGlobalPeriodSetAndWait(1000,0);
			}

			// End the test once the leader's trajectory management stage
			// reports completion (or a bounds violation). Followers, which
			// don't run trajectory management themselves, fall back to the
			// planned total duration.
			if (sysIdentityGet() == SPHERE1) {
				if (trajectoryComplete || boundsExceeded) {
					ctrlTestTerminate(TEST_RESULT_NORMAL);
				}
			} else {
				if (maneuver_time >= TRAJ_TOTAL_MS) {
					ctrlTestTerminate(TEST_RESULT_NORMAL);
				}
			}
			break;
		}
	}
}


void gspProcessRXData(default_rfm_packet packet)
{
	leaderPositionProcessPacket(packet);
}
