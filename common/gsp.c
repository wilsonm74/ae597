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
#include "ViewerControllerMAB.h"
#include "state_machine.h"

#define MINIMUM_PULSE_MS 10
#define INSERT_BOUND_EXCEEDANCE 0
#define OVERRIDE_BOUND_CHECK 0

state_vector trajectory_origin;

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
	leaderStateMachineInit();
	viewerStateMachineInit();
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
	state_vector ctrlState; // State of the sphere currenty being controlled
	state_vector ctrlStateTarget; // Target state for the current sphere being controlled
	state_vector ctrlStateError; // Error between the current state
	state_vector leaderCurState; // current state of the leader sphere used by the viewer controller to figure out target vector.

	viewerStates viewerCurrentState = ACQUISITION;

	float acceleration[3] = {0.0f, 0.0f, 0.0f};
	float pointingErrorDeg = 0.0f;
	int leaderPosReceived = 0;
	float ctrlControl[6];
	unsigned char trajectoryComplete = 0;
	unsigned char boundsExceeded = 0;
	prop_time firing_times;
	const int min_pulse = 10;
	int metrology_cycle = 0;
	static float pulse_demand_ms[12] = {0.0f};
	static unsigned int logged_maneuver = 0;
	static unsigned int elapsed_time = 0;

	static unsigned int next_log_time = 0;

	extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD, VEHICLE_MASS;

	//Clear all uninitialized vectors
	memset(ctrlControl,0,sizeof(float)*6);
	memset(ctrlStateTarget,0,sizeof(state_vector));
	memset(ctrlStateError,0,sizeof(state_vector));
	memset(leaderCurState,0,sizeof(state_vector));

	padsStateGet(ctrlState);

	switch(maneuver_number) {
		case 1: //Estimator initialization
			if (test_time >= 10000) {
				ctrlManeuverTerminate();
			}
			break;
		case 2:
		{
			if (logged_maneuver != maneuver_number) {
				logged_maneuver = maneuver_number;
				memcpy(trajectory_origin, ctrlState, sizeof(state_vector));
			}
			static trajectory_path_t plannedPath;   // Stage 1 output, built once
			static unsigned char pathGenerated = 0;
			static unsigned int following_start_time = 0;
			static unsigned char following_started = 0;

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
				trajectoryManagement(&plannedPath, leaderPos, &trajectoryComplete, &boundsExceeded, following_started, elapsed_time);

				if (INSERT_BOUND_EXCEEDANCE && test_time > 230000U) {
					boundsExceeded = 1;
				}
				if (OVERRIDE_BOUND_CHECK) {
					boundsExceeded = 0;
				}
				viewerModeGet(&viewerCurrentState);
				leaderStateMachineUpdate(viewerCurrentState, trajectoryComplete, boundsExceeded);

				if (leaderStateMachineGetState() != FOLLOWING) {
					ctrlStateTarget[POS_X] = trajectory_origin[POS_X];
					ctrlStateTarget[POS_Y] = trajectory_origin[POS_Y];
					ctrlStateTarget[POS_Z] = trajectory_origin[POS_Z];
				} else {
					if (!following_started) {
						following_start_time = maneuver_time;
						following_started = 1;
					}
					elapsed_time = maneuver_time - following_start_time;
					idx = elapsed_time / TRAJ_CTRL_PERIOD_MS;
					if (idx >= plannedPath.numPoints) {
						idx = plannedPath.numPoints - 1;
					}
					ctrlStateTarget[POS_X] = plannedPath.pos[idx][0];
					ctrlStateTarget[POS_Y] = plannedPath.pos[idx][1];
					ctrlStateTarget[POS_Z] = plannedPath.pos[idx][2];

					ctrlStateTarget[VEL_X] = plannedPath.vel[idx][0];
					ctrlStateTarget[VEL_Y] = plannedPath.vel[idx][1];
					ctrlStateTarget[VEL_Z] = plannedPath.vel[idx][2];

					acceleration[0] = plannedPath.accel[idx][0];
					acceleration[1] = plannedPath.accel[idx][1];
					acceleration[2] = plannedPath.accel[idx][2];
				}
				// Broadcast our position so the viewer (SPHERE2) can compute
				// its pointing error relative to us.
				ctrlStateTarget[QUAT_1] = 1.0f;

				leaderPositionBroadcast(leaderPos);

				metrology_cycle = ((maneuver_time % 1000U) < ctrlPeriodGet());
				padsGlobalPeriodSet(SYS_FOREVER);

			} else if (sysIdentityGet()==SPHERE2) {

				float viewerPos[3];
				float receivedLeaderPos[3];

				// Viewer (SPHERE2): compute pointing error toward the
				// leader, using the most recently received leader position.
				viewerPos[0] = ctrlState[POS_X];
				viewerPos[1] = ctrlState[POS_Y];
				viewerPos[2] = ctrlState[POS_Z];

				leaderPosReceived = leaderPositionGet(receivedLeaderPos);
				if (leaderPosReceived) {
					// ctrlState[QUAT_1..QUAT_4] are contiguous floats, and
					// (per the existing code's use of ctrlStateTarget[QUAT_1]
					// = 1.0f for identity attitude) QUAT_1 is the scalar
					// component - so this is scalar-first [qw,qx,qy,qz],
					// matching calculateTrackingError()'s expected order.
					pointingErrorDeg = calculateTrackingError(viewerPos, receivedLeaderPos,
															   &ctrlState[QUAT_1]);
					// pointingErrorDeg is available here for logging, or for
					// driving a "point-at-leader" attitude controller later.
					// (void)pointingErrorDeg;

					leaderCurState[POS_X] = receivedLeaderPos[0];
					leaderCurState[POS_Y] = receivedLeaderPos[1];
					leaderCurState[POS_Z] = receivedLeaderPos[2];

				}
				viewerStateMachineUpdate(pointingErrorDeg, leaderPosReceived);
				viewerCurrentState = viewerStateMachineGetState();
				viewerModeSend(viewerCurrentState);
				
				//void ViewerController(int maneuverNumber, state_vector viewCurState, state_vector leadCurState, state_vector * targetVector);
				ViewerController(1, ctrlState, leaderCurState, &ctrlStateTarget);
			}

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
				debug_values[0] = (float)test_time / 1000.0f;
				debug_values[1] = (float)pointingErrorDeg;
				debug_values[2] = (float)ctrlStateTarget[POS_X];
				debug_values[3] = (float)ctrlStateTarget[POS_Y];
				debug_values[4] = (float)ctrlStateTarget[POS_Z];
				debug_values[5] = (float)(int)trajectoryComplete;
				debug_values[6] = (float)(int)viewerStateMachineGetState();
				debug_values[7] = (float)(int)leaderStateMachineGetState();

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

			/*
			if (trajectoryComplete != 0) {
				ctrlTestTerminate(TEST_RESULT_NORMAL);
			}
			*/
			
			break;
		}
	}
}


void gspProcessRXData(default_rfm_packet packet)
{
	leaderPositionProcessPacket(packet);
	viewerModeProcessPacket(packet);
}
