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

#include "functionLibraryMSW.h"

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
	float ctrlControl[6];
	prop_time firing_times;
	const int min_pulse = 10;
    int metrology_cycle = 0;

	extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD,
        VEHICLE_MASS;

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
			if (sysIdentityGet()==SPHERE1) {
                metrology_cycle = ((maneuver_time % 1000U) < ctrlPeriodGet());
				padsGlobalPeriodSet(SYS_FOREVER);				
				ctrlStateTarget[POS_X] = 0.3f;
			} else {
				ctrlStateTarget[POS_X] = -0.5f;
			}					
			ctrlStateTarget[QUAT_1] = 1.0f;
			//find error
			findStateError(ctrlStateError,ctrlState,ctrlStateTarget);
			//call controllers

            //feedforward component
            ctrlControl[FORCE_X] += VEHICLE_MASS * trajectory_acceleration[0];
            ctrlControl[FORCE_Y] += VEHICLE_MASS * trajectory_acceleration[1];
            ctrlControl[FORCE_Z] += VEHICLE_MASS * trajectory_acceleration[2];
			ctrlPositionPDgains(KPpositionPD, KDpositionPD, 
								KPpositionPD, KDpositionPD, 
								KPpositionPD, KDpositionPD, ctrlStateError, ctrlControl);
			ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,
								KPattitudePD,KDattitudePD,
								KPattitudePD,KDattitudePD,
								ctrlStateError,ctrlControl);

			//mix forces/torques into thruster commands
			ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 20.0f, FORCE_FRAME_INERTIAL);
			
            if (metrology_cycle) {
                memset(&firing_times, 0, sizeof(prop_time));
            }

			//Set firing times
			propSetThrusterTimes(&firing_times);

			if (metrology_cycle && sysIdentityGet() == SPHERE1) {
				padsGlobalPeriodSetAndWait(1000,0);
			}
			if (maneuver_time>=60000) {
				ctrlTestTerminate(TEST_RESULT_NORMAL);
			}
			break;
	}
}


void gspProcessRXData(default_rfm_packet packet)
{
}
