#include "functionLibraryMSW.h"
#include "control.h"
#include "math_matrix.h"
#include "spheres_constants.h"
#include <math.h>

#define MIXER_DUTY_CYCLE_PERCENT 20.0f
#define MINIMUM_PULSE_MS 10

void trajectoryMixAndQuantize(prop_time *firing_times, float control[6],
    float state[13], float pulse_demand_ms[12])
{    
    extern const float VEHICLE_MOMENT_ARM, VEHICLE_THRUST_FORCE;
    float body_to_global[3][3];
	float body_control[6] = {0.0f};
	float pair_control[6];
	float desired_pulse_ms[12] = {0.0f};
	float max_pulse_ms = 0.01f * MIXER_DUTY_CYCLE_PERCENT *
		(float)ctrlPeriodGet();
	float available_pair_force = 0.01f * MIXER_DUTY_CYCLE_PERCENT *
		VEHICLE_THRUST_FORCE;
	float pulse_per_newton = max_pulse_ms / available_pair_force;
	float largest_pulse = 0.0f;
	float scale = 1.0f;
	int pair;
	int thruster;

	/* ctrlControl force is inertial; torque is already in the body frame.  The
	 * transpose-like indexing below performs global-to-body conversion. */
	mathBody2Global(body_to_global, state);
	body_control[0] = body_to_global[0][0]*control[FORCE_X] +
		body_to_global[1][0]*control[FORCE_Y] +
		body_to_global[2][0]*control[FORCE_Z];
	body_control[1] = body_to_global[0][1]*control[FORCE_X] +
		body_to_global[1][1]*control[FORCE_Y] +
		body_to_global[2][1]*control[FORCE_Z];
	body_control[2] = body_to_global[0][2]*control[FORCE_X] +
		body_to_global[1][2]*control[FORCE_Y] +
		body_to_global[2][2]*control[FORCE_Z];
	body_control[3] = control[TORQUE_X] / VEHICLE_MOMENT_ARM;
	body_control[4] = control[TORQUE_Y] / VEHICLE_MOMENT_ARM;
	body_control[5] = control[TORQUE_Z] / VEHICLE_MOMENT_ARM;

	/* Invert the SPHERES thruster-pair geometry.  Each pair contributes one body
	 * force component and one moment-arm-normalized torque component. */
	pair_control[0] = 0.5f*(body_control[0] + body_control[4]);
	pair_control[1] = 0.5f*(body_control[0] - body_control[4]);
	pair_control[2] = 0.5f*(body_control[1] + body_control[5]);
	pair_control[3] = 0.5f*(body_control[1] - body_control[5]);
	pair_control[4] = 0.5f*(body_control[2] + body_control[3]);
	pair_control[5] = 0.5f*(body_control[2] - body_control[3]);

	/* Sign selects one thruster from each opposing pair. */
	for (pair = 0; pair < 6; ++pair) {
		thruster = pair_control[pair] >= 0.0f ? pair : pair + 6;
		desired_pulse_ms[thruster] = pulse_per_newton * fabsf(pair_control[pair]);
		if (desired_pulse_ms[thruster] > largest_pulse)
			largest_pulse = desired_pulse_ms[thruster];
	}
	/* Uniform scaling preserves the requested wrench direction under saturation. */
	if (largest_pulse > max_pulse_ms)
		scale = max_pulse_ms / largest_pulse;

	/* Emit only realizable pulses.  Centering each pulse within the available
	 * firing window reduces timing bias relative to the controller sample. */
	for (thruster = 0; thruster < 12; ++thruster) {
		int pulse_ms = 0;
		pulse_demand_ms[thruster] += scale * desired_pulse_ms[thruster];
		if (pulse_demand_ms[thruster] >= (float)MINIMUM_PULSE_MS) {
			pulse_ms = (int)pulse_demand_ms[thruster];
			if (pulse_ms > (int)max_pulse_ms)
				pulse_ms = (int)max_pulse_ms;
			pulse_demand_ms[thruster] -= (float)pulse_ms;
		}
		if (pulse_ms > 0) {
			firing_times->on_time[thruster] = ((int)max_pulse_ms - pulse_ms)/2;
			firing_times->off_time[thruster] =
				firing_times->on_time[thruster] + pulse_ms;
		} else {
			firing_times->on_time[thruster] = 0;
			firing_times->off_time[thruster] = 0;
		}
	}
}

void generateTrajectory(state_vector target, float acceleration[3], unsigned int maneuver_time)
{
    float t_s = 0.001f * (float) maneuver_time;

    memcpy(target, trajectory_origin, sizeof(state_vector));
}

