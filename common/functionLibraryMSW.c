#include "functionLibraryMSW.h"
#include "control.h"
#include "math_matrix.h"
#include "spheres_constants.h"
#include <math.h>
#include <string.h>

#define MIXER_DUTY_CYCLE_PERCENT 20.0f
#define MINIMUM_PULSE_MS 10
#define CIRCLE_PERIOD_S 120.0f
#define CIRCLE_RADIUS_M 0.18f
#define TRAJECTORY_RAMP_S 15.0f
#define TWO_PI_F 6.283185307179586476925286766559f

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
	float phase, phase_rate, phase_accel;

	memcpy(target, trajectory_origin, sizeof(state_vector));

	acceleration[0] = 0.0f;
	acceleration[1] = 0.0f;
	acceleration[2] = 0.0f;
	target[VEL_X] = 0.0f;
	target[VEL_Y] = 0.0f;
	target[VEL_Z] = 0.0f;
	target[RATE_X] = 0.0f;
	target[RATE_Y] = 0.0f;
	target[RATE_Z] = 0.0f;

	trajectoryPhase(t_s, CIRCLE_PERIOD_S, &phase, &phase_rate, &phase_accel);
	target[POS_X] = trajectory_origin[POS_X] + CIRCLE_RADIUS_M * (cosf(phase) - 1.0f);
	target[POS_Z] = trajectory_origin[POS_Z] - CIRCLE_RADIUS_M * sinf(phase);
	target[VEL_X] = -CIRCLE_RADIUS_M * phase_rate * sinf(phase);
	target[VEL_Z] = -CIRCLE_RADIUS_M * phase_rate * cosf(phase);
	acceleration[0] = -CIRCLE_RADIUS_M * (phase_accel * sinf(phase) + phase_rate * phase_rate * cosf(phase));
	acceleration[2] = CIRCLE_RADIUS_M * (-phase_accel * cosf(phase) + phase_rate * phase_rate * sinf(phase));

}

void trajectoryPhase(float t_s, float period_s, float *phase, float *phase_rate, float *phase_accel)
{
	float omega = TWO_PI_F / period_s;
	if (t_s < TRAJECTORY_RAMP_S) {
		float tau = t_s / TRAJECTORY_RAMP_S;
		float tau2 = tau * tau;
		float tau3 = tau2 * tau;
		float tau4 = tau3 * tau;
		float tau5 = tau4 * tau;
		float tau6 = tau5 * tau;
		float speed_scale = 10.0f*tau3 - 15.0f*tau4 + 6.0f*tau5;
		float speed_scale_rate = (30.0f*tau2 - 60.0f*tau3 + 30.0f*tau4) / TRAJECTORY_RAMP_S;
		*phase = omega * TRAJECTORY_RAMP_S * (2.5f*tau4 - 3.0f*tau5 + tau6);
		*phase_rate = omega * speed_scale;
		*phase_accel = omega * speed_scale_rate;
	}
	else {
		*phase = omega * (t_s - 0.5f * TRAJECTORY_RAMP_S);
		*phase_rate = omega;
		*phase_accel = 0.0f;
	}
}


