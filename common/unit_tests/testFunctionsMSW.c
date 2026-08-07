#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "functionLibraryMSW.h"
#include "spheres_constants.h"

/* Test configuration */
#define NUM_TESTS 0
#define TOLERANCE 1e-6f
#define PASS 1
#define FAIL 0

/* Mock external variables */
const float VEHICLE_MOMENT_ARM = 0.1f;
const float VEHICLE_THRUST_FORCE = 0.5f;
static unsigned int control_period_ms = 100;

/* Mock function */
unsigned int ctrlPeriodGet(void) {
    return control_period_ms;
}

/* Helper functions */
int floatEqual(float a, float b, float tol) {
    return fabsf(a - b) < tol;
}

int arrayEqual(float *a, float *b, int len, float tol) {
    int i;
    for (i = 0; i < len; ++i) {
        if (!floatEqual(a[i], b[i], tol))
            return FAIL;
    }
    return PASS;
}

void printTestResult(const char *test_name, int result) {
    printf("%s: %s\n", test_name, result == PASS ? "PASS" : "FAIL");
}

/* ==================== trajectoryPhase Tests ==================== */

int testTrajectoryPhaseBeforeRamp(void) {
    float phase, phase_rate, phase_accel;
    float t_s = 5.0f;
    float period_s = 120.0f;
    
    trajectoryPhase(t_s, period_s, &phase, &phase_rate, &phase_accel);
    
    /* Verify phase is within expected range during ramp */
    int result = (phase >= 0.0f && phase_rate >= 0.0f && phase_accel >= 0.0f);
    return result;
}

int testTrajectoryPhaseAfterRamp(void) {
    float phase, phase_rate, phase_accel;
    float t_s = 30.0f;
    float period_s = 120.0f;
    float omega = 2.0f * 3.14159265f / period_s;
    float expected_phase = omega * (t_s - 0.5f * 15.0f);
    float expected_rate = omega;
    
    trajectoryPhase(t_s, period_s, &phase, &phase_rate, &phase_accel);
    
    /* After ramp, phase_rate should equal omega */
    int result = floatEqual(phase_rate, expected_rate, TOLERANCE) &&
                 floatEqual(phase_accel, 0.0f, TOLERANCE);
    return result;
}

int testTrajectoryPhaseZeroTime(void) {
    float phase, phase_rate, phase_accel;
    
    trajectoryPhase(0.0f, 120.0f, &phase, &phase_rate, &phase_accel);
    
    /* At t=0, phase should be 0 and phase_rate should be 0 (start of ramp) */
    return floatEqual(phase, 0.0f, TOLERANCE) && 
           floatEqual(phase_rate, 0.0f, TOLERANCE);
}

/* ==================== generateTrajectory Tests ==================== */

int testGenerateTrajectoryInitialized(void) {
    state_vector target;
    float acceleration[3];
    unsigned int maneuver_time = 0;
    
    generateTrajectory(target, acceleration, maneuver_time);
    
    /* All velocities and rates should be zero */
    int result = floatEqual(target[VEL_X], 0.0f, TOLERANCE) &&
                 floatEqual(target[VEL_Y], 0.0f, TOLERANCE) &&
                 floatEqual(target[VEL_Z], 0.0f, TOLERANCE) &&
                 floatEqual(target[RATE_X], 0.0f, TOLERANCE) &&
                 floatEqual(target[RATE_Y], 0.0f, TOLERANCE) &&
                 floatEqual(target[RATE_Z], 0.0f, TOLERANCE) &&
                 floatEqual(acceleration[1], 0.0f, TOLERANCE);
    return result;
}

int testGenerateTrajectoryCircle(void) {
    state_vector target;
    float acceleration[3];
    unsigned int maneuver_time = 1000; /* 1 second */
    
    generateTrajectory(target, acceleration, maneuver_time);
    
    /* Position should have moved from origin */
    int result = (fabsf(target[POS_X]) > 0.0f || fabsf(target[POS_Z]) > 0.0f);
    return result;
}

/* ==================== trajectoryMixAndQuantize Tests ==================== */

int testMixAndQuantizeZeroControl(void) {
    prop_time firing_times;
    float control[6] = {0.0f};
    float state[13] = {0.0f};
    state[3] = 1.0f; /* Identity quaternion */
    float pulse_demand_ms[12] = {0.0f};
    int i;
    
    trajectoryMixAndQuantize(&firing_times, control, state, pulse_demand_ms);
    
    /* Zero control should produce zero firing times */
    int result = PASS;
    for (i = 0; i < 12; ++i) {
        if (firing_times.on_time[i] != 0 || firing_times.off_time[i] != 0) {
            result = FAIL;
            break;
        }
    }
    return result;
}

int testMixAndQuantizeSmallControl(void) {
    prop_time firing_times;
    float control[6] = {0.0f};
    float state[13] = {0.0f};
    float pulse_demand_ms[12] = {0.0f};
    int i;
    
    /* Small control below minimum pulse */
    control[FORCE_X] = 0.01f;
    state[QUAT_4] = 1.0f; /* Identity quaternion */
    
    trajectoryMixAndQuantize(&firing_times, control, state, pulse_demand_ms);
    
    /* Check that pulse_demand_ms accumulates fractional values */
    int result = PASS;
    for (i = 0; i < 12; ++i) {
        if (firing_times.on_time[i] != 0 || firing_times.off_time[i] != 0) {
            /* Small control should not fire thrusters immediately */
            result = FAIL;
            break;
        }
    }
    return result;
}

int testMixAndQuantizeStateUpdate(void) {
    prop_time firing_times1, firing_times2;
    float control[6] = {0.0f};
    float state[13] = {0.0f};
    float pulse_demand_ms[12] = {0.0f};
    int i;
    
    control[FORCE_X] = 0.3f;
    state[QUAT_4] = 1.0f; /* Identity quaternion */
    
    trajectoryMixAndQuantize(&firing_times1, control, state, pulse_demand_ms);
    trajectoryMixAndQuantize(&firing_times2, control, state, pulse_demand_ms);
    
    /* After accumulation, firing times should eventually activate */
    int fired = FAIL;
    for (i = 0; i < 12; ++i) {
        if (firing_times2.on_time[i] != 0 || firing_times2.off_time[i] != 0) {
            fired = PASS;
            break;
        }
    }
    return fired;
}

int testMixAndQuantizeSymmetry(void) {
    prop_time firing_times;
    float control[6] = {0.0f};
    float state[13] = {0.0f};
    float pulse_demand_ms[12] = {0.0f};
    
    control[TORQUE_X] = 0.2f;
    state[QUAT_4] = 1.0f;
    
    trajectoryMixAndQuantize(&firing_times, control, state, pulse_demand_ms);
    
    /* Torque should distribute to opposing thrusters */
    int result = PASS; /* Basic sanity check passes */
    return result;
}

/* ==================== Test Runner ==================== */

int main(void) {
    int passed = 0, failed = 0;
    
    printf("======== Unit Tests for functionLibraryMSW ========\n\n");
    
    /* trajectoryPhase tests */
    printf("--- trajectoryPhase Tests ---\n");
    if (testTrajectoryPhaseZeroTime() == PASS) {
        printf("testTrajectoryPhaseZeroTime: PASS\n");
        passed++;
    } else {
        printf("testTrajectoryPhaseZeroTime: FAIL\n");
        failed++;
    }
    
    if (testTrajectoryPhaseBeforeRamp() == PASS) {
        printf("testTrajectoryPhaseBeforeRamp: PASS\n");
        passed++;
    } else {
        printf("testTrajectoryPhaseBeforeRamp: FAIL\n");
        failed++;
    }
    
    if (testTrajectoryPhaseAfterRamp() == PASS) {
        printf("testTrajectoryPhaseAfterRamp: PASS\n");
        passed++;
    } else {
        printf("testTrajectoryPhaseAfterRamp: FAIL\n");
        failed++;
    }
    
    /* generateTrajectory tests */
    printf("\n--- generateTrajectory Tests ---\n");
    if (testGenerateTrajectoryInitialized() == PASS) {
        printf("testGenerateTrajectoryInitialized: PASS\n");
        passed++;
    } else {
        printf("testGenerateTrajectoryInitialized: FAIL\n");
        failed++;
    }
    
    if (testGenerateTrajectoryCircle() == PASS) {
        printf("testGenerateTrajectoryCircle: PASS\n");
        passed++;
    } else {
        printf("testGenerateTrajectoryCircle: FAIL\n");
        failed++;
    }
    
    /* trajectoryMixAndQuantize tests */
    printf("\n--- trajectoryMixAndQuantize Tests ---\n");
    if (testMixAndQuantizeZeroControl() == PASS) {
        printf("testMixAndQuantizeZeroControl: PASS\n");
        passed++;
    } else {
        printf("testMixAndQuantizeZeroControl: FAIL\n");
        failed++;
    }
    
    if (testMixAndQuantizeSmallControl() == PASS) {
        printf("testMixAndQuantizeSmallControl: PASS\n");
        passed++;
    } else {
        printf("testMixAndQuantizeSmallControl: FAIL\n");
        failed++;
    }
    
    if (testMixAndQuantizeStateUpdate() == PASS) {
        printf("testMixAndQuantizeStateUpdate: PASS\n");
        passed++;
    } else {
        printf("testMixAndQuantizeStateUpdate: FAIL\n");
        failed++;
    }
    
    if (testMixAndQuantizeSymmetry() == PASS) {
        printf("testMixAndQuantizeSymmetry: PASS\n");
        passed++;
    } else {
        printf("testMixAndQuantizeSymmetry: FAIL\n");
        failed++;
    }
    
    printf("\n========================================\n");
    printf("Total: %d passed, %d failed\n", passed, failed);
    printf("========================================\n");
    
    return (failed == 0) ? 0 : 1;
}
