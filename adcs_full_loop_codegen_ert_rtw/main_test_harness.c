/* main_test_harness.c
 *
 * Standalone C test harness for the auto-generated adcs_full_loop_codegen
 * model code. Calls the generated model_step() function directly in a
 * loop -- no MATLAB or Simulink runtime involved at all -- to prove the
 * generated C is genuinely independent, deployable embedded code, and to
 * numerically verify it reproduces the validated Simulink result.
 *
 * Expected final values (from Simulink, fixed-step discrete model):
 *   err_deg_out = 0.491
 *   omega_out   = [~1e-15, ~1e-15, ~1e-15]  (converged to numerical zero)
 *   h_wheel_out = [0.0003046, -0.0006814, 0.0007529]
 */

#include <stdio.h>
#include "adcs_full_loop_codegen.h"

#define STEP_SIZE   0.01      /* must match the model's fixed-step size */
#define STOP_TIME   800.0     /* matches the model's Stop Time */
#define NUM_STEPS   ((int)(STOP_TIME / STEP_SIZE))
#define PRINT_EVERY 1000      /* progress print every 1000 steps = 10s */

int main(void)
{
    int step;
    double t;

    /* Same 45deg slew about [1,1,1]/sqrt(3) used in every validated run
       of this project -- held constant for the whole run */
    adcs_full_loop_codegen_U.q_target_in[0] = 0.9239;
    adcs_full_loop_codegen_U.q_target_in[1] = 0.2209;
    adcs_full_loop_codegen_U.q_target_in[2] = 0.2209;
    adcs_full_loop_codegen_U.q_target_in[3] = 0.2209;

    adcs_full_loop_codegen_initialize();

    printf("Standalone C harness -- adcs_full_loop_codegen (no MATLAB/Simulink)\n");
    printf("Running %d steps at dt=%.3fs (stop time %.1fs)\n\n", NUM_STEPS, STEP_SIZE, STOP_TIME);
    printf("%8s %12s %36s %36s\n", "t(s)", "err(deg)", "omega [x,y,z] (rad/s)", "h_wheel [x,y,z] (N*m*s)");

    for (step = 0; step < NUM_STEPS; step++) {
        /* This single call is the ENTIRE control loop for one tick:
           reads q_target_in, runs quaternion error -> PD control ->
           reaction wheel saturation -> rigid body dynamics -> kinematics
           -> normalize, and writes err_deg_out/omega_out/h_wheel_out.
           On a real embedded target this would be called from a
           hardware timer ISR firing every 10ms, not a software loop. */
        adcs_full_loop_codegen_step();

        t = (step + 1) * STEP_SIZE;

        if ((step + 1) % PRINT_EVERY == 0 || step == NUM_STEPS - 1) {
            printf("%8.2f %12.6f  [%10.3e %10.3e %10.3e]  [%10.3e %10.3e %10.3e]\n",
                   t,
                   adcs_full_loop_codegen_Y.err_deg_out,
                   adcs_full_loop_codegen_Y.omega_out[0],
                   adcs_full_loop_codegen_Y.omega_out[1],
                   adcs_full_loop_codegen_Y.omega_out[2],
                   adcs_full_loop_codegen_Y.h_wheel_out[0],
                   adcs_full_loop_codegen_Y.h_wheel_out[1],
                   adcs_full_loop_codegen_Y.h_wheel_out[2]);
        }
    }

    printf("\n--- FINAL VALUES (compare against Simulink baseline in the header comment above) ---\n");
    printf("err_deg_out = %.6f\n", adcs_full_loop_codegen_Y.err_deg_out);
    printf("omega_out   = [%.6e, %.6e, %.6e]\n",
           adcs_full_loop_codegen_Y.omega_out[0],
           adcs_full_loop_codegen_Y.omega_out[1],
           adcs_full_loop_codegen_Y.omega_out[2]);
    printf("h_wheel_out = [%.6e, %.6e, %.6e]\n",
           adcs_full_loop_codegen_Y.h_wheel_out[0],
           adcs_full_loop_codegen_Y.h_wheel_out[1],
           adcs_full_loop_codegen_Y.h_wheel_out[2]);

    adcs_full_loop_codegen_terminate();

    return 0;
}
