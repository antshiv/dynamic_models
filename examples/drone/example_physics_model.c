#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drone/physics_model.h"
#include "utilities/numerical_solvers.h"

typedef struct {
    const dm_vehicle_config_t* config;
    double rotor_omega[DM_MAX_ROTORS];
} hover_inputs_t;

static double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
}

static void set_diag(double mat[3][3], double d0, double d1, double d2) {
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            mat[i][j] = 0.0;
        }
    }
    mat[0][0] = d0;
    mat[1][1] = d1;
    mat[2][2] = d2;
}

static void configure_quad(dm_vehicle_config_t* cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->rotor_count = 4;
    cfg->mass = 1.6;      /* kg */
    cfg->gravity = 9.81;  /* m/s^2 (down, NED frame) */

    set_diag(cfg->inertia, 0.03, 0.03, 0.05);
    set_diag(cfg->inertia_inv, 1.0 / 0.03, 1.0 / 0.03, 1.0 / 0.05);

    const double arm = 0.23;  /* meters */
    const double thrust_coeff = 1.5e-5; /* N/(rad/s)^2 */
    const double torque_coeff = 2.0e-7; /* N·m/(rad/s)^2 */

    const double positions[4][3] = {
        { arm,  arm, 0.0},
        {-arm,  arm, 0.0},
        {-arm, -arm, 0.0},
        { arm, -arm, 0.0},
    };
    const double directions[4] = {1.0, -1.0, 1.0, -1.0};

    for (size_t i = 0; i < cfg->rotor_count; ++i) {
        cfg->rotors[i].position_body[0] = positions[i][0];
        cfg->rotors[i].position_body[1] = positions[i][1];
        cfg->rotors[i].position_body[2] = positions[i][2];
        cfg->rotors[i].axis_body[0] = 0.0;
        cfg->rotors[i].axis_body[1] = 0.0;
        cfg->rotors[i].axis_body[2] = -1.0; /* thrust along -Z body */
        cfg->rotors[i].direction = directions[i];
        cfg->rotors[i].thrust_coeff = thrust_coeff;
        cfg->rotors[i].torque_coeff = torque_coeff;
    }
}

static void identity_quaternion(double q[4]) {
    q[0] = 1.0;
    q[1] = q[2] = q[3] = 0.0;
}

static void quaternion_to_euler(const double q[4],
                                double* roll,
                                double* pitch,
                                double* yaw) {
    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];

    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0) {
        *pitch = copysign(M_PI / 2.0, sinp);
    } else {
        *pitch = asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

static void vehicle_derivative(const void* state_in,
                               const void* input_in,
                               void* state_dot_out,
                               void* user_ctx) {
    (void)user_ctx;
    const dm_state_t* state = (const dm_state_t*)state_in;
    const hover_inputs_t* input = (const hover_inputs_t*)input_in;

    dm_vehicle_model_t temp;
    memset(&temp, 0, sizeof(temp));
    temp.config = input->config;
    temp.state = *state;

    dm_vehicle_evaluate(&temp, input->rotor_omega, (dm_state_t*)state_dot_out);
}

int main(void) {
    dm_vehicle_config_t config;
    configure_quad(&config);

    dm_state_t state;
    memset(&state, 0, sizeof(state));
    identity_quaternion(state.quaternion);

    hover_inputs_t inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.config = &config;

    const double per_rotor_thrust = (config.mass * config.gravity) / config.rotor_count;
    const double hover_omega = sqrt(per_rotor_thrust / config.rotors[0].thrust_coeff);

    for (size_t i = 0; i < config.rotor_count; ++i) {
        inputs.rotor_omega[i] = hover_omega;
    }

    const double dt = 0.002;          /* seconds */
    const double sim_time = 2.0;      /* seconds */
    const size_t steps = (size_t)(sim_time / dt);
    const size_t print_stride = 50;   /* print every 0.1 s */

    dm_state_t k1, k2, k3, k4, temp;

    printf("Quadrotor hover demonstration (dt = %.3f s, duration = %.1f s)\n",
           dt, sim_time);
    printf("Hover speed per rotor: %.1f rad/s\n", hover_omega);
    printf("%6s %11s %11s %11s %11s\n",
           "t[s]", "altitude[m]", "vz[m/s]", "roll[deg]", "pitch[deg]");

    for (size_t step = 0; step <= steps; ++step) {
        const double time = step * dt;

        if (time > 1.0 && time <= 1.1) {
            const double boost = 1.01;
            const double reduce = 0.99;
            inputs.rotor_omega[0] = hover_omega * boost;  /* front-right */
            inputs.rotor_omega[1] = hover_omega * reduce; /* front-left */
            inputs.rotor_omega[2] = hover_omega * reduce; /* rear-left */
            inputs.rotor_omega[3] = hover_omega * boost;  /* rear-right */
        } else {
            for (size_t i = 0; i < config.rotor_count; ++i) {
                inputs.rotor_omega[i] = hover_omega;
            }
        }

        if (step % print_stride == 0) {
            double roll, pitch, yaw;
            quaternion_to_euler(state.quaternion, &roll, &pitch, &yaw);
            const double altitude = -state.position[2];      /* NED → altitude */
            const double vertical_velocity = -state.velocity[2];
            printf("%6.2f %11.4f %11.4f %11.4f %11.4f\n",
                   time,
                   altitude,
                   vertical_velocity,
                   rad2deg(roll),
                   rad2deg(pitch));
        }

        if (step == steps) {
            break;
        }

        dm_integrate_rk4(&state,
                         &inputs,
                         dt,
                         sizeof(state),
                         vehicle_derivative,
                         NULL,
                         &k1,
                         &k2,
                         &k3,
                         &k4,
                         &temp);
    }

    return 0;
}
