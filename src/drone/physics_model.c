#include "drone/physics_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "attitude/quaternion.h"

static void dm_zero_state(dm_state_t* out) {
    memset(out, 0, sizeof(*out));
}

static double dm_vector_norm(const double v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void dm_normalize_axis(const double in[3], double out[3]) {
    const double norm = dm_vector_norm(in);
    if (norm > 0.0) {
        const double inv = 1.0 / norm;
        out[0] = in[0] * inv;
        out[1] = in[1] * inv;
        out[2] = in[2] * inv;
    } else {
        out[0] = out[1] = out[2] = 0.0;
    }
}

static void dm_cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void dm_mat3_vec3_mul(const double mat[3][3],
                             const double vec[3],
                             double out[3]) {
    for (size_t i = 0; i < 3; ++i) {
        out[i] = mat[i][0] * vec[0] +
                 mat[i][1] * vec[1] +
                 mat[i][2] * vec[2];
    }
}

static void dm_quaternion_to_dcm(const double quat_in[4], double dcm[3][3]) {
    double q[4] = {
        quat_in[0], quat_in[1], quat_in[2], quat_in[3]
    };

    const double norm_sq = q[0] * q[0] + q[1] * q[1] +
                           q[2] * q[2] + q[3] * q[3];
    if (norm_sq <= 0.0) {
        q[0] = 1.0;
        q[1] = q[2] = q[3] = 0.0;
    } else {
        quaternion_normalize(q);
    }

    quaternion_to_dcm(q, dcm);
}

static void dm_quaternion_derivative(const double quat[4],
                                     const double omega[3],
                                     double quat_dot[4]) {
    const double wx = omega[0];
    const double wy = omega[1];
    const double wz = omega[2];
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    quat_dot[0] = 0.5 * (-wx * x - wy * y - wz * z);
    quat_dot[1] = 0.5 * ( wx * w + wy * z - wz * y);
    quat_dot[2] = 0.5 * ( wy * w + wz * x - wx * z);
    quat_dot[3] = 0.5 * ( wz * w + wx * y - wy * x);
}

static void dm_add_vec3(double a[3], const double b[3]) {
    a[0] += b[0];
    a[1] += b[1];
    a[2] += b[2];
}

void dm_vehicle_evaluate(const dm_vehicle_model_t* model,
                         const double rotor_omega[DM_MAX_ROTORS],
                         dm_state_t* state_dot) {
    if (!model || !model->config || !rotor_omega || !state_dot) {
        return;
    }

    const dm_vehicle_config_t* cfg = model->config;
    if (!cfg->mass || cfg->rotor_count == 0 ||
        cfg->rotor_count > DM_MAX_ROTORS) {
        dm_zero_state(state_dot);
        return;
    }

    double total_force_body[3] = {0.0, 0.0, 0.0};
    double total_torque_body[3] = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < cfg->rotor_count; ++i) {
        const dm_rotor_config_t* rotor = &cfg->rotors[i];
        const double omega = rotor_omega[i];
        const double omega_sq = omega * omega;

        double axis_unit[3];
        dm_normalize_axis(rotor->axis_body, axis_unit);
        if (axis_unit[0] == 0.0 && axis_unit[1] == 0.0 && axis_unit[2] == 0.0) {
            continue;
        }

        const double thrust = rotor->thrust_coeff * omega_sq;
        const double torque_mag = rotor->torque_coeff * omega_sq * rotor->direction;

        double thrust_vec[3] = {
            axis_unit[0] * thrust,
            axis_unit[1] * thrust,
            axis_unit[2] * thrust,
        };
        dm_add_vec3(total_force_body, thrust_vec);

        double moment_arm[3];
        dm_cross(rotor->position_body, thrust_vec, moment_arm);
        dm_add_vec3(total_torque_body, moment_arm);

        double reaction[3] = {
            axis_unit[0] * torque_mag,
            axis_unit[1] * torque_mag,
            axis_unit[2] * torque_mag,
        };
        dm_add_vec3(total_torque_body, reaction);
    }

    const double mass_inv = 1.0 / cfg->mass;
    double rotation[3][3];
    dm_quaternion_to_dcm(model->state.quaternion, rotation);

    double force_inertial[3];
    dm_mat3_vec3_mul((const double (*)[3])rotation, total_force_body, force_inertial);

    double acceleration[3] = {
        force_inertial[0] * mass_inv,
        force_inertial[1] * mass_inv,
        force_inertial[2] * mass_inv + cfg->gravity,
    };

    double quat_dot[4];
    dm_quaternion_derivative(model->state.quaternion,
                             model->state.angular_rate,
                             quat_dot);

    double inertia_omega[3];
    dm_mat3_vec3_mul(cfg->inertia, model->state.angular_rate, inertia_omega);

    double coriolis[3];
    dm_cross(model->state.angular_rate, inertia_omega, coriolis);

    double torque_net[3] = {
        total_torque_body[0] - coriolis[0],
        total_torque_body[1] - coriolis[1],
        total_torque_body[2] - coriolis[2],
    };

    double angular_accel[3];
    dm_mat3_vec3_mul(cfg->inertia_inv, torque_net, angular_accel);

    /* Position derivative equals current velocity */
    state_dot->position[0] = model->state.velocity[0];
    state_dot->position[1] = model->state.velocity[1];
    state_dot->position[2] = model->state.velocity[2];

    state_dot->velocity[0] = acceleration[0];
    state_dot->velocity[1] = acceleration[1];
    state_dot->velocity[2] = acceleration[2];

    state_dot->quaternion[0] = quat_dot[0];
    state_dot->quaternion[1] = quat_dot[1];
    state_dot->quaternion[2] = quat_dot[2];
    state_dot->quaternion[3] = quat_dot[3];

    state_dot->angular_rate[0] = angular_accel[0];
    state_dot->angular_rate[1] = angular_accel[1];
    state_dot->angular_rate[2] = angular_accel[2];
}
