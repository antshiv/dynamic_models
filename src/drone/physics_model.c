#include "drone/physics_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "attitude/quaternion.h"

static void dm_zero_state(dm_state_t* out) {
    memset(out, 0, sizeof(*out));
}

static int dm_is_finite_array(const double* values, size_t count) {
    if (!values) {
        return 0;
    }
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(values[i])) {
            return 0;
        }
    }
    return 1;
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

static int dm_state_is_valid(const dm_state_t* state) {
    if (!state || !dm_is_finite_array(state->position, 3) ||
        !dm_is_finite_array(state->velocity, 3) ||
        !dm_is_finite_array(state->quaternion, 4) ||
        !dm_is_finite_array(state->angular_rate, 3)) {
        return 0;
    }

    const double norm_sq =
        state->quaternion[0] * state->quaternion[0] +
        state->quaternion[1] * state->quaternion[1] +
        state->quaternion[2] * state->quaternion[2] +
        state->quaternion[3] * state->quaternion[3];
    return isfinite(norm_sq) && norm_sq > 1e-24;
}

dm_result_t dm_vehicle_config_validate(const dm_vehicle_config_t* cfg) {
    if (!cfg) {
        return DM_INVALID_ARGUMENT;
    }
    if (cfg->rotor_count == 0 || cfg->rotor_count > DM_MAX_ROTORS ||
        !isfinite(cfg->mass) || cfg->mass <= 0.0 ||
        !isfinite(cfg->gravity) || cfg->gravity <= 0.0 ||
        !dm_is_finite_array(&cfg->inertia[0][0], 9) ||
        !dm_is_finite_array(&cfg->inertia_inv[0][0], 9)) {
        return DM_INVALID_CONFIG;
    }

    for (size_t row = 0; row < 3; ++row) {
        if (cfg->inertia[row][row] <= 0.0) {
            return DM_INVALID_CONFIG;
        }
        for (size_t col = 0; col < 3; ++col) {
            if (fabs(cfg->inertia[row][col] - cfg->inertia[col][row]) > 1e-10) {
                return DM_INVALID_CONFIG;
            }
            double product = 0.0;
            for (size_t k = 0; k < 3; ++k) {
                product += cfg->inertia[row][k] * cfg->inertia_inv[k][col];
            }
            const double expected = row == col ? 1.0 : 0.0;
            if (fabs(product - expected) > 1e-7) {
                return DM_INVALID_CONFIG;
            }
        }
    }

    for (size_t i = 0; i < cfg->rotor_count; ++i) {
        const dm_rotor_config_t* rotor = &cfg->rotors[i];
        if (!dm_is_finite_array(rotor->position_body, 3) ||
            !dm_is_finite_array(rotor->axis_body, 3) ||
            !isfinite(rotor->direction) ||
            !isfinite(rotor->thrust_coeff) ||
            !isfinite(rotor->torque_coeff) ||
            dm_vector_norm(rotor->axis_body) <= 1e-12 ||
            (rotor->direction != -1.0 && rotor->direction != 1.0) ||
            rotor->thrust_coeff <= 0.0 || rotor->torque_coeff < 0.0) {
            return DM_INVALID_CONFIG;
        }
    }
    return DM_OK;
}

dm_result_t dm_vehicle_evaluate_checked(
    const dm_vehicle_model_t* model,
    const double rotor_omega[DM_MAX_ROTORS],
    dm_state_t* state_dot) {
    if (!state_dot) {
        return DM_INVALID_ARGUMENT;
    }
    dm_zero_state(state_dot);
    if (!model || !model->config || !rotor_omega) {
        return DM_INVALID_ARGUMENT;
    }
    if (dm_vehicle_config_validate(model->config) != DM_OK) {
        return DM_INVALID_CONFIG;
    }
    if (!dm_state_is_valid(&model->state)) {
        return DM_INVALID_STATE;
    }
    for (size_t i = 0; i < model->config->rotor_count; ++i) {
        if (!isfinite(rotor_omega[i]) || rotor_omega[i] < 0.0) {
            return DM_INVALID_ARGUMENT;
        }
    }

    const dm_vehicle_config_t* cfg = model->config;
    double total_force_body[3] = {0.0, 0.0, 0.0};
    double total_torque_body[3] = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < cfg->rotor_count; ++i) {
        const dm_rotor_config_t* rotor = &cfg->rotors[i];
        const double omega_sq = rotor_omega[i] * rotor_omega[i];
        double axis_unit[3];
        dm_normalize_axis(rotor->axis_body, axis_unit);

        const double thrust = rotor->thrust_coeff * omega_sq;
        const double torque_mag =
            rotor->torque_coeff * omega_sq * rotor->direction;
        double thrust_vec[3] = {
            axis_unit[0] * thrust,
            axis_unit[1] * thrust,
            axis_unit[2] * thrust,
        };
        dm_add_vec3(total_force_body, thrust_vec);

        double moment_arm[3];
        dm_cross(rotor->position_body, thrust_vec, moment_arm);
        dm_add_vec3(total_torque_body, moment_arm);
        const double reaction[3] = {
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
    dm_mat3_vec3_mul((const double (*)[3])rotation,
                     total_force_body, force_inertial);

    state_dot->position[0] = model->state.velocity[0];
    state_dot->position[1] = model->state.velocity[1];
    state_dot->position[2] = model->state.velocity[2];
    state_dot->velocity[0] = force_inertial[0] * mass_inv;
    state_dot->velocity[1] = force_inertial[1] * mass_inv;
    state_dot->velocity[2] = force_inertial[2] * mass_inv + cfg->gravity;
    dm_quaternion_derivative(model->state.quaternion,
                             model->state.angular_rate,
                             state_dot->quaternion);

    double inertia_omega[3];
    dm_mat3_vec3_mul(cfg->inertia, model->state.angular_rate, inertia_omega);
    double coriolis[3];
    dm_cross(model->state.angular_rate, inertia_omega, coriolis);
    const double torque_net[3] = {
        total_torque_body[0] - coriolis[0],
        total_torque_body[1] - coriolis[1],
        total_torque_body[2] - coriolis[2],
    };
    dm_mat3_vec3_mul(cfg->inertia_inv, torque_net,
                     state_dot->angular_rate);

    if (!dm_is_finite_array(state_dot->position, 3) ||
        !dm_is_finite_array(state_dot->velocity, 3) ||
        !dm_is_finite_array(state_dot->quaternion, 4) ||
        !dm_is_finite_array(state_dot->angular_rate, 3)) {
        dm_zero_state(state_dot);
        return DM_NUMERICAL_FAILURE;
    }
    return DM_OK;
}

void dm_vehicle_evaluate(const dm_vehicle_model_t* model,
                         const double rotor_omega[DM_MAX_ROTORS],
                         dm_state_t* state_dot) {
    (void)dm_vehicle_evaluate_checked(model, rotor_omega, state_dot);
}

static void dm_state_add_scaled(dm_state_t* out, const dm_state_t* base,
                                const dm_state_t* derivative, double scale) {
    for (size_t i = 0; i < 3; ++i) {
        out->position[i] = base->position[i] + scale * derivative->position[i];
        out->velocity[i] = base->velocity[i] + scale * derivative->velocity[i];
        out->angular_rate[i] =
            base->angular_rate[i] + scale * derivative->angular_rate[i];
    }
    for (size_t i = 0; i < 4; ++i) {
        out->quaternion[i] =
            base->quaternion[i] + scale * derivative->quaternion[i];
    }
}

static void dm_state_rk4_combine(dm_state_t* out, const dm_state_t* initial,
                                 const dm_state_t* k1, const dm_state_t* k2,
                                 const dm_state_t* k3, const dm_state_t* k4,
                                 double dt) {
    for (size_t i = 0; i < 3; ++i) {
        out->position[i] = initial->position[i] + (dt / 6.0) *
            (k1->position[i] + 2.0 * k2->position[i] +
             2.0 * k3->position[i] + k4->position[i]);
        out->velocity[i] = initial->velocity[i] + (dt / 6.0) *
            (k1->velocity[i] + 2.0 * k2->velocity[i] +
             2.0 * k3->velocity[i] + k4->velocity[i]);
        out->angular_rate[i] = initial->angular_rate[i] + (dt / 6.0) *
            (k1->angular_rate[i] + 2.0 * k2->angular_rate[i] +
             2.0 * k3->angular_rate[i] + k4->angular_rate[i]);
    }
    for (size_t i = 0; i < 4; ++i) {
        out->quaternion[i] = initial->quaternion[i] + (dt / 6.0) *
            (k1->quaternion[i] + 2.0 * k2->quaternion[i] +
             2.0 * k3->quaternion[i] + k4->quaternion[i]);
    }
}

dm_result_t dm_vehicle_step_rk4_checked(
    dm_vehicle_model_t* model,
    const double rotor_omega[DM_MAX_ROTORS],
    double dt) {
    if (!model || !model->config || !rotor_omega ||
        !isfinite(dt) || dt <= 0.0) {
        return DM_INVALID_ARGUMENT;
    }

    const dm_state_t initial = model->state;
    dm_state_t candidate;
    dm_state_t k1;
    dm_state_t k2;
    dm_state_t k3;
    dm_state_t k4;
    dm_vehicle_model_t stage = *model;

    dm_result_t result = dm_vehicle_evaluate_checked(&stage, rotor_omega, &k1);
    if (result != DM_OK) {
        return result;
    }
    dm_state_add_scaled(&stage.state, &initial, &k1, 0.5 * dt);
    result = dm_vehicle_evaluate_checked(&stage, rotor_omega, &k2);
    if (result != DM_OK) {
        return result;
    }
    dm_state_add_scaled(&stage.state, &initial, &k2, 0.5 * dt);
    result = dm_vehicle_evaluate_checked(&stage, rotor_omega, &k3);
    if (result != DM_OK) {
        return result;
    }
    dm_state_add_scaled(&stage.state, &initial, &k3, dt);
    result = dm_vehicle_evaluate_checked(&stage, rotor_omega, &k4);
    if (result != DM_OK) {
        return result;
    }

    dm_state_rk4_combine(&candidate, &initial, &k1, &k2, &k3, &k4, dt);

    const double q_norm = sqrt(
        candidate.quaternion[0] * candidate.quaternion[0] +
        candidate.quaternion[1] * candidate.quaternion[1] +
        candidate.quaternion[2] * candidate.quaternion[2] +
        candidate.quaternion[3] * candidate.quaternion[3]);
    if (!isfinite(q_norm) || q_norm <= 1e-12) {
        return DM_NUMERICAL_FAILURE;
    }
    for (size_t i = 0; i < 4; ++i) {
        candidate.quaternion[i] /= q_norm;
    }
    if (!dm_state_is_valid(&candidate)) {
        return DM_NUMERICAL_FAILURE;
    }
    model->state = candidate;
    return DM_OK;
}
