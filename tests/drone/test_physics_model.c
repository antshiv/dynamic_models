#include <assert.h>
#include <math.h>
#include <string.h>
#include "drone/physics_model.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

static void set_identity_quaternion(double q[4]) {
    q[0] = 1.0;
    q[1] = q[2] = q[3] = 0.0;
}

static void make_diagonal(double mat[3][3], double d0, double d1, double d2) {
    memset(mat, 0, sizeof(double) * 9);
    mat[0][0] = d0;
    mat[1][1] = d1;
    mat[2][2] = d2;
}

static dm_vehicle_config_t make_base_config(void) {
    dm_vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.rotor_count = 1;
    cfg.mass = 1.0;
    cfg.gravity = 9.81;
    make_diagonal(cfg.inertia, 0.02, 0.03, 0.04);
    make_diagonal(cfg.inertia_inv, 1.0 / 0.02, 1.0 / 0.03, 1.0 / 0.04);

    cfg.rotors[0].position_body[0] = 0.0;
    cfg.rotors[0].position_body[1] = 0.0;
    cfg.rotors[0].position_body[2] = 0.0;
    cfg.rotors[0].axis_body[0] = 0.0;
    cfg.rotors[0].axis_body[1] = 0.0;
    cfg.rotors[0].axis_body[2] = -1.0;
    cfg.rotors[0].direction = 1.0;
    cfg.rotors[0].thrust_coeff = 1.0;
    cfg.rotors[0].torque_coeff = 0.0;
    return cfg;
}

static dm_vehicle_model_t make_model(const dm_vehicle_config_t* cfg) {
    dm_vehicle_model_t model;
    memset(&model, 0, sizeof(model));
    model.config = cfg;
    set_identity_quaternion(model.state.quaternion);
    return model;
}

static void test_free_fall_gravity_only(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);

    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    dm_state_t state_dot;
    dm_vehicle_evaluate(&model, rotor_omega, &state_dot);

    assert(nearly_equal(state_dot.position[0], 0.0, 1e-12));
    assert(nearly_equal(state_dot.position[1], 0.0, 1e-12));
    assert(nearly_equal(state_dot.position[2], 0.0, 1e-12));

    assert(nearly_equal(state_dot.velocity[0], 0.0, 1e-12));
    assert(nearly_equal(state_dot.velocity[1], 0.0, 1e-12));
    assert(nearly_equal(state_dot.velocity[2], cfg.gravity, 1e-12));

    assert(nearly_equal(state_dot.angular_rate[0], 0.0, 1e-12));
    assert(nearly_equal(state_dot.angular_rate[1], 0.0, 1e-12));
    assert(nearly_equal(state_dot.angular_rate[2], 0.0, 1e-12));
}

static void test_hover_balance(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);

    const double hover_force = cfg.mass * cfg.gravity;
    const double omega = sqrt(hover_force / cfg.rotors[0].thrust_coeff);
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    rotor_omega[0] = omega;

    dm_state_t state_dot;
    dm_vehicle_evaluate(&model, rotor_omega, &state_dot);

    assert(nearly_equal(state_dot.velocity[2], 0.0, 1e-9));
    assert(nearly_equal(state_dot.velocity[0], 0.0, 1e-12));
    assert(nearly_equal(state_dot.velocity[1], 0.0, 1e-12));
}

static void test_offset_rotor_generates_roll_torque(void) {
    dm_vehicle_config_t cfg = make_base_config();
    cfg.rotors[0].position_body[0] = 0.0;
    cfg.rotors[0].position_body[1] = 0.1;
    cfg.rotors[0].position_body[2] = 0.0;
    cfg.rotors[0].torque_coeff = 0.0;

    dm_vehicle_model_t model = make_model(&cfg);

    const double thrust = 10.0;
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    rotor_omega[0] = sqrt(thrust / cfg.rotors[0].thrust_coeff);

    dm_state_t state_dot;
    dm_vehicle_evaluate(&model, rotor_omega, &state_dot);

    /* Torque = r × F => magnitude about x-axis = -1.0 N·m */
    const double expected_roll_alpha = -1.0 / cfg.inertia[0][0];
    assert(nearly_equal(state_dot.angular_rate[0], expected_roll_alpha, 1e-9));
    assert(nearly_equal(state_dot.angular_rate[1], 0.0, 1e-12));
    assert(nearly_equal(state_dot.angular_rate[2], 0.0, 1e-12));
}

static void test_reaction_torque_about_yaw(void) {
    dm_vehicle_config_t cfg = make_base_config();
    cfg.rotors[0].torque_coeff = 0.01;
    cfg.rotors[0].direction = 1.0;
    cfg.rotors[0].position_body[0] = 0.0;
    cfg.rotors[0].position_body[1] = 0.0;
    cfg.rotors[0].position_body[2] = 0.0;

    dm_vehicle_model_t model = make_model(&cfg);

    const double omega = 50.0;
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    rotor_omega[0] = omega;

    dm_state_t state_dot;
    dm_vehicle_evaluate(&model, rotor_omega, &state_dot);

    const double torque = -cfg.rotors[0].torque_coeff * omega * omega;
    const double expected_yaw_alpha = torque / cfg.inertia[2][2];
    assert(nearly_equal(state_dot.angular_rate[2], expected_yaw_alpha, 1e-6));
}

static void test_checked_evaluation_fails_closed(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    dm_state_t state_dot;
    memset(&state_dot, 0x5a, sizeof(state_dot));

    rotor_omega[0] = NAN;
    assert(dm_vehicle_evaluate_checked(&model, rotor_omega, &state_dot) ==
           DM_INVALID_ARGUMENT);
    for (size_t i = 0; i < 3; ++i) {
        assert(state_dot.position[i] == 0.0);
        assert(state_dot.velocity[i] == 0.0);
        assert(state_dot.angular_rate[i] == 0.0);
    }
    for (size_t i = 0; i < 4; ++i) {
        assert(state_dot.quaternion[i] == 0.0);
    }

    cfg.inertia_inv[0][0] = 1.0;
    rotor_omega[0] = 0.0;
    assert(dm_vehicle_evaluate_checked(&model, rotor_omega, &state_dot) ==
           DM_INVALID_CONFIG);
}

static void test_rk4_free_fall_matches_constant_acceleration(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    const double dt = 0.1;

    assert(dm_vehicle_step_rk4_checked(&model, rotor_omega, dt) == DM_OK);
    assert(nearly_equal(model.state.position[2],
                        0.5 * cfg.gravity * dt * dt, 1e-12));
    assert(nearly_equal(model.state.velocity[2], cfg.gravity * dt, 1e-12));
    assert(nearly_equal(model.state.quaternion[0], 1.0, 1e-12));
}

static void test_rk4_preserves_unit_quaternion(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    model.state.angular_rate[2] = 1.0;

    for (size_t i = 0; i < 1000; ++i) {
        assert(dm_vehicle_step_rk4_checked(&model, rotor_omega, 0.001) == DM_OK);
    }
    const double norm = sqrt(
        model.state.quaternion[0] * model.state.quaternion[0] +
        model.state.quaternion[1] * model.state.quaternion[1] +
        model.state.quaternion[2] * model.state.quaternion[2] +
        model.state.quaternion[3] * model.state.quaternion[3]);
    assert(nearly_equal(norm, 1.0, 1e-12));
    assert(nearly_equal(model.state.quaternion[0], cos(0.5), 1e-9));
    assert(nearly_equal(model.state.quaternion[3], sin(0.5), 1e-9));
}

static void test_failed_step_does_not_mutate_state(void) {
    dm_vehicle_config_t cfg = make_base_config();
    dm_vehicle_model_t model = make_model(&cfg);
    const dm_state_t original = model.state;
    double rotor_omega[DM_MAX_ROTORS] = {0.0};

    assert(dm_vehicle_step_rk4_checked(&model, rotor_omega, -0.01) ==
           DM_INVALID_ARGUMENT);
    assert(memcmp(&model.state, &original, sizeof(original)) == 0);
}

static void test_offset_rotor_generates_pitch_torque(void) {
    dm_vehicle_config_t cfg = make_base_config();
    cfg.rotors[0].position_body[0] = 0.12;
    cfg.rotors[0].position_body[1] = 0.0;
    cfg.rotors[0].position_body[2] = 0.0;
    cfg.rotors[0].torque_coeff = 0.0;

    dm_vehicle_model_t model = make_model(&cfg);

    const double thrust = 8.0;
    double rotor_omega[DM_MAX_ROTORS] = {0.0};
    rotor_omega[0] = sqrt(thrust / cfg.rotors[0].thrust_coeff);

    dm_state_t state_dot;
    dm_vehicle_evaluate(&model, rotor_omega, &state_dot);

    const double expected_pitch_alpha = (cfg.rotors[0].position_body[0] * thrust) /
                                        cfg.inertia[1][1];

    assert(nearly_equal(state_dot.angular_rate[1], expected_pitch_alpha, 1e-9));
    assert(nearly_equal(state_dot.angular_rate[0], 0.0, 1e-12));
    assert(nearly_equal(state_dot.angular_rate[2], 0.0, 1e-12));
}

int main(void) {
    test_free_fall_gravity_only();
    test_hover_balance();
    test_offset_rotor_generates_roll_torque();
    test_offset_rotor_generates_pitch_torque();
    test_reaction_torque_about_yaw();
    test_checked_evaluation_fails_closed();
    test_rk4_free_fall_matches_constant_acceleration();
    test_rk4_preserves_unit_quaternion();
    test_failed_step_does_not_mutate_state();
    return 0;
}
