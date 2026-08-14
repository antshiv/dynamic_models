#include "drone/physics_model.h"
#include "motor/motor_dynamics.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static dm_motor_actuator_config_t motor_fixture(void) {
    const dm_motor_actuator_config_t config = {
        .motor = {
            .resistance_ohm = 2.0,
            .inductance_h = 0.5,
            .back_emf_v_per_rad_s = 0.1,
            .torque_constant_nm_per_a = 0.1,
            .rotor_inertia_kg_m2 = 0.02,
            .viscous_friction_nm_per_rad_s = 0.02,
        },
        .dc_bus_voltage_v = 12.0,
    };
    return config;
}

static dm_bldc_actuator_config_t bldc_fixture(void) {
    const dm_bldc_actuator_config_t config = {
        .motor = {
            .phase_resistance_ohm = 0.4,
            .phase_inductance_h = 0.01,
            .back_emf_v_per_rad_s = 0.02,
            .torque_constant_nm_per_a = 0.02,
            .pole_pairs = 7,
            .rotor_inertia_kg_m2 = 0.002,
            .viscous_friction_nm_per_rad_s = 0.001,
        },
        .dc_bus_voltage_v = 12.0,
    };
    return config;
}

static void test_motor_adapter(void) {
    const dm_motor_actuator_config_t config = motor_fixture();
    const dm_motor_actuator_input_t input = {1.0, 0.2};
    dm_motor_actuator_state_t state = {{0.0, 0.0, 0.0}};

    assert(dm_motor_actuator_config_validate(&config) == DM_MOTOR_OK);
    for (size_t step = 0; step < 200000; ++step) {
        assert(dm_motor_actuator_step_checked(
            &config, &state, &input, 0.0001) == DM_MOTOR_OK);
    }
    assert(fabs(dm_motor_actuator_speed_rad_s(&state) - 16.0) < 1e-6);
}

static void test_motor_speed_feeds_vehicle(void) {
    const dm_bldc_actuator_config_t motor_config = bldc_fixture();
    dm_bldc_actuator_state_t motor_state = {
        .motor = {{0.0, 0.0, 0.0}, 0.0, 0.0},
    };
    dm_bldc_actuator_input_t motor_input = {0.35, 0, 0.01};
    for (size_t step = 0; step < 10000; ++step) {
        motor_input.commutation_sector = (uint32_t)((step / 1000) % 6);
        assert(dm_bldc_actuator_step_checked(
            &motor_config, &motor_state, &motor_input, 0.00001) ==
            DM_MOTOR_OK);
    }

    dm_vehicle_config_t vehicle_config;
    memset(&vehicle_config, 0, sizeof(vehicle_config));
    vehicle_config.rotor_count = 1;
    vehicle_config.mass = 1.0;
    vehicle_config.gravity = 9.81;
    vehicle_config.inertia[0][0] = 1.0;
    vehicle_config.inertia[1][1] = 1.0;
    vehicle_config.inertia[2][2] = 1.0;
    vehicle_config.inertia_inv[0][0] = 1.0;
    vehicle_config.inertia_inv[1][1] = 1.0;
    vehicle_config.inertia_inv[2][2] = 1.0;
    vehicle_config.rotors[0].axis_body[2] = -1.0;
    vehicle_config.rotors[0].direction = 1.0;
    vehicle_config.rotors[0].thrust_coeff = 1e-3;
    vehicle_config.rotors[0].torque_coeff = 1e-5;

    dm_vehicle_model_t vehicle;
    memset(&vehicle, 0, sizeof(vehicle));
    vehicle.config = &vehicle_config;
    vehicle.state.quaternion[0] = 1.0;

    double rotor_speed[DM_MAX_ROTORS] = {0.0};
    rotor_speed[0] = fabs(dm_bldc_actuator_speed_rad_s(&motor_state));
    dm_state_t derivative;
    assert(dm_vehicle_evaluate_checked(
        &vehicle, rotor_speed, &derivative) == DM_OK);
    assert(derivative.velocity[2] < vehicle_config.gravity);
}

static void test_invalid_bldc_sector_preserves_state(void) {
    const dm_bldc_actuator_config_t config = bldc_fixture();
    const dm_bldc_actuator_input_t input = {0.5, 6, 0.0};
    dm_bldc_actuator_state_t state = {
        .motor = {{1.0, -1.0, 0.0}, 2.0, 3.0},
    };
    const dm_bldc_actuator_state_t before = state;

    assert(dm_bldc_actuator_step_checked(
        &config, &state, &input, 0.001) == DM_MOTOR_INVALID_ARGUMENT);
    assert(memcmp(&state, &before, sizeof(state)) == 0);
}

static void test_invalid_command_preserves_state(void) {
    const dm_motor_actuator_config_t config = motor_fixture();
    const dm_motor_actuator_input_t input = {1.1, 0.0};
    dm_motor_actuator_state_t state = {{1.0, 2.0, 3.0}};
    const dm_motor_actuator_state_t before = state;

    assert(dm_motor_actuator_step_checked(
        &config, &state, &input, 0.001) == DM_MOTOR_INVALID_ARGUMENT);
    assert(memcmp(&state, &before, sizeof(state)) == 0);
}

int main(void) {
    test_motor_adapter();
    test_motor_speed_feeds_vehicle();
    test_invalid_command_preserves_state();
    test_invalid_bldc_sector_preserves_state();
    puts("dynamic_models motor integration tests passed");
    return 0;
}
