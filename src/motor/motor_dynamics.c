#include "motor/motor_dynamics.h"

#include <math.h>

static dm_motor_result_t dm_motor_translate_status(md_status_t status) {
    switch (status) {
        case MD_STATUS_OK:
            return DM_MOTOR_OK;
        case MD_STATUS_INVALID_CONFIG:
        case MD_STATUS_SINGULAR_MODEL:
            return DM_MOTOR_INVALID_CONFIG;
        case MD_STATUS_NULL_POINTER:
        case MD_STATUS_INVALID_ARGUMENT:
            return DM_MOTOR_INVALID_ARGUMENT;
        case MD_STATUS_NUMERICAL_FAILURE:
        default:
            return DM_MOTOR_NUMERICAL_FAILURE;
    }
}

dm_motor_result_t dm_motor_actuator_config_validate(
    const dm_motor_actuator_config_t *config) {
    if (!config || !isfinite(config->dc_bus_voltage_v) ||
        config->dc_bus_voltage_v <= 0.0) {
        return config ? DM_MOTOR_INVALID_CONFIG : DM_MOTOR_INVALID_ARGUMENT;
    }
    return dm_motor_translate_status(md_dc_motor_config_validate(&config->motor));
}

dm_motor_result_t dm_motor_actuator_step_checked(
    const dm_motor_actuator_config_t *config,
    dm_motor_actuator_state_t *state,
    const dm_motor_actuator_input_t *input,
    double dt_s) {
    dm_motor_result_t result = dm_motor_actuator_config_validate(config);
    if (result != DM_MOTOR_OK) {
        return result;
    }
    if (!state || !input || !isfinite(input->normalized_command) ||
        input->normalized_command < -1.0 ||
        input->normalized_command > 1.0 ||
        !isfinite(input->shaft_load_torque_nm) ||
        !isfinite(dt_s) || dt_s <= 0.0) {
        return DM_MOTOR_INVALID_ARGUMENT;
    }

    const md_dc_motor_input_t motor_input = {
        .terminal_voltage_v =
            input->normalized_command * config->dc_bus_voltage_v,
        .load_torque_nm = input->shaft_load_torque_nm,
    };
    return dm_motor_translate_status(md_dc_motor_step_rk4_checked(
        &config->motor, &state->motor, &motor_input, dt_s));
}

double dm_motor_actuator_speed_rad_s(
    const dm_motor_actuator_state_t *state) {
    return state ? state->motor.angular_speed_rad_s : NAN;
}
