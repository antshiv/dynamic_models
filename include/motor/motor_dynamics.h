/* Copyright (c) Antshiv Robotics */
#ifndef DYNAMIC_MODELS_MOTOR_DYNAMICS_H
#define DYNAMIC_MODELS_MOTOR_DYNAMICS_H

#include "motor/dc_motor.h"
#include "motor/bldc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    md_dc_motor_config_t motor;
    double dc_bus_voltage_v;
} dm_motor_actuator_config_t;

typedef struct {
    md_dc_motor_state_t motor;
} dm_motor_actuator_state_t;

typedef struct {
    /* Signed duty command in the closed interval [-1, 1]. */
    double normalized_command;
    double shaft_load_torque_nm;
} dm_motor_actuator_input_t;

/* Translate the platform-independent motor status into dynamic_models status. */
typedef enum {
    DM_MOTOR_OK = 0,
    DM_MOTOR_INVALID_ARGUMENT,
    DM_MOTOR_INVALID_CONFIG,
    DM_MOTOR_NUMERICAL_FAILURE
} dm_motor_result_t;

dm_motor_result_t dm_motor_actuator_config_validate(
    const dm_motor_actuator_config_t *config);

/*
 * Apply an ideal averaged bridge voltage and advance the external motor plant.
 * Switching, dead time, and current regulation belong in motorDynamics later.
 */
dm_motor_result_t dm_motor_actuator_step_checked(
    const dm_motor_actuator_config_t *config,
    dm_motor_actuator_state_t *state,
    const dm_motor_actuator_input_t *input,
    double dt_s);

double dm_motor_actuator_speed_rad_s(
    const dm_motor_actuator_state_t *state);

typedef struct {
    md_bldc_config_t motor;
    double dc_bus_voltage_v;
} dm_bldc_actuator_config_t;

typedef struct {
    md_bldc_state_t motor;
} dm_bldc_actuator_state_t;

typedef struct {
    double duty_cycle;
    uint32_t commutation_sector;
    double shaft_load_torque_nm;
} dm_bldc_actuator_input_t;

dm_motor_result_t dm_bldc_actuator_config_validate(
    const dm_bldc_actuator_config_t *config);

/* The ESC/control layer owns sector selection; this adapter owns composition. */
dm_motor_result_t dm_bldc_actuator_step_checked(
    const dm_bldc_actuator_config_t *config,
    dm_bldc_actuator_state_t *state,
    const dm_bldc_actuator_input_t *input,
    double dt_s);

double dm_bldc_actuator_speed_rad_s(
    const dm_bldc_actuator_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
