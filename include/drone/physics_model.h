/* Copyright (c) Antshiv Robotics */
#ifndef DYNAMIC_MODELS_DRONE_PHYSICS_MODEL_H
#define DYNAMIC_MODELS_DRONE_PHYSICS_MODEL_H

#include <stddef.h>
#include <stdint.h>
#include <stdalign.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Maximum number of rotors supported by the generic vehicle model.
 * Increase if you need octocopters or exotic layouts.
 */
#ifndef DM_MAX_ROTORS
#define DM_MAX_ROTORS 8
#endif

/**
 * State vector for the rigid body:
 *  - position: inertial frame coordinates (m)
 *  - velocity: inertial frame (m/s)
 *  - quaternion: body-to-inertial orientation [w, x, y, z]
 *  - angular_rate: body frame angular velocity (rad/s)
 */
typedef struct dm_state_s {
    double position[3];
    double velocity[3];
    double quaternion[4];
    double angular_rate[3];
} dm_state_t;

/**
 * Rotor configuration describing geometry and aerodynamic coefficients.
 * direction is +1 or -1 depending on spin (cw/ccw) for torque contributions.
 */
typedef struct dm_rotor_config_s {
    double position_body[3];
    double axis_body[3];
    double direction;
    double thrust_coeff;
    double torque_coeff;
} dm_rotor_config_t;

/**
 * Runtime rotor state cache aligned for SIMD friendliness.
 */
typedef struct alignas(16) dm_rotor_state_s {
    double omega;     /* rad/s */
    double thrust;    /* N */
    double torque;    /* N·m about rotor axis */
} dm_rotor_state_t;

/**
 * Vehicle model configuration. Precompute inertia inverse/allocation matrices
 * when possible for faster runtime evaluation.
 */
typedef struct dm_vehicle_config_s {
    size_t rotor_count;
    double mass;
    double gravity;
    double inertia[3][3];
    double inertia_inv[3][3];
    dm_rotor_config_t rotors[DM_MAX_ROTORS];
} dm_vehicle_config_t;

/**
 * Aggregated runtime model container. Holds the current state plus caches that
 * are reused across evaluations to keep memory accesses contiguous.
 */
typedef struct dm_vehicle_model_s {
    const dm_vehicle_config_t* config;
    dm_state_t state;
    dm_rotor_state_t rotor_cache[DM_MAX_ROTORS];
    double force_body[3];
    double torque_body[3];
} dm_vehicle_model_t;

/**
 * Compute state derivatives given rotor speeds (ω) and the current model.
 * The caller provides state_dot as output buffer.
 */
void dm_vehicle_evaluate(const dm_vehicle_model_t* model,
                         const double rotor_omega[DM_MAX_ROTORS],
                         dm_state_t* state_dot);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIC_MODELS_DRONE_PHYSICS_MODEL_H */
