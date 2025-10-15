#ifndef DYNAMIC_MODELS_LINEAR_PENDULUM_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_PENDULUM_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass;
    double length;
    double gravity;
    double damping;
} dm_pendulum_params_t;

typedef struct {
    double A[2][2];
    double B[2][1];
    double C[2][2];
    double D[2][1];
    double x_eq[2]; /* [theta, omega] */
} dm_pendulum_ss_t;

int dm_pendulum_linearize(const dm_pendulum_params_t* params,
                          dm_pendulum_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
