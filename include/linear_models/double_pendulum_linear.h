#ifndef DYNAMIC_MODELS_LINEAR_DOUBLE_PENDULUM_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_DOUBLE_PENDULUM_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass1;
    double mass2;
    double length1;
    double length2;
    double gravity;
    double damping1;
    double damping2;
} dm_double_pendulum_params_t;

typedef struct {
    double A[4][4];
    double B[4][2];
    double C[4][4];
    double D[4][2];
    double x_eq[4]; /* [theta1, theta2, omega1, omega2] */
} dm_double_pendulum_ss_t;

int dm_double_pendulum_linearize(const dm_double_pendulum_params_t* params,
                                 dm_double_pendulum_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
