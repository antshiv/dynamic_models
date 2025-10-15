#ifndef DYNAMIC_MODELS_LINEAR_INVERTED_PENDULUM_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_INVERTED_PENDULUM_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass_cart;
    double mass_pole;
    double length;
    double gravity;
    double damping_cart;
    double damping_pole;
} dm_inverted_pendulum_params_t;

typedef struct {
    double A[4][4];
    double B[4][1];
    double C[4][4];
    double D[4][1];
    double x_eq[4]; /* [x, theta, x_dot, theta_dot] */
    double u_eq;    /* equilibrium cart force */
} dm_inverted_pendulum_ss_t;

int dm_inverted_pendulum_linearize(const dm_inverted_pendulum_params_t* params,
                                   dm_inverted_pendulum_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
