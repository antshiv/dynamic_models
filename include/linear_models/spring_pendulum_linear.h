#ifndef DYNAMIC_MODELS_LINEAR_SPRING_PENDULUM_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_SPRING_PENDULUM_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass;
    double stiffness;
    double damping;
    double rest_length;
    double gravity;
} dm_spring_pendulum_params_t;

typedef struct {
    double A[4][4];
    double B[4][2];
    double C[4][4];
    double D[4][2];
    double x_eq[4]; /* Equilibrium state: [x, y, vx, vy] */
} dm_spring_pendulum_ss_t;

int dm_spring_pendulum_linearize(const dm_spring_pendulum_params_t* params,
                                 dm_spring_pendulum_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
