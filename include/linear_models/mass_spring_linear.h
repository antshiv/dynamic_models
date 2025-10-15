#ifndef DYNAMIC_MODELS_LINEAR_MASS_SPRING_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_MASS_SPRING_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass;
    double stiffness;
    double damping;
} dm_mass_spring_params_t;

typedef struct {
    double A[2][2];
    double B[2][1];
    double C[2][2];
    double D[2][1];
    double x_eq[2]; /* [position, velocity] */
} dm_mass_spring_ss_t;

int dm_mass_spring_linearize(const dm_mass_spring_params_t* params,
                             dm_mass_spring_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
