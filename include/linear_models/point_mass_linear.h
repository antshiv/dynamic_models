#ifndef DYNAMIC_MODELS_LINEAR_POINT_MASS_LINEAR_H
#define DYNAMIC_MODELS_LINEAR_POINT_MASS_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double A[2][2];
    double B[2][1];
    double C[2][2];
    double D[2][1];
} dm_point_mass_ss_t;

int dm_point_mass_linearize(double mass, dm_point_mass_ss_t* out);

#ifdef __cplusplus
}
#endif

#endif
