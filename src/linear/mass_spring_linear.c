#include "linear_models/mass_spring_linear.h"

#include <stddef.h>

int dm_mass_spring_linearize(const dm_mass_spring_params_t* params,
                             dm_mass_spring_ss_t* out) {
    if (!params || !out) {
        return -1;
    }

    const double m = params->mass;
    const double k = params->stiffness;
    const double c = params->damping;

    if (m <= 0.0 || k < 0.0 || c < 0.0) {
        return -1;
    }

    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            out->A[i][j] = 0.0;
            out->C[i][j] = 0.0;
        }
        out->B[i][0] = 0.0;
        out->D[i][0] = 0.0;
        out->x_eq[i] = 0.0;
    }

    out->A[0][1] = 1.0;
    out->A[1][0] = (k > 0.0) ? -k / m : 0.0;
    out->A[1][1] = (c > 0.0) ? -c / m : 0.0;
    out->B[1][0] = 1.0 / m;

    out->C[0][0] = 1.0;
    out->C[1][1] = 1.0;

    return 0;
}
