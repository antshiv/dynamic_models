#include "linear_models/point_mass_linear.h"

int dm_point_mass_linearize(double mass, dm_point_mass_ss_t* out) {
    if (!out || mass <= 0.0) {
        return -1;
    }

    const double inv_mass = 1.0 / mass;

    out->A[0][0] = 0.0;
    out->A[0][1] = 1.0;
    out->A[1][0] = 0.0;
    out->A[1][1] = 0.0;

    out->B[0][0] = 0.0;
    out->B[1][0] = inv_mass;

    out->C[0][0] = 1.0;
    out->C[0][1] = 0.0;
    out->C[1][0] = 0.0;
    out->C[1][1] = 1.0;

    out->D[0][0] = 0.0;
    out->D[1][0] = 0.0;

    return 0;
}
