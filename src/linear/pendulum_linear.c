#include "linear_models/pendulum_linear.h"

#include <stddef.h>

int dm_pendulum_linearize(const dm_pendulum_params_t* params,
                          dm_pendulum_ss_t* out) {
    if (!params || !out) {
        return -1;
    }

    const double m = params->mass;
    const double L = params->length;
    const double g = params->gravity;
    const double c = params->damping;

    if (m <= 0.0 || L <= 0.0 || g < 0.0 || c < 0.0) {
        return -1;
    }

    const double inv_inertia = 1.0 / (m * L * L);
    const double a_theta = -g / L;
    const double a_omega = -c * inv_inertia;
    const double b_omega = inv_inertia;

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
    out->A[1][0] = a_theta;
    out->A[1][1] = a_omega;
    out->B[1][0] = b_omega;

    out->C[0][0] = 1.0;
    out->C[1][1] = 1.0;

    return 0;
}
