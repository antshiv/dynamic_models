#include "linear_models/spring_pendulum_linear.h"

#include <stddef.h>

int dm_spring_pendulum_linearize(const dm_spring_pendulum_params_t* params,
                                 dm_spring_pendulum_ss_t* out) {
    if (!params || !out) {
        return -1;
    }

    const double m = params->mass;
    const double k = params->stiffness;
    const double c = params->damping;
    const double l0 = params->rest_length;
    const double g = params->gravity;

    if (m <= 0.0 || k <= 0.0 || l0 <= 0.0 || g < 0.0) {
        return -1;
    }

    /* Static equilibrium: x = 0, y = -r_eq, velocities = 0 */
    const double r_eq = l0 + (m * g) / k;
    const double y_eq = -r_eq;

    /* Linearized stiffness terms */
    const double coeff_x = -k * (1.0 - l0 / r_eq) / m;
    const double coeff_y = -k / m;
    const double damping_term = -c / m;

    /* Initialize matrices to zero */
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out->A[i][j] = 0.0;
            out->C[i][j] = 0.0;
        }
        for (size_t j = 0; j < 2; ++j) {
            out->B[i][j] = 0.0;
            out->D[i][j] = 0.0;
        }
    }

    /* A matrix */
    out->A[0][2] = 1.0;
    out->A[1][3] = 1.0;
    out->A[2][0] = coeff_x;
    out->A[2][2] = damping_term;
    out->A[3][1] = coeff_y;
    out->A[3][3] = damping_term;

    /* B matrix: external force inputs along x and y */
    out->B[2][0] = 1.0 / m;
    out->B[3][1] = 1.0 / m;

    /* C = identity, D = 0 */
    for (size_t i = 0; i < 4; ++i) {
        out->C[i][i] = 1.0;
    }

    out->x_eq[0] = 0.0;
    out->x_eq[1] = y_eq;
    out->x_eq[2] = 0.0;
    out->x_eq[3] = 0.0;

    return 0;
}
