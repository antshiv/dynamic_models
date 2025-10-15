#include "linear_models/inverted_pendulum_linear.h"

#include <stddef.h>

int dm_inverted_pendulum_linearize(const dm_inverted_pendulum_params_t* params,
                                   dm_inverted_pendulum_ss_t* out) {
    if (!params || !out) {
        return -1;
    }

    const double M = params->mass_cart;
    const double m = params->mass_pole;
    const double l = params->length;
    const double g = params->gravity;
    const double b_cart = params->damping_cart;
    const double b_pole = params->damping_pole;

    if (M <= 0.0 || m <= 0.0 || l <= 0.0 || g < 0.0) {
        return -1;
    }

    const double total_mass = M + m;
    const double pole_mass_length = m * l;
    const double denominator = l * (4.0 / 3.0 - (m / total_mass));

    if (denominator == 0.0) {
        return -1;
    }

    const double a_theta_theta = g / denominator;
    const double a_theta_xdot = b_cart / (denominator * total_mass);
    const double a_theta_thetadot = -b_pole / (denominator * pole_mass_length);
    const double b_theta = -1.0 / (denominator * total_mass);

    const double coeff = pole_mass_length / total_mass;

    const double a_x_theta = -coeff * a_theta_theta;
    const double a_x_xdot = -(b_cart / total_mass) - coeff * a_theta_xdot;
    const double a_x_thetadot = -coeff * a_theta_thetadot;
    const double b_x = (1.0 / total_mass) - coeff * b_theta;

    /* Zero matrices */
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out->A[i][j] = 0.0;
            out->C[i][j] = 0.0;
        }
        out->B[i][0] = 0.0;
        out->D[i][0] = 0.0;
        out->x_eq[i] = 0.0;
    }

    out->A[0][2] = 1.0;
    out->A[1][3] = 1.0;

    out->A[2][0] = a_x_theta;
    out->A[2][2] = a_x_xdot;
    out->A[2][3] = a_x_thetadot;
    out->B[2][0] = b_x;

    out->A[3][0] = a_theta_theta;
    out->A[3][2] = a_theta_xdot;
    out->A[3][3] = a_theta_thetadot;
    out->B[3][0] = b_theta;

    for (size_t i = 0; i < 4; ++i) {
        out->C[i][i] = 1.0;
    }

    out->u_eq = 0.0;
    return 0;
}
