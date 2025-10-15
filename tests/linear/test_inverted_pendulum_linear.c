#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "linear_models/inverted_pendulum_linear.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

int main(void) {
    const dm_inverted_pendulum_params_t params = {
        .mass_cart = 1.0,
        .mass_pole = 0.1,
        .length = 0.5,
        .gravity = 9.81,
        .damping_cart = 0.05,
        .damping_pole = 0.002,
    };

    dm_inverted_pendulum_ss_t ss;

    int status = dm_inverted_pendulum_linearize(&params, &ss);
    assert(status == 0);

    const double M = params.mass_cart;
    const double m = params.mass_pole;
    const double l = params.length;
    const double g = params.gravity;
    const double b_cart = params.damping_cart;
    const double b_pole = params.damping_pole;

    const double total_mass = M + m;
    const double denominator = l * (4.0 / 3.0 - (m / total_mass));
    const double coeff = (m * l) / total_mass;

    const double a_theta_theta = g / denominator;
    const double a_theta_xdot = b_cart / (denominator * total_mass);
    const double a_theta_thetadot = -b_pole / (denominator * m * l);
    const double b_theta = -1.0 / (denominator * total_mass);

    const double a_x_theta = -coeff * a_theta_theta;
    const double a_x_xdot = -(b_cart / total_mass) - coeff * a_theta_xdot;
    const double a_x_thetadot = -coeff * a_theta_thetadot;
    const double b_x = (1.0 / total_mass) - coeff * b_theta;

    printf("Inverted pendulum linearization around upright equilibrium\n");
    printf("A(3,1)=%.6f expected %.6f\n", ss.A[3][0], a_theta_theta);

    assert(nearly_equal(ss.A[0][2], 1.0, 1e-12));
    assert(nearly_equal(ss.A[1][3], 1.0, 1e-12));

    assert(nearly_equal(ss.A[2][0], a_x_theta, 1e-9));
    assert(nearly_equal(ss.A[2][2], a_x_xdot, 1e-12));
    assert(nearly_equal(ss.A[2][3], a_x_thetadot, 1e-12));
    assert(nearly_equal(ss.B[2][0], b_x, 1e-9));

    assert(nearly_equal(ss.A[3][0], a_theta_theta, 1e-9));
    assert(nearly_equal(ss.A[3][2], a_theta_xdot, 1e-12));
    assert(nearly_equal(ss.A[3][3], a_theta_thetadot, 1e-12));
    assert(nearly_equal(ss.B[3][0], b_theta, 1e-9));

    for (int i = 0; i < 4; ++i) {
        assert(nearly_equal(ss.C[i][i], 1.0, 1e-12));
        for (int j = 0; j < 4; ++j) {
            if (i != j) {
                assert(nearly_equal(ss.C[i][j], 0.0, 1e-12));
            }
        }
        assert(nearly_equal(ss.D[i][0], 0.0, 1e-12));
        assert(nearly_equal(ss.x_eq[i], 0.0, 1e-12));
    }
    assert(nearly_equal(ss.u_eq, 0.0, 1e-12));

    assert(dm_inverted_pendulum_linearize(NULL, &ss) == -1);
    assert(dm_inverted_pendulum_linearize(&params, NULL) == -1);
    dm_inverted_pendulum_params_t bad = params;
    bad.mass_cart = 0.0;
    assert(dm_inverted_pendulum_linearize(&bad, &ss) == -1);

    return 0;
}
