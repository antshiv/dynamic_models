#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "linear_models/spring_pendulum_linear.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

int main(void) {
    const dm_spring_pendulum_params_t params = {
        .mass = 0.5,
        .stiffness = 15.0,
        .damping = 0.3,
        .rest_length = 0.5,
        .gravity = 9.81,
    };

    dm_spring_pendulum_ss_t ss;
    int status = dm_spring_pendulum_linearize(&params, &ss);
    assert(status == 0);

    const double m = params.mass;
    const double k = params.stiffness;
    const double c = params.damping;
    const double l0 = params.rest_length;
    const double g = params.gravity;
    const double r_eq = l0 + (m * g) / k;

    const double expected_coeff_x = -k * (1.0 - l0 / r_eq) / m;
    const double expected_coeff_y = -k / m;
    const double expected_damping = -c / m;

    printf("Spring pendulum linearization around hanging equilibrium\n");
    printf("r_eq = %.6f m, y_eq = %.6f m\n", r_eq, ss.x_eq[1]);
    printf("A(2,0) = %.6f, expected %.6f\n", ss.A[2][0], expected_coeff_x);
    printf("A(3,1) = %.6f, expected %.6f\n", ss.A[3][1], expected_coeff_y);

    assert(nearly_equal(ss.x_eq[0], 0.0, 1e-12));
    assert(nearly_equal(ss.x_eq[1], -r_eq, 1e-12));
    assert(nearly_equal(ss.x_eq[2], 0.0, 1e-12));
    assert(nearly_equal(ss.x_eq[3], 0.0, 1e-12));

    assert(nearly_equal(ss.A[0][2], 1.0, 1e-12));
    assert(nearly_equal(ss.A[1][3], 1.0, 1e-12));
    assert(nearly_equal(ss.A[2][0], expected_coeff_x, 1e-9));
    assert(nearly_equal(ss.A[3][1], expected_coeff_y, 1e-12));
    assert(nearly_equal(ss.A[2][2], expected_damping, 1e-12));
    assert(nearly_equal(ss.A[3][3], expected_damping, 1e-12));

    assert(nearly_equal(ss.B[2][0], 1.0 / m, 1e-12));
    assert(nearly_equal(ss.B[3][1], 1.0 / m, 1e-12));

    for (int i = 0; i < 4; ++i) {
        assert(nearly_equal(ss.C[i][i], 1.0, 1e-12));
        for (int j = 0; j < 4; ++j) {
            if (i != j) {
                assert(nearly_equal(ss.C[i][j], 0.0, 1e-12));
            }
        }
        for (int j = 0; j < 2; ++j) {
            assert(nearly_equal(ss.D[i][j], 0.0, 1e-12));
        }
    }

    dm_spring_pendulum_ss_t invalid;
    assert(dm_spring_pendulum_linearize(NULL, &invalid) == -1);
    assert(dm_spring_pendulum_linearize(&params, NULL) == -1);

    dm_spring_pendulum_params_t bad_params = params;
    bad_params.mass = 0.0;
    assert(dm_spring_pendulum_linearize(&bad_params, &invalid) == -1);

    return 0;
}
