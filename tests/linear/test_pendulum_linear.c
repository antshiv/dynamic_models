#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "linear_models/pendulum_linear.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

int main(void) {
    const dm_pendulum_params_t params = {
        .mass = 0.8,
        .length = 0.7,
        .gravity = 9.81,
        .damping = 0.05,
    };

    dm_pendulum_ss_t ss;
    int status = dm_pendulum_linearize(&params, &ss);
    assert(status == 0);

    const double inv_inertia = 1.0 / (params.mass * params.length * params.length);
    const double expected_a10 = -params.gravity / params.length;
    const double expected_a11 = -params.damping * inv_inertia;
    const double expected_b1 = inv_inertia;

    printf("Simple pendulum linearization about downward equilibrium\n");
    printf("A = [[%.1f, %.1f], [%.3f, %.6f]]\n",
           ss.A[0][0], ss.A[0][1], ss.A[1][0], ss.A[1][1]);

    assert(nearly_equal(ss.A[0][0], 0.0, 1e-12));
    assert(nearly_equal(ss.A[0][1], 1.0, 1e-12));
    assert(nearly_equal(ss.A[1][0], expected_a10, 1e-12));
    assert(nearly_equal(ss.A[1][1], expected_a11, 1e-12));

    assert(nearly_equal(ss.B[0][0], 0.0, 1e-12));
    assert(nearly_equal(ss.B[1][0], expected_b1, 1e-12));

    assert(nearly_equal(ss.C[0][0], 1.0, 1e-12));
    assert(nearly_equal(ss.C[1][1], 1.0, 1e-12));
    assert(nearly_equal(ss.C[0][1], 0.0, 1e-12));
    assert(nearly_equal(ss.C[1][0], 0.0, 1e-12));

    assert(nearly_equal(ss.D[0][0], 0.0, 1e-12));
    assert(nearly_equal(ss.D[1][0], 0.0, 1e-12));

    assert(nearly_equal(ss.x_eq[0], 0.0, 1e-12));
    assert(nearly_equal(ss.x_eq[1], 0.0, 1e-12));

    dm_pendulum_params_t invalid = params;
    invalid.mass = 0.0;
    assert(dm_pendulum_linearize(&invalid, &ss) == -1);
    invalid = params;
    invalid.length = 0.0;
    assert(dm_pendulum_linearize(&invalid, &ss) == -1);
    assert(dm_pendulum_linearize(NULL, &ss) == -1);
    assert(dm_pendulum_linearize(&params, NULL) == -1);

    return 0;
}
