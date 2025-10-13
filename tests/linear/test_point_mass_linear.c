#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "linear_models/point_mass_linear.h"

static int nearly_equal(double a, double b) {
    return fabs(a - b) <= 1e-12;
}

int main(void) {
    dm_point_mass_ss_t ss;
    const double mass = 1.2;
    const double expected_inv_mass = 1.0 / mass;

    int status = dm_point_mass_linearize(mass, &ss);
    assert(status == 0);

    printf("Point mass linear model around constant thrust equilibrium (mass=%.3f kg)\n", mass);
    printf("A = [[%.1f, %.1f],[%.1f, %.1f]]\n",
           ss.A[0][0], ss.A[0][1], ss.A[1][0], ss.A[1][1]);
    printf("B = [[%.1f],[%.6f]]\n", ss.B[0][0], ss.B[1][0]);

    assert(nearly_equal(ss.A[0][0], 0.0));
    assert(nearly_equal(ss.A[0][1], 1.0));
    assert(nearly_equal(ss.A[1][0], 0.0));
    assert(nearly_equal(ss.A[1][1], 0.0));
    assert(nearly_equal(ss.B[0][0], 0.0));
    assert(nearly_equal(ss.B[1][0], expected_inv_mass));

    assert(nearly_equal(ss.C[0][0], 1.0));
    assert(nearly_equal(ss.C[0][1], 0.0));
    assert(nearly_equal(ss.C[1][0], 0.0));
    assert(nearly_equal(ss.C[1][1], 1.0));
    assert(nearly_equal(ss.D[0][0], 0.0));
    assert(nearly_equal(ss.D[1][0], 0.0));

    status = dm_point_mass_linearize(-1.0, &ss);
    assert(status == -1);
    status = dm_point_mass_linearize(1.0, NULL);
    assert(status == -1);

    return 0;
}
