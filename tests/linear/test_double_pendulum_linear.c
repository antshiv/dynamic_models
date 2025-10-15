#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "linear_models/double_pendulum_linear.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

int main(void) {
    const dm_double_pendulum_params_t params = {
        .mass1 = 1.0,
        .mass2 = 0.8,
        .length1 = 0.6,
        .length2 = 0.4,
        .gravity = 9.81,
        .damping1 = 0.05,
        .damping2 = 0.02,
    };

    dm_double_pendulum_ss_t ss;
    int status = dm_double_pendulum_linearize(&params, &ss);
    assert(status == 0);

    const double m1 = params.mass1;
    const double m2 = params.mass2;
    const double L1 = params.length1;
    const double L2 = params.length2;
    const double g = params.gravity;
    const double c1 = params.damping1;
    const double c2 = params.damping2;

    const double M11 = (m1 + m2) * L1 * L1;
    const double M12 = m2 * L1 * L2;
    const double M22 = m2 * L2 * L2;
    const double det = M11 * M22 - M12 * M12;
    const double inv_det = 1.0 / det;
    const double Minv11 =  M22 * inv_det;
    const double Minv12 = -M12 * inv_det;
    const double Minv21 = -M12 * inv_det;
    const double Minv22 =  M11 * inv_det;

    const double G1 = (m1 + m2) * g * L1;
    const double G2 = m2 * g * L2;

    const double Atheta11 = -(Minv11 * G1);
    const double Atheta12 = -(Minv12 * G2);
    const double Atheta21 = -(Minv21 * G1);
    const double Atheta22 = -(Minv22 * G2);

    const double Aomega11 = -(Minv11 * c1);
    const double Aomega12 = 0.0;
    const double Aomega21 = -(Minv21 * c1);
    const double Aomega22 = -(Minv22 * c2);

    const double B11 = Minv11;
    const double B12 = Minv12;
    const double B21 = Minv21;
    const double B22 = Minv22;

    printf("Double pendulum linearization (downward equilibrium)\n");
    printf("A(2,0)=%.6f expected %.6f\n", ss.A[2][0], Atheta11);

    assert(nearly_equal(ss.A[0][2], 1.0, 1e-12));
    assert(nearly_equal(ss.A[1][3], 1.0, 1e-12));

    assert(nearly_equal(ss.A[2][0], Atheta11, 1e-9));
    assert(nearly_equal(ss.A[2][1], Atheta12, 1e-9));
    assert(nearly_equal(ss.A[2][2], Aomega11, 1e-9));
    assert(nearly_equal(ss.A[2][3], Aomega12, 1e-12));
    assert(nearly_equal(ss.B[2][0], B11, 1e-9));
    assert(nearly_equal(ss.B[2][1], B12, 1e-9));

    assert(nearly_equal(ss.A[3][0], Atheta21, 1e-9));
    assert(nearly_equal(ss.A[3][1], Atheta22, 1e-9));
    assert(nearly_equal(ss.A[3][2], Aomega21, 1e-9));
    assert(nearly_equal(ss.A[3][3], Aomega22, 1e-9));
    assert(nearly_equal(ss.B[3][0], B21, 1e-9));
    assert(nearly_equal(ss.B[3][1], B22, 1e-9));

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
        assert(nearly_equal(ss.x_eq[i], 0.0, 1e-12));
    }

    assert(dm_double_pendulum_linearize(NULL, &ss) == -1);
    assert(dm_double_pendulum_linearize(&params, NULL) == -1);
    dm_double_pendulum_params_t bad = params;
    bad.mass1 = 0.0;
    assert(dm_double_pendulum_linearize(&bad, &ss) == -1);

    return 0;
}
