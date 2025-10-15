#include "linear_models/double_pendulum_linear.h"

#include <stddef.h>

int dm_double_pendulum_linearize(const dm_double_pendulum_params_t* params,
                                 dm_double_pendulum_ss_t* out) {
    if (!params || !out) {
        return -1;
    }

    const double m1 = params->mass1;
    const double m2 = params->mass2;
    const double L1 = params->length1;
    const double L2 = params->length2;
    const double g = params->gravity;
    const double c1 = params->damping1;
    const double c2 = params->damping2;

    if (m1 <= 0.0 || m2 <= 0.0 || L1 <= 0.0 || L2 <= 0.0 || g < 0.0) {
        return -1;
    }

    const double M11 = (m1 + m2) * L1 * L1;
    const double M12 = m2 * L1 * L2;
    const double M22 = m2 * L2 * L2;

    const double det = M11 * M22 - M12 * M12;
    if (det == 0.0) {
        return -1;
    }

    const double inv_det = 1.0 / det;
    const double Minv11 =  M22 * inv_det;
    const double Minv12 = -M12 * inv_det;
    const double Minv21 = -M12 * inv_det;
    const double Minv22 =  M11 * inv_det;

    const double G1 = (m1 + m2) * g * L1;
    const double G2 = m2 * g * L2;

    const double D1 = c1;
    const double D2 = c2;

    const double Atheta11 = -(Minv11 * G1);
    const double Atheta12 = -(Minv12 * G2);
    const double Atheta21 = -(Minv21 * G1);
    const double Atheta22 = -(Minv22 * G2);

    const double Aomega11 = -(Minv11 * D1);
    const double Aomega12 = 0.0;
    const double Aomega21 = -(Minv21 * D1);
    const double Aomega22 = -(Minv22 * D2);

    const double B11 = Minv11;
    const double B12 = Minv12;
    const double B21 = Minv21;
    const double B22 = Minv22;

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out->A[i][j] = 0.0;
            out->C[i][j] = 0.0;
        }
        for (size_t j = 0; j < 2; ++j) {
            out->B[i][j] = 0.0;
            out->D[i][j] = 0.0;
        }
        out->x_eq[i] = 0.0;
    }

    out->A[0][2] = 1.0;
    out->A[1][3] = 1.0;

    out->A[2][0] = Atheta11;
    out->A[2][1] = Atheta12;
    out->A[2][2] = Aomega11;
    out->A[2][3] = Aomega12;
    out->B[2][0] = B11;
    out->B[2][1] = B12;

    out->A[3][0] = Atheta21;
    out->A[3][1] = Atheta22;
    out->A[3][2] = Aomega21;
    out->A[3][3] = Aomega22;
    out->B[3][0] = B21;
    out->B[3][1] = B22;

    for (size_t i = 0; i < 4; ++i) {
        out->C[i][i] = 1.0;
    }

    return 0;
}
