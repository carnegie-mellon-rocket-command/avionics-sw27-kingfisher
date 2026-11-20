/* kalman.c -- see kalman.h
 *
 * Each step has two phases:
 *   PREDICT: push the state forward with the physics model   x = F x,   P = F P F^T + Q
 *   UPDATE : blend in the new measurements                   K = P H^T (H P H^T + R)^-1
 *                                                             x = x + K (z - H x),  P = (I - K H) P
 * H picks out altitude (state 0) and acceleration (state 2), so the matrix algebra below is written
 * out for that specific H instead of a general matrix library. The only inverse is a 2x2.
 */
#include "kalman.h"
#include "ats_config.h"
#include <math.h>

void kalman_init(kalman_t *kf, float alt0, float acc0)
{
    kf->x[0] = alt0;
    kf->x[1] = 0.0f;
    kf->x[2] = acc0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
}

void kalman_update(kalman_t *kf, float dt, float alt_meas, float acc_meas)
{
    /* ---- PREDICT ---- */
    const float F[3][3] = { {1.0f, dt,   dt * dt / 2.0f},
                            {0.0f, 1.0f, dt},
                            {0.0f, 0.0f, 1.0f} };
    const float Q[3] = { KF_Q_POS * KF_Q_POS, KF_Q_VEL * KF_Q_VEL, KF_Q_ACC * KF_Q_ACC };  /* diagonal */

    float xn[3], FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; i++) {
        xn[i] = 0.0f;
        for (int j = 0; j < 3; j++) xn[i] += F[i][j] * kf->x[j];
    }
    for (int i = 0; i < 3; i++)                       /* FP = F * P */
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * kf->P[k][j];
        }
    for (int i = 0; i < 3; i++)                       /* Pn = FP * F^T + Q */
        for (int j = 0; j < 3; j++) {
            Pn[i][j] = (i == j) ? Q[i] : 0.0f;
            for (int k = 0; k < 3; k++) Pn[i][j] += FP[i][k] * F[j][k];
        }

    /* ---- UPDATE ---- */
    float y0 = alt_meas - xn[0];                      /* innovation = measurement - prediction */
    float y1 = acc_meas - xn[2];

    /* S = H P H^T + R  (2x2): rows/cols of P for states 0 and 2, plus measurement noise */
    float S00 = Pn[0][0] + KF_ALT_NOISE * KF_ALT_NOISE;
    float S01 = Pn[0][2];
    float S10 = Pn[2][0];
    float S11 = Pn[2][2] + KF_IMU_NOISE * KF_IMU_NOISE;
    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-9f) {                         /* degenerate: keep the prediction only */
        for (int i = 0; i < 3; i++) kf->x[i] = xn[i];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) kf->P[i][j] = Pn[i][j];
        return;
    }
    float i00 =  S11 / det, i01 = -S01 / det;         /* S^-1 */
    float i10 = -S10 / det, i11 =  S00 / det;

    float K[3][2];                                    /* K = P H^T S^-1, and P H^T = columns 0 and 2 of P */
    for (int i = 0; i < 3; i++) {
        K[i][0] = Pn[i][0] * i00 + Pn[i][2] * i10;
        K[i][1] = Pn[i][0] * i01 + Pn[i][2] * i11;
    }

    for (int i = 0; i < 3; i++)
        kf->x[i] = xn[i] + K[i][0] * y0 + K[i][1] * y1;

    for (int i = 0; i < 3; i++)                       /* P = (I - K H) P */
        for (int j = 0; j < 3; j++)
            kf->P[i][j] = Pn[i][j] - K[i][0] * Pn[0][j] - K[i][1] * Pn[2][j];
}
