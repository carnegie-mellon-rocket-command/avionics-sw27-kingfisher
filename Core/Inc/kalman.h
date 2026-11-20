/* kalman.h -- small hand-written Kalman filter (non-extended), 3 states / 2 observations.
 *
 * Replaces last year's KALMAN<3,2> library. Plain float arrays, no dynamic memory, no linalg library.
 *   state x = [altitude (ft), velocity (ft/s), acceleration (ft/s^2)]
 *   obs   z = [altitude measurement, acceleration measurement]  (velocity is never measured directly --
 *             it comes out of the filter, which is why it's called "filtered velocity")
 * Model: constant acceleration over one step dt.
 */
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x[3];      /* filtered [altitude, velocity, acceleration] */
    float P[3][3];   /* state covariance (how uncertain each state is) */
} kalman_t;

/* Start the filter at the first real measurements (avoids a big transient from starting at 0). */
void kalman_init(kalman_t *kf, float alt0, float acc0);

/* One filter step. dt = seconds since the last call; alt_meas in ft; acc_meas in ft/s^2.
 * Results are in kf->x[0..2]. */
void kalman_update(kalman_t *kf, float dt, float alt_meas, float acc_meas);

#endif
