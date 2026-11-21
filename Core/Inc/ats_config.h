/* ats_config.h -- every tunable constant for ATS in one place.
 *
 * Units are IPS (inches/pounds/seconds -> feet, lbs, s) unless noted.
 * GRAVITY lives in imu.h; METERS_TO_FEET and SEA_LEVEL_PRESSURE_HPA live in ms5611.h.
 */
#ifndef ATS_CONFIG_H
#define ATS_CONFIG_H

#include "main.h"
#include "imu.h"
#include "ms5611.h"

/* ***************** META (build switches) ***************** */
#ifdef DEBUG
#undef DEBUG
#endif
/* 1 = enable SWO diagnostics such as periodic IMU measurements. */
#define DEBUG               1
/* 1 = print saved CSV files over SWO at startup, then stop before arming. */
#define DUMP_MODE           0
/* 1 = play piezo startup, status, event and fault sounds; 0 = silent. */
#define ENABLE_PIEZO        0
/* 1 = no servo is mounted: skip all servo calls (bench testing without the flap assembly) */
#define SKIP_ATS            0
/* 1 = format littlefs at boot. Destroys files. Use once, then restore 0 and reflash. */
#define ERASE_ON_BOOT       0
/* 1 = log from power-on instead of from launch detection (handy for bench tests; fills flash ~2 KB/s) */
#define LOG_BEFORE_LAUNCH   0

/* ***************** ROCKET CONSTANTS ***************** */
#define ATMOSPHERE_FLUID_DENSITY      0.06970f      /* lbs/ft^3 */
#define ROCKET_DRAG_COEFFICIENT       0.6621875f          /* Average value from OpenRocket */
#define ROCKET_CROSS_SECTIONAL_AREA   0.0530286193821f  /* ft^2 facing upwards */
#define ROCKET_MASS                   11.5475f      /* lbs burnout mass */
#define MAX_FLAP_SURFACE_AREA         0.0593888889f  /* ft^2 */
/* Max surface area (ft^2) with flaps extended, including the rocket's own area.
 * (Last year's macro had no parentheses, which silently broke the predicted-altitude math.) */
#define ATS_MAX_SURFACE_AREA          (MAX_FLAP_SURFACE_AREA + ROCKET_CROSS_SECTIONAL_AREA)

/* ***************** FLIGHT PARAMETERS ***************** */
#define LOOP_TARGET_MS      30u      /* sample period: how often data is collected (ms) */
#define ALT_TARGET      3100.0f      /* ft above launch pad. Sims say ~4250-4500 ft; aiming 3600-3800 after added mass/flap efficacy */
#define ACCEL_THRESHOLD     (3.0f * GRAVITY)  /* launch detection: acceleration above this (ft/s^2). Was 2 g once -- tuned last year */
#define LAUNCH_CONFIRM_SAMPLES  3u      /* require this many consecutive valid over-threshold samples */
#define IMU_DEBUG_PERIOD_MS   250u      /* SWO measurement print period; integer units avoid float-printf setup */

/* Landing detection: "still" = |accel - g| < ACCEL_TOL and |velocity| < VEL_TOL for HOLD_MS in a row */
#define LANDING_ACCEL_TOL   2.0f     /* ft/s^2 */
#define LANDING_VEL_TOL     0.1f     /* ft/s */
#define LANDING_HOLD_MS     5000u    /* how long the rocket must be still */
#define LANDING_CUTOFF_MS   300000u  /* mark as landed this long after launch no matter what (DEF_cutoff_landing_time) */

/* ***************** ATS TIMING (ms after launch) ***************** */
#define ATS_ACTIVE_AFTER_MS   4500u  /* flaps do nothing before this (motor burn) */
#define ATS_WIGGLE_OUT_MS     5000u  /* 4500-5000 IN, 5000-5500 OUT, 5500-6000 IN: self-test wiggle */
#define ATS_WIGGLE_BACK_MS    5500u
#define ATS_WIGGLE_END_MS     6000u  /* after this, the controller drives the flaps */
#define ATS_RETRACT_AFTER_MS  18000u /* fully retract flaps after this */

/* ***************** PID ***************** */
#define ATS_KP   0.03f   /* higher = stronger immediate response */
#define ATS_KD   0.0f    /* adjusts response based on how fast the error changes */

/* ***************** SERVO ***************** */
/* Same mapping as the Arduino Servo library: 0..180 "degrees" -> 544..2400 us pulse. */
#define SERVO_MIN_PULSE_US  544
#define SERVO_MAX_PULSE_US  2400
#define ATS_MIN   180.0f    /* servo angle when percent = 1.0 (flaps OUT) -- names are historical, don't "fix" */
#define ATS_MAX   13.0f     /* servo angle when percent = 0.0 (flaps IN): 254 constraint from flaps / 270 (servo max) * 180 */
#define ATS_IN    1.0f      /* flaps fully retracted */
#define ATS_OUT   0.0f      /* flaps fully extended */

/* ***************** KALMAN FILTER ***************** */
/* States: [altitude (ft), velocity (ft/s), acceleration (ft/s^2)]. Observations: [altitude, acceleration]. */
#define KF_ALT_NOISE   1.0f   /* altimeter noise (TODO: change) */
#define KF_IMU_NOISE   1.0f   /* IMU noise (TODO: change) */
#define KF_Q_POS       0.1f   /* model covariance (TODO: change) */
#define KF_Q_VEL       0.1f
#define KF_Q_ACC       0.8f

/* ***************** BOARD I/O ***************** */
/* Status LED = D1 on the SPARE net (PA1). (LED2 on MCU_STATUS isn't wired to the MCU on this PCB revision.) */
#define STATUS_LED_PORT   SPARE_GPIO_Port
#define STATUS_LED_PIN    SPARE_Pin

#endif
