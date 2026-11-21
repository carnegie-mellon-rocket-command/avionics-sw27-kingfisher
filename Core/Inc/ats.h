/* ats.h -- public interface of kingfisher ATS.
 *
 * PROJECT STRUCTURE (read this first)
 *   main.c / main.h   CubeMX-generated: clocks, pins, SPI1/I2C1/TIM2 setup. We only add two calls (below).
 *   ats.c             THE PROGRAM: startup checks, the 30 ms flight loop, launch/landing detection, ATS control.
 *   ats_config.h      all tunable constants + build switches (ERASE_ON_BOOT, DEBUG, ...)
 *   imu.c             LSM6DSOX accelerometer/gyro (uses ST's lsm6dsox_reg.c driver)
 *   ms5611.c          MS5611 barometer over SPI -> pressure, altitude
 *   kalman.c          Kalman filter -> filtered altitude / velocity / acceleration
 *   servo.c           PWM driver for the flap servo
 *   flashlog.c        writes flight data to the on-board flash, and dumps it as CSV afterwards
 *
 * main.c needs only:   #include "ats.h"  /  ats_init();  before the while loop  /  ats_run_once();  inside it.
 *
 * STATUS LED (D1): slow heartbeat = checking/not armed; solid = verified and ready.
 *   Fast continuous blink = fault/not armed. Slow steady blink = landed/data saved.
 */
#ifndef ATS_H
#define ATS_H

/* Initialise sensors, flash, filter and servo; self-test the flaps. Call once before the main loop. */
void ats_init(void);

/* One 30 ms tick: read sensors, filter, detect launch, drive the flaps, log. Call forever from main's while(1). */
void ats_run_once(void);

#endif
