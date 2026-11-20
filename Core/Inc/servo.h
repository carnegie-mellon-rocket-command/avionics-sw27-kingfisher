/* servo.h -- hobby-servo driver on TIM2 channel 3 (PA2 = ATS_PWM). Replaces Arduino's Servo.h.
 *
 * Needs TIM2 CH3 set to "PWM Generation" in CubeMX. servo_init() sets the timer to 1 us ticks and a
 * 20 ms (50 Hz) period itself, so it doesn't matter what clock the CubeMX project ended up with.
 * "Attached" = PWM running (servo powered and holding position); "detached" = PWM stopped.
 */
#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include <stdbool.h>

void servo_init(TIM_HandleTypeDef *htim, uint32_t channel);
void servo_attach(void);                 /* start PWM at the last commanded angle */
void servo_detach(void);                 /* stop PWM (saves power) */
bool servo_attached(void);
void servo_write_degrees(float deg);     /* 0..180, clamped; same scale as Arduino Servo.write() */

#endif
