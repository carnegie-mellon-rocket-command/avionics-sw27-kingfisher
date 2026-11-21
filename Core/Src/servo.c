/* servo.c -- see servo.h */
#include "servo.h"
#include "ats_config.h"

static TIM_HandleTypeDef *tim;
static uint32_t           chan;
static bool               attached = false;
static uint32_t           pulse_us = 1500;   /* last commanded pulse width */

void servo_init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    tim = htim;
    chan = channel;

    /* Work out the timer's input clock (APB1 timers run at 2x PCLK1 when APB1 is divided) */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE1_2) tim_clk *= 2;

    __HAL_TIM_SET_PRESCALER(tim, tim_clk / 1000000u - 1u);   /* 1 tick = 1 us */
    __HAL_TIM_SET_AUTORELOAD(tim, 19999u);                   /* 20 ms period */
    __HAL_TIM_SET_COMPARE(tim, chan, pulse_us);
    HAL_TIM_GenerateEvent(tim, TIM_EVENTSOURCE_UPDATE);      /* load the new prescaler now */
}

bool servo_attach(void)
{
    if (attached) return true;
    __HAL_TIM_SET_COMPARE(tim, chan, pulse_us);
    if (HAL_TIM_PWM_Start(tim, chan) != HAL_OK) return false;
    attached = true;
    return true;
}

void servo_detach(void)
{
    if (!attached) return;
    HAL_TIM_PWM_Stop(tim, chan);
    attached = false;
}

bool servo_attached(void) { return attached; }

void servo_write_degrees(float deg)
{
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    pulse_us = (uint32_t)(SERVO_MIN_PULSE_US + deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180.0f);
    __HAL_TIM_SET_COMPARE(tim, chan, pulse_us);
}
