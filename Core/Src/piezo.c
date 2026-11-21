#include "piezo.h"
#include "main.h"

/* PA0 is TIM5_CH1 on STM32F413. Use TIM5 so piezo tone generation cannot
 * change TIM2's 50 Hz servo period. The board's piezo is a passive transducer. */
#define PIEZO_TONE_HZ          4000u
#define PIEZO_SHORT_MS         180u
#define PIEZO_LONG_MS          900u
#define PIEZO_INTER_BEEP_MS    180u
#define PIEZO_PAIR_PAUSE_MS    700u
#define PIEZO_SIREN_LOW_HZ     3500u
#define PIEZO_SIREN_HIGH_HZ    4500u
#define PIEZO_SIREN_SWEEP_MS   900u
#define PIEZO_STARTUP_CHIME_MS 1250u
#define PIEZO_SIREN_PAUSE_MS   900u
#define PIEZO_SIREN_COUNT      3u

typedef enum { MODE_SILENT, MODE_READY, MODE_SIREN, MODE_STARTUP } piezo_mode_t;

static TIM_HandleTypeDef htim5;
static piezo_mode_t base_mode = MODE_SILENT;
static bool event_active;
static uint32_t event_until;
static uint32_t pattern_start;
static bool tone_on;
static uint32_t tone_hz = PIEZO_TONE_HZ;

static void piezo_set_frequency(uint32_t hz)
{
    if (hz < 500u) hz = 500u;
    uint32_t period = 16000000u / hz;
    if (period == 0u) return;
    __HAL_TIM_SET_AUTORELOAD(&htim5, period - 1u);
    htim5.Init.Period = period - 1u;
    tone_hz = hz;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, tone_on ? period / 2u : 0u);
    __HAL_TIM_SET_COUNTER(&htim5, 0u);
}

static void piezo_tone(bool on)
{
    if (tone_on == on) return;
    tone_on = on;
    uint32_t period = htim5.Init.Period + 1u;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, on ? period / 2u : 0u);
}

void piezo_init(void)
{
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = (16000000u / PIEZO_TONE_HZ) - 1u;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef channel = {0};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &channel, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim5);
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    base_mode = MODE_SILENT;
    event_active = false;
    tone_on = false;
}

void piezo_set_ready(bool ready)
{
    base_mode = ready ? MODE_READY : MODE_SILENT;
    pattern_start = HAL_GetTick();
    event_active = false;
    piezo_tone(false);
}

void piezo_set_fault(void)
{
    base_mode = MODE_SIREN;
    pattern_start = HAL_GetTick();
    event_active = false;
    piezo_tone(false);
}

void piezo_set_landed(void)
{
    base_mode = MODE_SIREN;
    pattern_start = HAL_GetTick();
    event_active = false;
    piezo_tone(false);
}

void piezo_play_startup_chime(void)
{
    base_mode = MODE_STARTUP;
    pattern_start = HAL_GetTick();
    event_active = false;
    piezo_tone(false);
}

void piezo_play_event(piezo_event_t event)
{
    (void)event;
    event_active = true;
    event_until = HAL_GetTick() + PIEZO_LONG_MS;
    piezo_set_frequency(PIEZO_TONE_HZ);
    piezo_tone(true);
}

void piezo_update(void)
{
    uint32_t now = HAL_GetTick();
    if (event_active) {
        if ((int32_t)(now - event_until) < 0) return;
        event_active = false;
        pattern_start = now;
        piezo_tone(false);
    }

    uint32_t phase = now - pattern_start;
    uint32_t cycle;
    uint32_t on_time;
    if (base_mode == MODE_STARTUP) {
        /* Pikmin-like chirp: high, dipping/wobbling gestures, then a sharp upward accent. */
        static const uint32_t note_hz[] = {4400u, 3950u, 3650u, 3900u, 3550u, 4050u, 4500u};
        static const uint16_t note_ms[] = {180u, 120u, 120u, 100u, 120u, 120u, 260u};
        uint32_t note_start = 0u;
        bool note_on = false;
        uint32_t note_frequency = PIEZO_TONE_HZ;
        for (uint32_t i = 0; i < sizeof(note_ms) / sizeof(note_ms[0]); i++) {
            if (phase >= note_start && phase < note_start + note_ms[i]) {
                note_on = true;
                note_frequency = note_hz[i];
                break;
            }
            note_start += note_ms[i] + 30u;
        }
        if (phase >= PIEZO_STARTUP_CHIME_MS) {
            base_mode = MODE_SILENT;
            piezo_tone(false);
        } else {
            if (tone_hz != note_frequency) piezo_set_frequency(note_frequency);
            piezo_tone(note_on);
        }
    } else if (base_mode == MODE_READY) {
        /* BEEP BEEP, pause, BEEP BEEP, pause */
        cycle = 2u * (2u * PIEZO_SHORT_MS + PIEZO_INTER_BEEP_MS + PIEZO_PAIR_PAUSE_MS);
        phase %= cycle;
        phase %= (2u * PIEZO_SHORT_MS + PIEZO_INTER_BEEP_MS + PIEZO_PAIR_PAUSE_MS);
        on_time = (phase < PIEZO_SHORT_MS) ||
                  (phase >= PIEZO_SHORT_MS + PIEZO_INTER_BEEP_MS &&
                   phase < 2u * PIEZO_SHORT_MS + PIEZO_INTER_BEEP_MS);
        piezo_tone(on_time);
    } else if (base_mode == MODE_SIREN) {
        /* Three siren sweeps with a pause after each, then repeat. */
        uint32_t siren_duration = 2u * PIEZO_SIREN_SWEEP_MS;
        uint32_t siren_slot = siren_duration + PIEZO_SIREN_PAUSE_MS;
        cycle = PIEZO_SIREN_COUNT * siren_slot;
        phase %= cycle;
        uint32_t slot_phase = phase % siren_slot;
        on_time = slot_phase < siren_duration;
        if (on_time) {
            uint32_t sweep_phase = slot_phase;
            uint32_t range = PIEZO_SIREN_HIGH_HZ - PIEZO_SIREN_LOW_HZ;
            uint32_t offset = (sweep_phase <= PIEZO_SIREN_SWEEP_MS)
                            ? (range * sweep_phase) / PIEZO_SIREN_SWEEP_MS
                            : (range * (2u * PIEZO_SIREN_SWEEP_MS - sweep_phase)) / PIEZO_SIREN_SWEEP_MS;
            uint32_t hz = PIEZO_SIREN_LOW_HZ + offset;
            if (hz != tone_hz) piezo_set_frequency(hz);
        } else {
            piezo_tone(false);
        }
        if (on_time) piezo_tone(true);
    } else {
        piezo_tone(false);
    }
}
