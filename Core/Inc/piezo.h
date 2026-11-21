#ifndef PIEZO_H
#define PIEZO_H

#include <stdbool.h>

typedef enum {
    PIEZO_EVENT_LAUNCH,
    PIEZO_EVENT_BURNOUT_TIMER,
    PIEZO_EVENT_RETRACTION_TIMER,
    PIEZO_EVENT_LANDING
} piezo_event_t;

void piezo_init(void);
void piezo_set_ready(bool ready);
void piezo_set_fault(void);
void piezo_set_landed(void);
void piezo_play_startup_chime(void);
void piezo_play_event(piezo_event_t event);
void piezo_update(void);

#endif
