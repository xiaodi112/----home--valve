#ifndef CH4_ALARM_H
#define CH4_ALARM_H

#include "ch4_sensor.h"

/* alarm level (platform protocol: 0=normal 1=alarm) */
typedef enum {
    CH4_ALARM_NORMAL = 0,
    CH4_ALARM_ALARM = 1
} CH4_AlarmLevel_t;

/* internal sub-state machine states */
typedef enum {
    CH4_STATE_IDLE = 0,
    CH4_STATE_POWER_ON,
    CH4_STATE_WARMING,
    CH4_STATE_COLLECT,
    CH4_STATE_REPORT,
    CH4_STATE_POWER_OFF
} CH4_InternalState_t;

/* state machine entry (called from main.c STATE_CH4) */
void do_ch4(void);

/* threshold check */
CH4_AlarmLevel_t ch4_check_alarm(const CH4_Data_t *data);

#endif /* CH4_ALARM_H */
