/**
 * @brief api_level.h
 *
 */

#ifndef API_LEVEL_H
#define API_LEVEL_H

// Needed for timers definition
#include "timer.h"

// Needed for replyable structs
#include "replyables.h"

// Needed for whoop_error_t
#include "common.h"

#include "device_level.h"


// opaque pointer to internal active object
extern QActive * const g_ao_api_level;

typedef enum
{
    API_LEVEL_UNKNOWN      = 0,
    API_LEVEL_DISABLED     = 1,
    API_LEVEL_ENABLED      = 2,
    API_LEVEL_FATAL_ERROR  = 3,

} api_level_status_t;

typedef struct
{
    q_event_replyable_request_t    super;       /**<Extend q_event_replyable_response_t */
} api_level_start_pattern_event_t;

typedef struct
{
    /* inherit: */
    q_event_replyable_response_t super;           /**<Extend q_event_replyable_response_t */
} api_level_response_event_t;

void api_level_ctor(void);

void api_level_start(void);

api_level_status_t api_level_get_status(void);

int32_t api_level_get_last_error(void);

bool api_level_is_busy(void);

timer_count_t api_level_get_active_counts(void);

#endif
