/**
 * @file        api_level.c
 * @author      @whoop.com
 * @brief       High level API_LEVEL device driver
 * @details     The API_LEVEL driver is a QP Active Object with a state
 *              machine architected in the following manner:
 *
 *              One starting state:
 *              api_level_initial        -   The initial state as required by QP
 *
 *              One super state:
 *              api_level_backstop       -   Handler for uncaught or error case signals
 *
 *              These states are children of the backstop state:
 *              api_level_disabled       -   Bounces all requests, waits for an enable signal
 *              api_level_starting       -   Have received enable signal, wait for API_LEVEL ready
 *              api_level_enabled        -   API_LEVEL is now ready, signal the supervisor, move to idle
 *              api_level_error          -   Fatal error state
 *
 *              These states are children of the enabled state:
 *              api_level_idle           -   The normal inactive state of the API_LEVEL object
 *              api_level_busy           -   A superstate for while the API_LEVEL I2C is busy
 *                                          Incoming requests while the driver is in the busy
 *                                          state are cached in a queue until the driver is
 *                                          idle again
 *
 
 *
 *
 *
 * @version     0.1
 * @date        2020-06-18
 *
 * @copyright   Copyright (c) 2020
 *
 */

#include "qpc.h"
#include "common.h"
#include "ao_timings.h"
#include "events.h"
#include "signals.h"
#include "whoop_printf.h"
#include "whoop_qp_time.h"

#include "device_level.h"
#include "api_level.h"
#include "device_level.h"
#include "api_level.h"

/**
 *  @brief      define the human-readable name for this module
*/

#define API_LEVEL_NAME             "API_LEVEL"

/**
    @brief define the power-up default debug level threshold for this
    module.
*/
#define STARTING_DEBUG_LEVEL        1u

/**
    @brief define the debug level threshold for DEBUG_OUT calls.
    Maps the DEBUG_LEVEL macro that DEBUG_OUT calls to the local data
    storage that stores the current debug level for this module.
    DEBUG_OUT(N, MSG) will only produce output when N <= DEBUG_LEVEL.
*/
#define DEBUG_LEVEL                 api_level_debug_level   // this must be defined for DEBUG_OUT

static uint32_t api_level_debug_level = STARTING_DEBUG_LEVEL; // Default debug level


/**
    @brief Ensure the AO doesn't wait forever if the device is stuck
    Set to the minimum allowed timeout time.
*/
#define API_LEVEL_INIT_LOCKUP_TIME_MS   1000u
#define API_LEVEL_LOCKUP_TIME_MS        250u

#define API_LEVEL_QUEUE_SIZE            10U

/**
    @brief define the number of transactions that can be queued up
    for future processing while the object is busy processing an earlier
    transaction.

    Once the deferred event queue is full, any additional requested events
    will be discarded (without asserting).
*/
#define API_LEVEL_DEFERRED_QUEUE_SIZE    5u

typedef struct
{
    QActive                 super;
    QActive *               requestor;                  /**< This is the send requestor */
    uint8_t                 request_id;                 /**< The request ID passed to this AO */
    QEQueue                 deferred_event_queue;
    QEvent const *          deferred_events_queue_buf[API_LEVEL_DEFERRED_QUEUE_SIZE];
    QTimeEvt                time_event;                 /**< Timeout timer. */
    QTimeEvt                busy_event;                 /**< Busy timer. */
    api_level_status_t      status;                     
    ao_timings_t            ao_timings;                 /**< Timing data*/
    uint32_t                debug_level;                /**< Current threshold for gating debug output. >*/
} api_level_t;

static QEvt const * api_level_que_sto[API_LEVEL_QUEUE_SIZE];     // api_level queue storage space

// Signals for use in local context only
enum
{
    LOCAL_API_LEVEL_TIMEOUT_SIG = MAX_SIG,  /**< Timeout signal used in several spots >*/
    LOCAL_API_LEVEL_BUSY_TIMEOUT_SIG,
    LOCAL_API_LEVEL_START_INIT_SIG,
    LOCAL_API_LEVEL_RETRY_SIG,
};

// Single instance of the internal api_level object
static api_level_t ao_api_level =
{
    .debug_level                = STARTING_DEBUG_LEVEL,
    .status                     = API_LEVEL_UNKNOWN,
    .last_error                 = E_WHOOP_NO_ERROR,
};

// globally scoped opaque pointer
QActive * const g_ao_api_level = &ao_api_level.super;

// state functions
static QState api_level_initial            (api_level_t * const me, QEvt const * const e);
static QState api_level_backstop           (api_level_t * const me, QEvt const * const e);
static QState api_level_disabled           (api_level_t * const me, QEvt const * const e);
static QState api_level_starting           (api_level_t * const me, QEvt const * const e);
static QState api_level_enabled            (api_level_t * const me, QEvt const * const e);
static QState api_level_idle               (api_level_t * const me, QEvt const * const e);
static QState api_level_busy               (api_level_t * const me, QEvt const * const e);
static QState api_level_error              (api_level_t * const me, QEvt const * const e);

// Private functions
static void api_level_error_response(api_level_t * const me, int32_t error_code,
                                      whoop_error_severity_t error_severity);

static void api_level_publish_status(api_level_t * const me);

/************************************************************************************/
/***    START OF HSM                                                              ***/
/************************************************************************************/

/*! @fn         void api_level_ctor(void)
*   @brief      API_LEVEL active object contructor
*   @details
*   @param[in]  nothing
*   @param[in]  nothing
*   @param[out] nothing
*   @return     nothing
*/
void api_level_ctor(void)
{
    //Create a pointer to the instance of myself
    api_level_t * const me = &ao_api_level;

    //Register AO, and set entry state
    QActive_ctor(&me->super, (QStateHandler)&api_level_initial);

    // Create a timer object for API_LEVEL communications timeout detection
    QTimeEvt_ctorX(&me->time_event,  &me->super, LOCAL_API_LEVEL_TIMEOUT_SIG, 0U);

    // Create a timer object for API_LEVEL busy state timeout detection
    QTimeEvt_ctorX(&me->busy_event,  &me->super, LOCAL_API_LEVEL_BUSY_TIMEOUT_SIG, 0U);

}

/*! @fn         static QState api_level_initial(api_level_t * me, QEvt const * const e)
*   @brief      Initial state as required by QP
*   @details
*   @param[in]  api_level_t     - pointer to instance high level API_LEVEL driver
*   @param[in]  QEvt         - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState       - pointer to the QHsm object
*/
static QState api_level_initial(api_level_t * me, QEvt const * const e)
{
    (void)e;    // avoid compiler warning

    // create object dictionary entries
    QS_OBJ_DICTIONARY(me);
    QS_FUN_DICTIONARY(&api_level_initial);
    QS_FUN_DICTIONARY(&api_level_backstop);
    QS_FUN_DICTIONARY(&api_level_disabled);
    QS_FUN_DICTIONARY(&api_level_starting);
    QS_FUN_DICTIONARY(&api_level_enabled);
    QS_FUN_DICTIONARY(&api_level_idle);
    QS_FUN_DICTIONARY(&api_level_busy);
    QS_FUN_DICTIONARY(&api_level_error);

    // Subscribe to signal from low level driver
    QActive_subscribe(&me->super, DEVICE_LEVEL_DISABLE_REPORT_SIG);
    QActive_subscribe(&me->super, DEVICE_LEVEL_READY_REPORT_SIG);
    QActive_subscribe(&me->super, DEVICE_LEVEL_ERROR_REPORT_SIG);

    // Set the initial status to disabled
    me->status = API_LEVEL_DISABLED;

    // Initialize the timings
    ao_timings_init(&me->ao_timings);

    // Move to the disabled state, and wait for enable signal
    return Q_TRAN(&api_level_disabled);
}

/*! @brief      Backstop handles signals not caught by substates
*   @details
*   @param[in]  api_level_t  -   pointer to instance high level API_LEVEL driver
*   @param[in]  QEvt         -   pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState       - pointer to the QHsm object
*/
QState api_level_backstop(api_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&QHsm_top);

    switch (e->sig)
    {
        case Q_EMPTY_SIG:
        {
            // The Q_EMPTY_SIG is used by Qp to discover the configuration of the active object.
            // it should always call Q_SUPER. In the backstop, we do not want to generate an error
            // when this occurs, so create a special case
            break;
        }

        case Q_INIT_SIG:
        case Q_ENTRY_SIG:
        case Q_EXIT_SIG:
        {
            // Ignore expected case
            status = Q_HANDLED();
            break;
        }

        //Set the local debug level
        case DEBUG_LEVEL_SIG:
        {
            debug_level_event_t * debug_evt = (debug_level_event_t *) e;
            api_level_debug_level = debug_evt->new_debug_level;

            ao_api_level = api_level_debug_level;

            DEBUG_OUT(1u, "%s: Setting debug menu to %u\n", API_LEVEL_NAME, api_level_debug_level);
            status = Q_HANDLED();
            break;
        }

        // Handle this in the backstop since it can happen at any point
        case API_LEVEL_REQ_STATUS_SIG:
        {
            api_level_publish_status(me);
            break;
        }

        // If we receive a request to disable the device, service it here
        case API_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver Disabled.\n", API_LEVEL_NAME);

            // Mark the device status as DISABLED
            me->status = API_LEVEL_DISABLED;

            // Disable the device driver
            static QEvt const dis_evt = {DEVICE_LEVEL_DISABLE_SIG, 0u, 0u};
            QACTIVE_POST(g_ao_device_level, &dis_evt, me);

            // Move to the disabled state
            status = Q_TRAN(&api_level_disabled);
            break;
        }

        case DEVICE_LEVEL_ERROR_REPORT_SIG:
        {
            DEBUG_OUT(1u, "%s: Caught device error report in backstop..\n", API_LEVEL_NAME);

            // Mark the device as disabled
            me->status = API_LEVEL_FATAL_ERROR;

            // Set the last error
            me->last_error = E_WHOOP_API_LEVEL_DEVICE_LEVEL_UNAVAILABLE;

            api_level_error_response(me, E_WHOOP_API_LEVEL_DEVICE_LEVEL_UNAVAILABLE, E_S_WHOOP_ERROR);

            status = Q_TRAN(&api_level_error);
            break;
        }

        // Catch unhandled signals here
        default:
        {
            if (e->sig < MAX_SIG)
            {
                DEBUG_OUT(1u, "%s: Ignoring unhandled signal %s.\n", API_LEVEL_NAME, signals_get_signal_name(e->sig));
            }
            else
            {
                DEBUG_OUT(1u, "%s: Ignoring unhandled local signal at offset %d.\n", API_LEVEL_NAME, (e->sig - MAX_SIG));
            }
            break;
        }
    }
    return status;
}

/*! @brief      Wait for enable signal from supervisor
*   @details    The disabled state waits to receive an 'enable' signal from the supervisor
*               Once this signal is received, the AO will transition to the 'starting' state
*   @param[in]  api_level_t   - pointer to instance of API_LEVEL Active Object
*   @param[in]  QEvt          - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState        - pointer to the QHsm object
*/
QState api_level_disabled(api_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&api_level_backstop);

    switch (e->sig)
    {

        case Q_ENTRY_SIG:
        {
            ao_set_idle(&me->ao_timings);

            // Set the status of the API_LEVEL device to disabled
            me->status = API_LEVEL_DISABLED;
            api_level_publish_status(me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case API_LEVEL_ENABLE_SIG:
        {
            // Once we receive a start signal, move to the 'starting' state
            DEBUG_OUT(1u, "%s: Driver Starting.\n", API_LEVEL_NAME);

            status = Q_TRAN(&api_level_starting);
            break;
        }

        // Ignore a repeated attempt to disable
        case API_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Device already disabled.\n", API_LEVEL_NAME);
            status = Q_HANDLED();
            break;
        }

        default:
        {
            break;
        }
    }
    return status;
}

/*! @brief      Wait for i2c to become available
*   @details    Waits for the I2C bus to become available before transitioning to the 'enabled' state.
*               At this point we have received the 'enable' signal from the supervisor, and therefore
*               also know that the i2c bus AO is running. We then check to confirm that the communication
*               channel is available and working before transitioning to the 'enabled' state
*   @param[in]  api_level_t  - pointer to instance of API_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState api_level_starting(api_level_t * me, QEvt const *e)
{
    QState status = Q_SUPER(&api_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            ao_set_busy(&me->ao_timings);

            QActive_subscribe(&me->super, DEVICE_LEVEL_READY_REPORT_SIG);

            // Self-post the init starting sig
            static QEvt const start_evt = {LOCAL_API_LEVEL_START_INIT_SIG, 0u, 0u};
            QACTIVE_POST(g_ao_api_level, &start_evt, me);

            status = Q_HANDLED();
            break;
        }

        case LOCAL_API_LEVEL_RETRY_SIG:
        case LOCAL_API_LEVEL_START_INIT_SIG:
        {
            // Arm the One-shot timer in case device not ready or unresponsive
            whoop_qp_time_safe_arm(&me->time_event, MS_TO_TICKS(API_LEVEL_INIT_LOCKUP_TIME_MS), 0u);

            // Request i2c bus status from low level driver
            static QEvt const device_level_status_req_evt = {DEVICE_LEVEL_ENABLE_SIG, 0u, 0u};
            QACTIVE_POST(g_ao_device_level, &device_level_status_req_evt, me);

            status = Q_HANDLED();
            break;

        }

        case DEVICE_LEVEL_READY_REPORT_SIG:
        {
            QActive_unsubscribe(&me->super, DEVICE_LEVEL_READY_REPORT_SIG);
            DEBUG_OUT(1u, "%s: Low level driver active. Moving to idle state\n", API_LEVEL_NAME);

            status = Q_TRAN(&api_level_idle);
            break;
        }

        case DEVICE_LEVEL_ERROR_REPORT_SIG:
        {
            DEBUG_OUT(1u, "caught error\n");

            api_level_error_response(me, E_WHOOP_API_LEVEL_DEVICE_LEVEL_UNAVAILABLE, E_S_WHOOP_ERROR);
            DEBUG_OUT(1u, "%s: Low-level driver not available. Enable Failed\n", API_LEVEL_NAME);

            // Mark the device as in error state
            me->status = API_LEVEL_FATAL_ERROR;

            // Set the last error
            me->last_error = E_WHOOP_API_LEVEL_DEVICE_LEVEL_UNAVAILABLE;

            status = Q_TRAN(&api_level_error);
            break;
        }

        case LOCAL_API_LEVEL_TIMEOUT_SIG:
        {
            api_level_error_response(me, E_WHOOP_API_LEVEL_TIMEOUT, E_S_WHOOP_ERROR);
            DEBUG_OUT(1u, "%s: API LEVEL has timed out. Enable Failed\n", API_LEVEL_NAME);

            // Mark the device as errored
            me->status = API_LEVEL_FATAL_ERROR;

            // Set the last error
            me->last_error = E_WHOOP_API_LEVEL_TIMEOUT;
            status = Q_TRAN(&api_level_error);


            break;
        }

        case API_LEVEL_ENABLE_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            // Disable the timeout timer
            QTimeEvt_disarm(&me->time_event);
            status = Q_HANDLED();
            break;
        }

        default:
        {
            break;
        }
    }
    return status;
}

/*! @brief      All subsystems are ready and available
*   @details    Transitions to idle.
*
*   @param[in]  api_level_t  - pointer to instance of API_LEVEL Active Object
*   @param[in]  QEvt          - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState        - pointer to the QHsm object
*/
static QState api_level_enabled(api_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&api_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // disarm lockup detection timer if it hasn't already fired.
            QTimeEvt_disarm(&me->time_event);

            DEBUG_OUT(1u, "%s: Driver Enabled.\n", API_LEVEL_NAME);

            me->status = API_LEVEL_ENABLED;
            api_level_publish_status(me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case API_LEVEL_ENABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver already enabled.\n", API_LEVEL_NAME);
            status = Q_HANDLED();
            break;
        }

        // If we receive a request to disable the device, service it here
        case API_LEVEL_DISABLE_SIG:
        {
            // Mark the device status as DISABLED
            me->status = API_LEVEL_DISABLED;

            DEBUG_OUT(1u, "%s: Driver Disabled\n", API_LEVEL_NAME);

            // Move to the disabled state
            status = Q_TRAN(&api_level_disabled);
            break;
        }

        default:
        {
            break;
        }
    }

    return status;
}

/*! @brief      API_LEVEL normal idle state
*   @details    Idle waits for actionable signals

*   @param[in]  api_level_t     -   pointer to instance high level API_LEVEL driver
*   @param[in]  QEvt             -   pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState           - pointer to the QHsm object
*/
QState api_level_idle(api_level_t * me, QEvt const * const e)
{
    QState status = Q_SUPER(&api_level_enabled);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Mark the API_LEVEL driver as enabled
            ao_set_idle(&me->ao_timings);
            me->status = API_LEVEL_ENABLED;

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        default:
        {
            break;
        }
    }

    return status;
}

/** @brief      API_LEVEL busy state
*   @details    Busy queues any requests received while the API_LEVEL is waiting for a response
*               from the device.
*   @param[in]  api_level_t     -   pointer to instance high level API_LEVEL driver
*   @param[in]  QEvt         -   pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState       - pointer to the QHsm object
*/
QState api_level_busy(api_level_t * me, QEvt const * const e)
{
    QState status = Q_SUPER(&api_level_enabled);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            ao_set_busy(&me->ao_timings);

            // Arm busy timer
            whoop_qp_time_safe_arm(&me->busy_event, MS_TO_TICKS(API_LEVEL_LOCKUP_TIME_MS), 0u);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            QTimeEvt_disarm(&me->busy_event);

            status = Q_HANDLED();
            break;

        }

        // Add a state to handle requests and fill up the defer queue using 
        /*
            bool success = QActive_defer(&me->super, &me->deferred_event_queue, e);
            if (!success)
            {
                DEBUG_OUT(1u, "%s: queue full--could NOT defer -> (signal %d).\n", API_LEVEL_NAME, e->sig);

                me->last_error = E_WHOOP_API_LEVEL_QUEUE_FULL;
                api_level_error_response(me, me->last_error, E_S_WHOOP_ERROR, E_WHOOP_NO_EXTRA_INFO);
            }
        */

        case LOCAL_API_LEVEL_BUSY_TIMEOUT_SIG:
        {
            // AO was busy for too long. Publish an error and exit.
            api_level_error_response(me, E_WHOOP_API_LEVEL_BUSY_TIMEOUT, E_S_WHOOP_ERROR);
            me->last_error = E_WHOOP_API_LEVEL_BUSY_TIMEOUT;
            status = Q_TRAN(&api_level_idle);
            break;
        }
        default:
        {
            break;
        }
    }

    return status;
}

/*! @brief      Fatal error state
*   @details    Backstop for error state
*   @param[in]  api_level_t    - pointer to instance of API_LEVEL Active Object
*   @param[in]  QEvt           - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState         - pointer to the QHsm object
*/
QState api_level_error(api_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&api_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Set the status of the API_LEVEL device to error
            ao_set_idle(&me->ao_timings);
            me->status = API_LEVEL_FATAL_ERROR;
            api_level_publish_status(me);
            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case API_LEVEL_ENABLE_SIG:
        {
            // Once we receive a start signal, move to the 'starting' state
            DEBUG_OUT(1u, "%s: Driver Starting from error state\n", API_LEVEL_NAME);

            status = Q_TRAN(&api_level_starting);
            break;
        }

        // Ignore a repeated attempt to disable
        case API_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Disabling from error state.\n", API_LEVEL_NAME);
            status = Q_TRAN(&api_level_disabled);
            break;
        }

        default:
        {
            break;
        }
    }
    return status;
}

/************************************************************************************/
/***    END OF HSM                                                                ***/
/************************************************************************************/

/*!
*   @brief      Publish an error response
*   @details
*   @param[in]  api_level_t  - pointer to instance of API_LEVEL Active Object
*   @param[in]  error_code    - Reported error
*   @param[out] nothing
*   @return     nothing
*/
static void api_level_error_response(api_level_t * const me, int32_t error_code,
                                      whoop_error_severity_t error_severity)
{
    DEBUG_OUT(2u, "%s: Error reported, error code 0x%02X\n", API_LEVEL_NAME, error_code);
    generic_error_signal_t * err_evt = Q_NEW(generic_error_signal_t, GENERIC_ERROR_REPORT_SIG);

    err_evt->error_code     = error_code;
    err_evt->ao_name        = API_LEVEL_NAME;
    err_evt->error_severity = error_severity;
    err_evt->error_subsys   = E_WHOOP_SUBSYS_API_LEVEL;
    err_evt->extra_info     = 0u;

    QF_PUBLISH((QEvt*)err_evt, me);
}

/*!
*   @brief      Publish the status of the API Level AO
*   @param[in]  api_level_t - pointer to instance of API_LEVEL active object
*   @param[out] nothing
*   @return     nothing
*/
static void api_level_publish_status(api_level_t * const me)
{
    if (me->status == API_LEVEL_ENABLED)
    {
        static QEvt const stat_evt = {API_LEVEL_READY_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
    else if (me->status == API_LEVEL_DISABLED)
    {
        static QEvt const stat_evt = {API_LEVEL_DISABLE_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
    else
    {
        static QEvt const stat_evt = {API_LEVEL_ERROR_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
}

/*!
*   @brief      Setup and start the Active Object
*   @details
*   @param[in]  None
*   @param[out] nothing
*   @return     nothing
*/
void api_level_start(void)
{
    // API_LEVEL Active Object constructor
    api_level_ctor();

    // Start the AO
    QACTIVE_START(g_ao_api_level,                 // AO pointer to start
                  API_LEVEL_PRIORITY,             // unique QP priority of the AO
                  api_level_que_sto,            // storage for the AO's queue
                  Q_DIM(api_level_que_sto),     // length of the queue [entries]
                  (void *)0,                       // stack storage (not used in QK)
                  0U,                              // stack size [bytes] (not used in QK)
                  (QEvt *)0);                      // initial event (or 0)
}

/**
 * @brief Retrieve AO status
 *
 */
bool api_level_get_status(void)
{
    return ao_api_level.status;
}

/**
 * @brief Retrieve whoop defined error information
 *
 */
int32_t api_level_get_last_error(void)
{
    return ao_api_level.last_error;
}
