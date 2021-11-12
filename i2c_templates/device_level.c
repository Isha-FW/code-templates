/**
 * @file        device_level.c
 * @author      
 * @brief       This file contains the function implementations for the
 *              DEVICE_LEVEL low-level driver.
*
 *              One starting state:
*               device_level_initial        -   The initial state as required by QP
 *
 *              One super state:
 *              device_level_backstop       -   Handler for uncaught or error case signals
 *
 *              Four states are children of the backstop state:
 *              device_level_disabled     -   Bounces all requests, waits for an enable signal
 *              device_level_starting     -   Have received enable signal, wait for DEVICE_LEVEL ready
 *              device_level_enabled      -   DEVICE_LEVEL is now ready, signal the supervisor, move to idle
 *              device_level_error          - Fatal error state
 *
 *              Two states are children of the enabled state:
 *              device_level_idle           -   The normal inactive state of the DEVICE_LEVEL object
 *              device_level_busy           -   A superstate for while the DEVICE_LEVEL I2C is busy
 *                                      Incoming requests while the driver is in the busy
 *                                      state are cached in a queue until the driver is
 *                                      idle again
 *
 *
 * @version     0.1
 * @date        2020-06-15
 *
 * @copyright   Copyright (c) 2020 WHOOP
 *
 */
#include <stdint.h>
#include "qpc.h"

#include "common.h"

#include "events.h"
#include "signals.h"
#include "whoop_i2c.h"
#include "i2c_patch.h"
#include "dio.h"
#include "dio_pin.h"
#include "whoop_qp_time.h"
#include "device_level.h"


// I2C information
#define DEVICE_LEVEL_SLAVE_ADDRESS        0xXXu

/**
 *  @brief      define the human-readable name for this module
*/

#define DEVICE_LEVEL_NAME                 "DEVICE_LEVEL"

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
#define DEBUG_LEVEL                       (me->debug_level)

#define DEVICE_LEVEL_I2C_ACTIVE_RETRIES   10U

/**
    @brief Ensure the AO doesn't wait forever if the device is stuck
    Set to the minimum allowed timeout time.
*/
#define DEVICE_LEVEL_LOCKUP_TIME_MS       20u

// Allow more time for initialization
#define DEVICE_LEVEL_INIT_LOCKUP_TIME_MS  500u

/**
    @brief define the maximum allowable busy time for the AO
    To ensure that the AO does not fail to exit the busy state and
    block sleep, start a timer on entry to the busy state and disarm on
    exit. If the timer fires, perform any necessary cleanup and exit to
    idle.
    This time was chosen as an absolute maximum. In normal operation,
    only one register should be read at a time, which should last
    substantially less than 100ms including timeouts and retries.
*/
#define DEVICE_LEVEL_BUSY_TIME_MS         100u

// Define the queue size for Qp
#define DEVICE_LEVEL_QUEUE_SIZE           10u

#define DEVICE_LEVEL_BUFFER_SIZE          10u

// the single instance of the internal device_level object
static device_level_t ao_device_level =
{
    .debug_level                = STARTING_DEBUG_LEVEL,
    .status                     = DEVICE_LEVEL_UNKNOWN,
    .i2c_transaction_id         = 0u,
    .last_error                 = E_WHOOP_NO_ERROR,
    .last_hal_error             = E_NO_ERROR,
};

// Globally scoped opaque pointer
QActive * const g_ao_device_level = &ao_device_level.super;

// I2C object queue storage space
static QEvt const * device_level_que_sto[DEVICE_LEVEL_QUEUE_SIZE];

// Define a retry counter for functions that need it
static uint8_t  device_level_retry_counter = 0u;


// state functions
static QState device_level_initial        (device_level_t * const me, QEvt const * const e);
static QState device_level_backstop       (device_level_t * const me, QEvt const * const e);
static QState device_level_disabled       (device_level_t * const me, QEvt const * const e);
static QState device_level_starting       (device_level_t * const me, QEvt const * const e);
static QState device_level_enabled        (device_level_t * const me, QEvt const * const e);
static QState device_level_idle           (device_level_t * const me, QEvt const * const e);
static QState device_level_busy           (device_level_t * const me, QEvt const * const e);
static QState device_level_read           (device_level_t * const me, QEvt const * const e);
static QState device_level_write          (device_level_t * const me, QEvt const * const e);
static QState device_level_error          (device_level_t * const me, QEvt const * const e);

// Helper functions

static bool device_level_is_i2c_bus_enabled(i2c_bus_status_t status);

static void device_level_publish_error_response(device_level_t * const me, int32_t error_code,
        whoop_error_severity_t error_severity);

static void device_level_publish_status(device_level_t * const me);

static void device_level_i2c_read(device_level_t * const me);

static void device_level_i2c_write(device_level_t * const me);

static void device_level_i2c_comm_req(device_level_t * const me);

static bool device_level_try_retry(device_level_t * const me);

// Signals for use in local context only
enum
{
    LOCAL_DEVICE_LEVEL_TIMEOUT_SIG = MAX_SIG,  /**< Timeout signal used in several spots >*/
    LOCAL_DEVICE_LEVEL_BUSY_TIMEOUT_SIG,
    LOCAL_DEVICE_LEVEL_ACTION_ENTER_IDLE_SIG,
    LOCAL_DEVICE_LEVEL_RETRY_SIG,
    LOCAL_DEVICE_LEVEL_I2C_TRANSACTION_START_RW_SIG,
};

/************************************************************************************/
/***    START OF HSM                                                              ***/
/************************************************************************************/

/**
*   @brief      DEVICE_LEVEL active object contructor
*   @details
*   @param[in]  nothing
*   @param[in]  nothing
*   @param[out] nothing
*   @return     nothing
*/
void device_level_ctor(void)
{
    //Create a pointer to the instance of myself
    device_level_t * const me = &ao_device_level;

    //Register AO, and set entry state
    QActive_ctor(&me->super, (QStateHandler)&device_level_initial);

    // Create a timer object for DEVICE_LEVEL communications timeout detection
    QTimeEvt_ctorX(&me->time_event,  &me->super, LOCAL_DEVICE_LEVEL_TIMEOUT_SIG, 0U);

    // Create a timer object for the DEVICE_LEVEL busy state timeout detection
    QTimeEvt_ctorX(&me->busy_timer,  &me->super, LOCAL_DEVICE_LEVEL_BUSY_TIMEOUT_SIG, 0U);
}

/**
*   @brief      Initial state as required by QP
*   @details
*   @param[in]  device_level_t     - pointer to instance low level device_level driver
*   @param[in]  QEvt         - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState       - pointer to the QHsm object
*/
static QState device_level_initial(device_level_t * me, QEvt const * const e)
{
    (void)e;    // avoid compiler warning

    // create object dictionary entries
    QS_OBJ_DICTIONARY(me);
    QS_FUN_DICTIONARY(&device_level_initial);
    QS_FUN_DICTIONARY(&device_level_backstop);
    QS_FUN_DICTIONARY(&device_level_disabled);
    QS_FUN_DICTIONARY(&device_level_starting);
    QS_FUN_DICTIONARY(&device_level_enabled);
    QS_FUN_DICTIONARY(&device_level_idle);
    QS_FUN_DICTIONARY(&device_level_busy);
    QS_FUN_DICTIONARY(&device_level_read);
    QS_FUN_DICTIONARY(&device_level_write);
    QS_FUN_DICTIONARY(&device_level_error);

    // Subscribe to the necessary I2C messages
    QActive_subscribe((QActive *)me, I2C_BUS_STATUS_SIG);

    me->status = DEVICE_LEVEL_DISABLED;

    // Move to the idle state, and begin to service requests
    return Q_TRAN(&device_level_disabled);
}

/**
*   @brief      Backstop handles signals not caught by substates
*   @details
*   @param[in]  device_level_t    - pointer to instance low level device_level driver
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
QState device_level_backstop(device_level_t * const me, QEvt const * const e)
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
        // If bus status changes to not ready, go back to the disabled state
        case I2C_BUS_STATUS_SIG:
        {
            i2c_comm_status_event_t * p_evt = (i2c_comm_status_event_t *) e;
            
            status = Q_HANDLED();
            
            if (!device_level_is_i2c_bus_enabled(p_evt->status))
            {
                // Flag the device as disabled
                me->status = DEVICE_LEVEL_DISABLED;

                //Switch to the disabled state
                status = Q_TRAN(&device_level_disabled);
            }
            break;
        }
        // Handle this in the backstop since it can happen at any point
        case DEVICE_LEVEL_REQ_STAT_SIG:
        {
            device_level_publish_status(me);
            status = Q_HANDLED();
            break;
        }

        // If we receive a request to disable the device, service it here
        case DEVICE_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver Disabled.\n", DEVICE_LEVEL_NAME);

            // Mark the device status as DISABLED
            me->status = DEVICE_LEVEL_DISABLED;

            // Move to the disabled state
            status = Q_TRAN(&device_level_disabled);
            break;
        }

        default:
        {
            // Catch unhandled signals here
            if (e->sig < MAX_SIG)
            {
                DEBUG_OUT(1u, "%s: ignoring unhandled signal %s.\n", DEVICE_LEVEL_NAME, signals_get_signal_name(e->sig));
            }
            else
            {
                DEBUG_OUT(1u, "%s: ignoring unhandled signal %d.\n", DEVICE_LEVEL_NAME, (e->sig - MAX_SIG));
            }

            break;
        }
    }
    return status;
}

/**
*   @brief      Wait for enable signal from supervisor
*   @details    The disabled state waits to receive an 'enable' signal from the supervisor
*               Once this signal is received, the AO will transition to the 'starting' state

*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_disabled(device_level_t * me, QEvt const *e)
{
    QState status = Q_SUPER(&device_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Set device to disabled
            me->status = DEVICE_LEVEL_DISABLED;
            device_level_publish_status(me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        // If we get the enable signal, transition to starting
        case DEVICE_LEVEL_ENABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver Starting\n", DEVICE_LEVEL_NAME);
            status = Q_TRAN(&device_level_starting);
            break;
        }

        // Ignore a repeated attempt to disable
        case DEVICE_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Device already disabled.\n", DEVICE_LEVEL_NAME);
            status = Q_HANDLED();
            break;
        }

        case DEVICE_LEVEL_WRITE_SIG:
        case DEVICE_LEVEL_READ_SIG:
        {
            DEBUG_OUT(1u, "%s: Device is disabled, cannot complete request %d\n", DEVICE_LEVEL_NAME, e->sig);
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

/**
*   @brief      Wait for i2c to become available
*   @details    Waits for the I2C bus to become available before transitioning to the 'enabled' state.
*               At this point we have received the 'enable' signal from the supervisor, and therefore
*               also know that the i2c bus AO is running. We then check to confirm that the communication
*               channel is available and working before transitioning to the 'enabled' state
*   @param[in]  device_level_t  - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_starting(device_level_t * me, QEvt const *e)
{
    QState status = Q_SUPER(&device_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {

            // One-shot timer in case I2C not ready or unresponsive
            whoop_qp_time_safe_arm(&me->time_event, MS_TO_TICKS(DEVICE_LEVEL_INIT_LOCKUP_TIME_MS), 0u);

            // Post a local signal to begin the process
            static QEvt const start_rw_event = {LOCAL_DEVICE_LEVEL_ACTION_ENTER_IDLE_SIG, 0, 0};
            QACTIVE_POST(&me->super, &start_rw_event, me);
            
            status = Q_HANDLED();
            break;
        }

        case LOCAL_DEVICE_LEVEL_ACTION_ENTER_IDLE_SIG:
        {
            status = Q_TRAN(&device_level_idle);
            break;
        }

        // If timed out before the i2c bus is ready, re-enter state and try again
        case LOCAL_DEVICE_LEVEL_TIMEOUT_SIG:
        {
            bool retry_ok = device_level_try_retry(me);
            
            status = Q_HANDLED();
            
            if (!retry_ok)
            {
                DEBUG_OUT(1u, "%s: Too many timeouts during startup, giving up\n", DEVICE_LEVEL_NAME);
                status = Q_TRAN(&device_level_error);
            }
            
            break;
        }

        // Ignore repeated attempts to enable
        case DEVICE_LEVEL_ENABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Device is already starting.\n", DEVICE_LEVEL_NAME);
            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
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

/**
*   @brief      i2c bus is ready and available
*   @details    Transitions to idle if the I2C bus is ready and transitions to
*               disabled if the disable signal is received
*
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_enabled(device_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&device_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver enabled.\n", DEVICE_LEVEL_NAME);
            // Mark the device status as enabled
            me->status = DEVICE_LEVEL_ENABLED;
            device_level_publish_status(me);

            // Post a local signal to begin the process
            static QEvt const start_rw_event = {LOCAL_DEVICE_LEVEL_ACTION_ENTER_IDLE_SIG, 0, 0};
            QACTIVE_POST(&me->super, &start_rw_event, me);

            status = Q_HANDLED();
            break;
        }

        case LOCAL_DEVICE_LEVEL_ACTION_ENTER_IDLE_SIG:
        {
            // Transition to Idle
            status = Q_TRAN(&device_level_idle);
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case DEVICE_LEVEL_ENABLE_SIG:
        {
            DEBUG_OUT(2u, "%s: Already Enabled\n", DEVICE_LEVEL_NAME);
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

/**
*   @brief      Idle waits for read and write requests.
*   @details
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_idle(device_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&device_level_enabled);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Set status as enabled
            me->status = DEVICE_LEVEL_ENABLED;

            // Reset the I2C request ID
            me->i2c_transaction_id = 0u;
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        case DEVICE_LEVEL_WRITE_SIG:
        {
            DEBUG_OUT(1u, "%s: Received write request\n", DEVICE_LEVEL_NAME);
            device_level_write_request_event_t * p_evt  = (device_level_write_request_event_t *) e;

            // Store transaction type
            me->i2c_operation = I2C_WRITE;

            // Store the signal, request ID, requester, write address, data
            me->device_level_req_id           = Q_GET_REPLYABLE_REQUEST_ID(p_evt);
            me->requestor               = Q_GET_REPLYABLE_REQUEST_REQUESTOR(p_evt);
            me->write_data              = p_evt->buffer;

            status =  Q_TRAN(&device_level_write);

            break;
        }

        case DEVICE_LEVEL_READ_SIG:
        {
            DEBUG_OUT(1u, "%s: Received read request\n", DEVICE_LEVEL_NAME);

            // Store a local copy of the event data for use in the transaction
            device_level_read_request_event_t * p_evt   = (device_level_read_request_event_t *) e;


            // Store transaction type
            me->i2c_operation = I2C_READ;

            // Store the request ID, requester, and read address
            me->device_level_req_id               = Q_GET_REPLYABLE_REQUEST_ID(p_evt);
            me->requestor                   = Q_GET_REPLYABLE_REQUEST_REQUESTOR(p_evt);
            me->read_data                   = p_evt->buffer;

            status = Q_TRAN(&device_level_read);

            break;
        }

        default:
        {

            break;
        }
    }
    return status;
}

/**
*   @brief      Device active state
*   @details    Busy handles any signals that come in during an active read
*               or write cycle.
*
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_busy(device_level_t * me, QEvt const *e)
{
    QState status = Q_SUPER(&device_level_enabled);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Arm dedicated busy state timer
            whoop_qp_time_safe_arm(&me->busy_timer, MS_TO_TICKS(DEVICE_LEVEL_BUSY_TIME_MS), 0);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG:
        {
            // Disarm timer
            QTimeEvt_disarm(&me->busy_timer);
            status = Q_HANDLED();
            break;
        }

        // We shouldn't get any requests while we're busy--they should be queued by the caller until ready
        case DEVICE_LEVEL_WRITE_SIG:
        case DEVICE_LEVEL_READ_SIG:
        {
            // AO will need to subscribe to the busy signal
            device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_BUSY, E_S_WHOOP_WARNING);
            me->last_error = E_WHOOP_DEVICE_LEVEL_BUSY;
            status = Q_HANDLED();
            break;
        }

        case LOCAL_DEVICE_LEVEL_BUSY_TIMEOUT_SIG:
        {
            // Problem: we didn't get an I2C response after the timeout interval
            bool retry_ok = device_level_try_retry(me);

            if (!retry_ok)
            {
                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT, E_S_WHOOP_ERROR);
                me->last_error = E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT;
                me->last_hal_error = E_TIME_OUT;

                status = Q_TRAN(&device_level_idle);
            }
            else
            {
                DEBUG_OUT(1u, "%s: Got timeout error during read, retrying\n", DEVICE_LEVEL_NAME);
                status = Q_HANDLED();
            }

            break;
        }

        default:
        {
            break;
        }
    }
    return status;
}

/**
*   @brief      Performs a page read
*   @details    The read state issues a read request to the DEVICE_LEVEL device based on information passed
*               down from the upper level driver, and receives the requested data into a local buffer.
*               The upper level driver is then notified of the status of the request and any data returned.
*
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_read(device_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&device_level_busy);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Start a timer to catch i2c lockups.
            whoop_qp_time_safe_arm(&me->time_event, MS_TO_TICKS(DEVICE_LEVEL_LOCKUP_TIME_MS), 0U);

            // Post a local signal to begin the process
            static QEvt const start_rw_event = {LOCAL_DEVICE_LEVEL_I2C_TRANSACTION_START_RW_SIG, 0, 0};
            QACTIVE_POST(&me->super, &start_rw_event, me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            // Disarm lockup detection timer if it hasn't already fired.
            QTimeEvt_disarm(&me->time_event);
            status = Q_HANDLED();
            break;
        }

        case LOCAL_DEVICE_LEVEL_I2C_TRANSACTION_START_RW_SIG:
        {
            device_level_i2c_read(me);
            status = Q_HANDLED();
            break;
        }

        case I2C_COMM_COMPLETE_SIG:
        {

            DEBUG_OUT(2u, "%s: Received i2c response to read request\n", DEVICE_LEVEL_NAME);
            i2c_comm_cmpt_event_t * p_evt = (i2c_comm_cmpt_event_t *) e;

            // Make sure this is a response to our signal
            if (Q_DOES_REPLYABLE_RESPONSE_REQUEST_ID_MATCH(p_evt, me->i2c_transaction_id))
            {
                QTimeEvt_disarm(&me->time_event);

                device_level_response_event_t * rsp_evt = Q_NEW(device_level_response_event_t, DEVICE_LEVEL_RESPONSE_SIG);
                rsp_evt->req_type = DEVICE_LEVEL_READ;
                rsp_evt->buffer = me->read_data;

                QACTIVE_POST_REPLYABLE_RESPONSE(me->requestor, me->device_level_req_id, rsp_evt, me);

                status = Q_TRAN(&device_level_idle);

            }
            else
            {
                // If we received a mismatch, then it may not necessarily be an error. The replyable system allows
                // for multiple requests from a single sender, with unique transaction id's. Since we are not using
                // it in this way here, a mismatch, albeit highly unlikely, may in fact indicate an error here.
                // Treat it as a warning at this point.

                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID, E_S_WHOOP_WARNING);
                me->last_error = E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID;
                status = Q_HANDLED();

            }
            break;
        }

        case I2C_COMM_ERROR_SIG:
        {
            i2c_comm_error_event_t * p_evt = (i2c_comm_error_event_t *) e;

            // Make sure this is a response to our signal
            if (Q_DOES_REPLYABLE_RESPONSE_REQUEST_ID_MATCH(p_evt, me->i2c_transaction_id))
            {
                DEBUG_OUT(1u, "%s: Got communication error during read\n", DEVICE_LEVEL_NAME);

                QTimeEvt_disarm(&me->time_event);

                device_level_publish_error_response(me, p_evt->error_code, E_S_WHOOP_ERROR);
                me->last_error = E_WHOOP_DEVICE_LEVEL_I2C_ERROR;
                me->last_hal_error = p_evt->error_code;

                status = Q_TRAN(&device_level_error);
            }
            else
            {
                // If we received a mismatch, then it may not necessarily be an error. The replyable system allows
                // for multiple requests from a single sender, with unique transaction id's. Since we are not using
                // it in this way here, a mismatch, albeit highly unlikely, may in fact indicate an error here.
                // Treat it as a warning at this point.

                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID, E_S_WHOOP_WARNING);
                me->last_error = E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID;
                status = Q_HANDLED();
            }
            break;
        }

        case LOCAL_DEVICE_LEVEL_TIMEOUT_SIG:
        {
            // Problem: we didn't get an I2C response after the timeout interval
            bool retry_ok = device_level_try_retry(me);

            if (!retry_ok)
            {
                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT, E_S_WHOOP_ERROR);
                me->last_error = E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT;
                me->last_hal_error = E_TIME_OUT;

                status = Q_TRAN(&device_level_idle);
            }
            else
            {
                DEBUG_OUT(1u, "%s: Got timeout error during read, retrying\n", DEVICE_LEVEL_NAME);
                status = Q_HANDLED();
            }

            break;
        }

        default:    // should never get here
        {

            break;
        }
    }
    return status;
}

/**
*   @brief      Performs a register write
*   @details    The write state issues a write request to the device_level device based on information passed
*               down from the upper level driver, and receives the requested data into a local buffer.
*               The upper level driver is then notified of the status of the request and any data returned.
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_write(device_level_t * const me, QEvt const * const e)
{
    QState status = Q_SUPER(&device_level_busy);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Start a timer to catch i2c lockups.
            whoop_qp_time_safe_arm(&me->time_event, MS_TO_TICKS(DEVICE_LEVEL_LOCKUP_TIME_MS), 0U);

            // Post a local signal to begin the process
            static QEvt const start_rw_event = {LOCAL_DEVICE_LEVEL_I2C_TRANSACTION_START_RW_SIG, 0, 0};
            QACTIVE_POST(&me->super, &start_rw_event, me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            // Disarm lockup detection timer if it hasn't already fired.
            QTimeEvt_disarm(&me->time_event);
            status = Q_HANDLED();
            break;
        }

        case LOCAL_DEVICE_LEVEL_I2C_TRANSACTION_START_RW_SIG:
        {
            // Initiate a read transaction on the I2C bus.
            me->i2c_operation = I2C_WRITE;
            device_level_i2c_write(me);
            status = Q_HANDLED();
            break;
        }

        case I2C_COMM_COMPLETE_SIG:
        {
            i2c_comm_cmpt_event_t * p_evt = (i2c_comm_cmpt_event_t *) e;

            DEBUG_OUT(1u, "%s: Received i2c response to write request\n", DEVICE_LEVEL_NAME);

            // Make sure this is a response to our signal
            if (Q_DOES_REPLYABLE_RESPONSE_REQUEST_ID_MATCH(p_evt, me->i2c_transaction_id))
            {
                QTimeEvt_disarm(&me->time_event);

                device_level_response_event_t * rsp_evt = Q_NEW(device_level_response_event_t, DEVICE_LEVEL_RESPONSE_SIG);
                rsp_evt->req_type = DEVICE_LEVEL_WRITE;
                rsp_evt->buffer = me->write_data;

                QACTIVE_POST_REPLYABLE_RESPONSE(me->requestor, me->device_level_req_id, rsp_evt, me);

                status = Q_TRAN(&device_level_idle);
            }
            else
            {
                // If we received a mismatch, then it may not necessarily be an error. The replyable system allows
                // for multiple requests from a single sender, with unique transaction id's. Since we are not using
                // it in this way here, a mismatch, albeit highly unlikely, may in fact indicate an error here.
                // Treat it as a warning at this point.

                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID, E_S_WHOOP_WARNING);
                me->last_error = E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID;
                status = Q_HANDLED();
            }
            break;
        }

        case I2C_COMM_ERROR_SIG:
        {
            i2c_comm_error_event_t * p_evt = (i2c_comm_error_event_t *) e;

            // Make sure this is a response to our signal
            if (Q_DOES_REPLYABLE_RESPONSE_REQUEST_ID_MATCH(p_evt, me->i2c_transaction_id))
            {
                DEBUG_OUT(1u, "%s: Got communication error during write\n", DEVICE_LEVEL_NAME);
                QTimeEvt_disarm(&me->time_event);

                device_level_publish_error_response(me, p_evt->error_code, E_S_WHOOP_ERROR);
                me->last_error = E_WHOOP_DEVICE_LEVEL_I2C_ERROR;
                me->last_hal_error = p_evt->error_code;

                status = Q_TRAN(&device_level_error);
            }
            else
            {
                // If we received a mismatch, then it may not necessarily be an error. The replyable system allows
                // for multiple requests from a single sender, with unique transaction id's. Since we are not using
                // it in this way here, a mismatch, albeit highly unlikely, may in fact indicate an error here.
                // Treat it as a warning at this point.

                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID, E_S_WHOOP_WARNING);
                me->last_error = E_WHOOP_DEVICE_LEVEL_MISMATCH_RESP_ID;
                status = Q_HANDLED();
            }
            break;
        }

        case LOCAL_DEVICE_LEVEL_TIMEOUT_SIG:
        {
            // Problem: we didn't get an I2C response after the timeout interval
            bool retry_ok = device_level_try_retry(me);

            if (!retry_ok)
            {
                device_level_publish_error_response(me, E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT, E_S_WHOOP_ERROR);
                me->last_error = E_WHOOP_DEVICE_LEVEL_I2C_TIMEOUT;
                me->last_hal_error = E_TIME_OUT;

                status = Q_TRAN(&device_level_idle);
            }
            else
            {
                DEBUG_OUT(1u, "%s: Got timeout error during write, retrying\n", DEVICE_LEVEL_NAME);
                status = Q_HANDLED();
            }

            break;
        }

        default:
        {
            break;
        }
    }
    return status;
}

/*! @brief      Superstate for fatal error condition
*   @details    Don't move to disabled when we reach an error condition. Instead,
*               enter the fatal error state and alert the supervisor.
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  QEvt        - pointer to event that caused entrance to state
*   @param[out] nothing
*   @return     QState      - pointer to the QHsm object
*/
static QState device_level_error(device_level_t * const me, QEvt const *e)
{
    QState status = Q_SUPER(&device_level_backstop);

    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Publish the error report signal.
            me->status = DEVICE_LEVEL_FATAL_ERROR;
            device_level_publish_status(me);

            status = Q_HANDLED();
            break;
        }

        case Q_EXIT_SIG:
        {
            status = Q_HANDLED();
            break;
        }

        // If we get the enable signal, try a restart
        case DEVICE_LEVEL_ENABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver starting from fatal error state.\n", DEVICE_LEVEL_NAME);
            status = Q_TRAN(&device_level_starting);
            break;
        }
        
        // Move to disabled if we get a disable signal
        case DEVICE_LEVEL_DISABLE_SIG:
        {
            DEBUG_OUT(1u, "%s: Driver disabling.\n", DEVICE_LEVEL_NAME);
            status = Q_TRAN(&device_level_disabled);
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

/**
*   @brief      Wrapper for I2C write
*   @details
*   @param[in]  device_level_t  - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  nothing
*   @param[out] nothing
*   @return     nothing
*/
static void device_level_i2c_write(device_level_t * const me)
{
    me->i2c_operation = I2C_WRITE;
    device_level_i2c_comm_req(me);
}

/*! @fn         static void device_level_i2c_read(device_level_t * const me)
*   @brief      Wrapper for I2C read
*   @details
*   @param[in]  device_level_t  - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  nothing
*   @param[out] nothing
*   @return     nothing
*/
static void device_level_i2c_read(device_level_t * const me)
{
    me->i2c_operation = I2C_READ;
    device_level_i2c_comm_req(me);
}

/**
*   @brief      Make an I2C request with the DEVICE_LEVEL data and the given operation
*   @details
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  i2c_ops_t   - Specifies operation to perform (read or write)
*   @param[out] nothing
*   @return     nothing
*/
static void device_level_i2c_comm_req(device_level_t * const me)
{
    i2c_comm_req_event_t * const p_evt = Q_NEW(i2c_comm_req_event_t, I2C_COMM_REQUEST_SIG);

    p_evt->bus_id = INTERNAL;
    p_evt->address = DEVICE_LEVEL_SLAVE_ADDRESS;

    // Increment transaction ID
    me->i2c_transaction_id++;

    i2c_transaction_data_t transaction  = {0};

    transaction.reg_addr_md = I2C_USE_REG_ADDR;
    transaction.operation = me->i2c_operation;
    transaction.nak_expected = false;
    transaction.rec_data = NULL;
    transaction.rec_data_len = 0u;
    transaction.send_data = NULL;
    transaction.send_data_len = 0u;
    transaction.reg_addr = 0u;


    if ((i2c_ops_t)(transaction.operation) == I2C_READ)
    {
        transaction.reg_addr = me->read_data.address;
        transaction.rec_data = me->read_data.p_data;
        transaction.rec_data_len = me->read_data.length;

        DEBUG_OUT(1u, "%s: dispatching read request to I2C, addr = 0x%03x\n", DEVICE_LEVEL_NAME, me->read_data.address);
    }
    else if ((i2c_ops_t)(transaction.operation) == I2C_WRITE)
    {
        transaction.reg_addr = me->write_data.address;
        transaction.send_data = me->write_data.p_data;
        transaction.send_data_len = me->write_data.length;
        DEBUG_OUT(1u, "%s: dispatching write request to I2C, addr = 0x%03x\n", DEVICE_LEVEL_NAME, me->write_data.address);
    }
    else
    {
        transaction.reg_addr = me->write_data.address;
        transaction.rec_data = me->write_data.p_data;
        transaction.rec_data_len = me->write_data.length;

        transaction.rec_data = me->read_data.p_data;
        transaction.rec_data_len = me->read_data.length;
        DEBUG_OUT(2u, "%s: dispatching write-verify request to I2C, addr = 0x%02x\n", DEVICE_LEVEL_NAME, me->write_data.address);
    }

    p_evt->transactions[0] = transaction;
    p_evt->num_transactions = 1;

    QACTIVE_POST_REPLYABLE_REQUEST(i2c_comm_ao, me->i2c_transaction_id, p_evt, me);

}

/**
*   @brief      Returns TRUE if internal bus (DEVICE_LEVEL I2C bus) is ready
*   @details
*   @param[in]  i2c_bus_status_t  - internally reported state of i2c bus
*   @param[in]  nothing
*   @param[out] nothing
*   @return     bool ready(True) or Not Ready (False)
*/
static bool device_level_func_is_i2c_bus_enabled(i2c_bus_status_t status)
{
    return (status == INTERNAL_ONLY_READY) || (status == BOTH_READY);
}

/**
*   @brief      Publish an error response
*   @details
*   @param[in]  device_level_t    - pointer to instance of DEVICE_LEVEL Active Object
*   @param[in]  error_code  - Reported error
*   @param[out] nothing
*   @return     nothing
*/
static void device_level_publish_error_response(device_level_t * const me, int32_t error_code,
        whoop_error_severity_t error_severity)
{
    DEBUG_OUT(2u, "%s: Error reported, error code 0x%02X\n", DEVICE_LEVEL_NAME, error_code);
    generic_error_signal_t * err_evt = Q_NEW(generic_error_signal_t, GENERIC_ERROR_REPORT_SIG);

    err_evt->error_code     = error_code;
    err_evt->ao_name        = DEVICE_LEVEL_NAME;
    err_evt->error_severity = error_severity;
    err_evt->error_subsys   = E_WHOOP_SUBSYS_DEVICE_LEVEL;

    // AO based extra info member
    err_evt->extra_info     = 0u;

    QF_PUBLISH((QEvt*)err_evt, me);
}

/*! @brief      Publish the status of the DEVICE_LEVEL AO
*   @param[in]  device_level_t - pointer to instance of DEVICE_LEVEL active object
*   @param[out] nothing
*   @return     nothing
*/
static void device_level_publish_status(device_level_t * const me)
{
    if (me->status == DEVICE_LEVEL_ENABLED)
    {
        static QEvt const stat_evt = {DEVICE_LEVEL_READY_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
    else if (me->status == DEVICE_LEVEL_DISABLED)
    {
        static QEvt const stat_evt = {DEVICE_LEVEL_DISABLE_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
    else
    {
        static QEvt const stat_evt = {DEVICE_LEVEL_ERROR_REPORT_SIG, 0u, 0u};
        QF_PUBLISH(&stat_evt, me);
    }
}

/**
*   @brief      Setup and start the Active Object
*   @details
*   @param[in]  None
*   @param[out] nothing
*   @return     nothing
*/
void device_level_start(void)
{
    // Call the DEVICE_LEVEL Active Object constructor
    device_level_ctor();

    // Start the AO
    QACTIVE_START(g_ao_device_level,                  // AO pointer to start
                  DEVICE_LEVEL_PRIORITY,              // unique QP priority of the AO
                  device_level_que_sto,             // storage for the AO's queue
                  Q_DIM(device_level_que_sto),      // length of the queue [entries]
                  (void *)0,                    // stack storage (not used in QK)
                  0U,                           // stack size [bytes] (not used in QK)
                  (QEvt *)0);                   // initial event (or 0)
}

/**
 * @brief Retrieve device_level status
 *
 */
bool device_level_get_status(void)
{
    return ao_device_level.status;
}

/**
 * @brief Retrieve device_level write data address
 *
 */
device_level_register_t device_level_get_write_address(void)
{
    return ao_device_level.write_data.address;
}

/**
 * @brief Retrieve device_level write data
 *
 */
uint8_t *  device_level_get_write_data(void)
{
    return ao_device_level.write_data.p_data;
}

/**
 * @brief Retrieve device_level read data address
 *
 */
device_level_register_t device_level_get_read_address(void)
{
    return ao_device_level.read_data.address;
}

/**
 * @brief Retrieve device_level read data
 *
 */
uint8_t * device_level_get_read_data(void)
{
    return ao_device_level.read_data.p_data;
}

/**
 * @brief Retrieve MAxim HAL error information
 *
 */
int32_t device_level_get_last_hal_error(void)
{
    return ao_device_level.last_hal_error;
}

/**
 * @brief Retrieve whoop defined error information
 *
 */
int32_t device_level_get_last_error(void)
{
    return ao_device_level.last_error;
}

/*! @brief      Check the retry counter and try a retry
*   @param[in]  device_level_t - Pointer to AO structure
*   @param[out] nothing
*   @return     bool      - true if retry signal was posted, false otherwise
*/
static bool device_level_try_retry(device_level_t * const me)
{
    bool retry_ok = true;

    // Check if we've already used all the retries
    if (device_level_retry_counter >= DEVICE_LEVEL_I2C_ACTIVE_RETRIES)
    {
        DEBUG_OUT(1u, "%s: Maximum number of retries (%u) reached\n", DEVICE_LEVEL_NAME, device_level_retry_counter);

        retry_ok = false;
    }
    // If we still have retries available, then retry
    else
    {
        // Increment the retry counter
        device_level_retry_counter++;

        // Post retry signal
        static QEvt const retry_evt = {LOCAL_DEVICE_LEVEL_RETRY_SIG, 0, 0};
        QACTIVE_POST(&me->super, &retry_evt, me);
        retry_ok = true;
    }

    return retry_ok;
}
