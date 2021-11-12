/**
 * @file        device_level.h
 */

#ifndef device_level_H
#define device_level_H

#include "replyables.h"
#include "common.h"
#include "whoop_i2c.h"

#define DEVICE_LEVEL_NUM_REGISTERS   20u

#define DEVICE_LEVEL_BUFFER_SIZE    DEVICE_LEVEL_NUM_REGISTERS

// Enumerated driver status
typedef enum
{
    DEVICE_LEVEL_UNKNOWN      = 0,
    DEVICE_LEVEL_DISABLED     = 1,
    DEVICE_LEVEL_ENABLED      = 2,
    DEVICE_LEVEL_FATAL_ERROR  = 3,

} device_level_status_t;

/*! @struct device_level_t
*   @brief  Active Object structure
*/
typedef struct
{
    QActive                 super;
    QActive *               requestor;                          /**< Ptr to AO whose request we're servicing >*/
    uint32_t                device_level_req_id;                /**< I2C Request ID >*/
    QTimeEvt                time_event;                         /**< Timeout timer. >*/
    QTimeEvt                startup_timer;                      /**< Timeout timer. >*/
    QTimeEvt                busy_timer;                         /**< Dedicated Busy State timer. >*/
    uint32_t                i2c_transaction_id;                 /**< I2C request id value >*/
    i2c_ops_t               i2c_operation;                      /**< I2C read or write? >*/
    uint8_t                 write_data[DEVICE_LEVEL_BUFFER_SIZE];     /**< Data Buffer for write requests > */
    uint8_t                 read_data[DEVICE_LEVEL_BUFFER_SIZE];      /**< Data Buffer for read requests > */
    device_level_register_t reg_ptr;                            /**< Register to be read or written to >*/
    uint32_t                data_len;                           /**< Length of data to be read/written >*/
    uint8_t                 n_retries;                          /**< I2C Retry attempts > */
    uint8_t                 event_req_id;                       /**< Request ID of requests sent to the AO */
    uint32_t                debug_level;                        /**< Current threshold for gating debug output. >*/
    device_level_status_t   status;                             /**< Current status of the AO. >*/
} device_level_t;

// opaque pointer to internal active object
extern QActive * const g_ao_device_level;

/************************************************************************************/
/***    PUBLIC EVENT DEFINITIONS                                                  ***/
/************************************************************************************/

/**
    @brief AO Status Event
*/
typedef struct
{
    QEvt               super;                 /**<Extend the QEvent class */

    device_level_status_t    status;                /**<Current status of  device */
} device_level_status_event_t;

/**
    @brief High-level AO read report event
    @note  Dispatched to the high-level AO driver from the device driver
    @note  This is a replyable event
*/
typedef struct
{
    q_event_replyable_request_t     super;      /**<Extend q_event_replyable_response_t */
    uint8_t                         data;       /**<Transaction data */
    device_level_register_t         reg;        /**<Data register */
} device_level_read_request_event_t;

/**
    @brief High-level AO write request event
    @note  Dispatched to the high-level AO driver
*/
typedef struct
{
    q_event_replyable_request_t     super;      /**<Extend q_event_replyable_response_t */
    uint8_t                         data;       /**<Transaction data */
    device_level_register_t         reg;        /**<Data register */
} device_level_write_request_event_t;


typedef struct
{
    /* inherit: */
    q_event_replyable_response_t    super;        /**<Extend q_event_replyable_response_t */

    /* extend: */
    uint8_t                         data;         /**<Transaction data */
    device_level_register_t         reg;          /**<Data register */
} device_level_read_report_event_t;

typedef struct
{
    /* inherit: */
    q_event_replyable_response_t    super;        /**<Extend q_event_replyable_response_t */

    /* extend: */
    uint8_t                         data;         /**<Transaction data */
    device_level_register_t         reg;          /**<Data register */
} device_level_write_report_event_t;

typedef struct
{
    /* inherit: */
    q_event_replyable_response_t    super;        /**<Extend q_event_replyable_response_t */

    /* extend: */
    whoop_error_t                   error;        /**<Whoop error information*/
} device_level_error_event_t;

// Helper functions
void device_level_ctor(void);
void device_level_start(void);
device_level_status_t device_level_get_status(void);
device_level_register_t device_level_get_write_address(void);
uint8_t * device_level_get_write_data(void);
device_level_register_t device_level_get_read_address(void);
uint8_t * device_level_get_read_data(void);
void device_level_set_debug_level(uint32_t level);

#endif
