#ifndef NTURT_STATES_H_
#define NTURT_STATES_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

// project incldues
#include "nturt/util.h"

/**
 * @addtogroup States States
 *
 * @{
 */

/* macros --------------------------------------------------------------------*/
/**
 * @addtogroup ErrorMacro Error Macros
 *
 * @{
 */

/// @brief Error severity. Warning means that the system can continue to
/// operate.
#define ERR_SEV_WARN false

/// @brief Error severity. Fatal means that the system must stop.
#define ERR_SEV_FATAL true

/// @brief Error code set bit.
#define ERR_CODE_SET BIT(31)

/// @brief Error code clear bit.
#define ERR_CODE_CLEAR 0

/// @brief Prefix for error code names.
#define ERR_CODE_PREFIX ERR_CODE_

/**
 * @brief Listify error codes defined by @ref ERR_CODE.
 *
 * @param ... List of error codes.
 */
#define ERR_CODE_LISTIFY(...) __VA_ARGS__

/**
 * @brief Check if error code is valid.
 *
 * @param LIST List of error codes defined by @ref ERR_CODE_LISTIFY.
 */
#define ERR_CODE_VALIDATE(LIST)         \
  BUILD_ASSERT(NUM_VA_ARGS(LIST) <= 31, \
               "Too many error codes. Only supports up to 31.")

#define _ERR_CODE_DECLARE(I, X) CONCAT(ERR_CODE_PREFIX, X) = BIT(I)

/**
 * @brief Declare the values of the error codes using enum.
 *
 * @param LIST List of error codes defined by @ref ERR_CODE_LISTIFY.
 */
#define ERR_CODE_DECLARE(LIST) \
  enum err_code { FOR_EACH_IDX(_ERR_CODE_DECLARE, (, ), LIST) }

#define _ERR_CODE_GET_FATAL_MASK(I, X) COND_CODE_1(X, (BIT(I)), (0))

/**
 * @brief Get the mask of fatal error codes.
 *
 * @param LIST List of error codes defined by @ref ERR_CODE_LISTIFY.
 */
#define ERR_CODE_GET_FATAL_MASK(LIST) \
  (FOR_EACH_IDX(_ERR_CODE_GET_FATAL_MASK, (|), GET_EVERY_PAIR_SECOND(LIST)))

#define _ERR_CODE_GET_WARN_MASK(I, X) COND_CODE_0(X, (BIT(I)), (0))

/**
 * @brief Get the mask of warning error codes.
 *
 * @param LIST List of error codes defined by @ref ERR_CODE_LISTIFY.
 */
#define ERR_CODE_GET_WARN_MASK(LIST) \
  (FOR_EACH_IDX(_ERR_CODE_GET_WARN_MASK, (|), GET_EVERY_PAIR_SECOND(LIST)))

/**
 * @brief Iterate over each individual error code in a combined error.
 *
 * @param ERR Combined error code.
 * @param CODE Individual error code.
 */
#define ERR_CODE_FOR_EACH(ERR, CODE)  \
  for (int __i = 0; ERR >> __i != 0;) \
    if ((__i += __builtin_ffs(ERR >> __i)) && (CODE = BIT(__i - 1)))

/// @brief Define the error code list.
#define ERR_CODE_LIST                                                        \
  ERR_CODE_LISTIFY(NODE_FB, NODE_RB, NODE_RPI, NODE_ACC, NODE_INV_FL,        \
                   NODE_INV_FR, NODE_INV_RL, NODE_INV_RR, CAN, STEER, APPS1, \
                   APPS2, APPS_PLAUS, BSE_F, BSE_R, PEDAL_PLAUS, SUSP_DIVE,  \
                   SUSP_ROLL)

/**
 * @} // ErrorMacro
 */

ERR_CODE_VALIDATE(ERR_CODE_LIST);

// #define ERR_CODE_FATAL_MASK ERR_CODE_GET_FATAL_MASK(ERR_CODE_LIST)

// #define ERR_CODE_WARN_MASK ERR_CODE_GET_WARN_MASK(ERR_CODE_LIST)

/* types ---------------------------------------------------------------------*/
typedef uint32_t err_t;

ERR_CODE_DECLARE(ERR_CODE_LIST);

/* exported varaibles --------------------------------------------------------*/
ZBUS_CHAN_DECLARE(error_chan);

/* function declaration ------------------------------------------------------*/
/**
 * @brief Set or clear errors. Multiple error codes can be set at once. Must be
 * ones defined by @ref ERR_CODE_LISTIFY.
 *
 * @param errors Errors to set or clear.
 * @param set Set or clear errors.
 */
void states_set_errors(err_t errors, bool set);

/**
 * @brief Get the current errors.
 *
 * @return Current errors.
 */
err_t states_get_errors();

/**
 * @} // States
 */

#endif  // NTURT_STATES_H_
