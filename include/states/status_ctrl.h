#ifndef STATES_STATUS_CTRL_H_
#define STATES_STATUS_CTRL_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/kernel.h>

// project includes
#include "states.h"

/* macro ---------------------------------------------------------------------*/
/// @brief Designated initializer for @ref status_ctrl.
#define STATUS_CTRL_INITIALIZER()                                    \
  {                                                                  \
      .cmd =                                                         \
          {                                                          \
              .inv_ctrl_word =                                       \
                  {                                                  \
                      .fl = 0,                                       \
                      .fr = 0,                                       \
                      .rl = 0,                                       \
                      .rr = 0,                                       \
                  },                                                 \
          },                                                         \
      .inv_fault_reset_work =                                        \
          Z_WORK_DELAYABLE_INITIALIZER(status_inv_fault_reset_work), \
  }

/* type ----------------------------------------------------------------------*/
/// @brief Status control structure.
struct status_ctrl {
  struct status_cmd cmd;
  struct k_work_delayable inv_fault_reset_work;
};

/* function declaration ------------------------------------------------------*/
/**
 * @brief Enable or disable the status control.
 * 
 * @param status The status control structure.
 * @param enable True to enable, false to disable.
 */
void status_enable(struct status_ctrl *status, bool enable);

/**
 * @brief Reset inverter fault.
 * 
 * @param status The status control structure.
 */
void status_inv_fault_reset(struct status_ctrl *status);

/// @brief Internal work function for @ref status_inv_fault_reset.
void status_inv_fault_reset_work(struct k_work *work);

/**
 * @brief Set the inverter direction.
 * 
 * @param status The status control structure.
 * @param dir True for forward, false for reverse.
 */
void status_inv_dir(struct status_ctrl *status, bool dir);

#endif  // STATES_STATUS_CTRL_H_
