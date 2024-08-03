#ifndef STATES_H_
#define STATES_H_

// glibc includes
#include <stdint.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

/**
 * @addtogroup States States
 * @brief
 *
 * ```raw
 * ┌─────────────────────────────────────────────────┐
 * │ROOT                                             │
 * │ ┌───────────────────────────────┐ error ┌─────┐ │
 * │ │ERR_FREE                       ├──────►│ERROR│ │
 * │ │ ┌─────────────────┐           │       └──┬──┘ │
 * │ │ │READY            │           │          │    │
 * │ │ │  ┌─────────┐    │           │◄─────────┘    │
 * │ │ │  │RTD_READY│◄───┼──┐        │ error cleared │
 * │ │ │  └───┬─────┘    │  │        │               │
 * │ │ │ break│ ▲        │  │disable │               │
 * │ │ │      ▼ │no break│  │        │               │
 * │ │ │  ┌─────┴────┐   │ ┌┴──────┐ │               │
 * │ │ │  │RTD_STEADY│   │ │RUNNING│ │               │
 * │ │ │  └─┬────────┘   │ └───────┘ │               │
 * │ │ │    │RTD button  │  ▲        │               │
 * │ │ │    ▼            │  │ sound  │               │
 * │ │ │   ┌─────────┐   │  │finished│               │
 * │ │ │   │RTD_SOUND├───┼──┘        │               │
 * │ │ │   └─────────┘   │           │               │
 * │ │ └─────────────────┘           │               │
 * │ └───────────────────────────────┘               │
 * └─────────────────────────────────────────────────┘
 * ```
 */

/* macro ---------------------------------------------------------------------*/
#define STATES_CMD_START STATES_CMD_RUN
#define STATES_CMD_END STATES_CMD_MODE_CHANGE

/* type ----------------------------------------------------------------------*/
typedef uint16_t state_t;

/// @brief States state machine states.
enum states_state {
  STATE_ROOT = 0,
  STATE_ERR_FREE,
  STATE_READY,
  STATE_RTD_BLINK,
  STATE_RTD_STEADY,
  STATE_RTD_SOUND,
  STATE_RUNNING,
  STATE_ERROR,

  NUM_STATES,
};

enum states_update_type {
  STATES_COND_NONE,
  STATES_COND_ERR,
  STATES_COND_APPS,
  STATES_COND_BSE,
  STATES_COND_RTD_BUTTON,

  STATES_CMD_RUN,
  STATES_CMD_DISABLE,
  STATES_CMD_FAULT_RESET,
  STATES_CMD_MODE_CHANGE,
};

struct status_cmd {
  struct {
    uint16_t fl;
    uint16_t fr;
    uint16_t rl;
    uint16_t rr;
  } inv_ctrl_word;
};

/* exported variable ---------------------------------------------------------*/
ZBUS_CHAN_DECLARE(state_chan);
ZBUS_CHAN_DECLARE(status_cmd_chan);

/* function declaration ------------------------------------------------------*/
int states_cmd(enum states_update_type type);

/**
 * @brief Set the inverter direction.
 *
 * @param dir True for forward, false for reverse.
 */
int states_inv_dir(bool dir);

#endif  // STATES_H_
