#ifndef STATES_H_
#define STATES_H_

// glibc includes
#include <stdint.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/rear_box/states.h>

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
#define STATES_CMD_START (__STATES_CMD_START + 1)
#define STATES_CMD_END (__STATES_CMD_END - 1)

/* type ----------------------------------------------------------------------*/
enum states_update_type {
  STATES_UPDATE_NONE = 0,

  STATES_COND_ERR,
  STATES_COND_APPS,
  STATES_COND_BSE,

  __STATES_CMD_START,

  STATES_CMD_RTD,
  STATES_CMD_RUN,
  STATES_CMD_DISABLE,

  __STATES_CMD_END,

  NUM_STATES_UPDATE_TYPE,
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
ZBUS_CHAN_DECLARE(status_cmd_chan);

/* function declaration ------------------------------------------------------*/
states_t states_get_states();

void states_cmd(enum states_update_type cmd);

/**
 * @brief Set the inverter direction.
 *
 * @param dir True for forward, false for reverse.
 */
void states_inv_dir(bool dir);

void states_inv_reset();

#endif  // STATES_H_
