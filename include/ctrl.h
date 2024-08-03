#ifndef CTRL_H_
#define CTRL_H_

// zephyr includes
#include <zephyr/zbus/zbus.h>

/* types ---------------------------------------------------------------------*/
enum ctrl_mode {
  CTRL_MODE_LOW = 0,
  CTRL_MODE_HIGH,
  CTRL_MODE_REVERSED,

  NUM_CTRL_MODES,
};

struct ctrl_data {
  float speed;

  struct {
    float torque_cmd;
  } fl, fr, rl, rr;
};

/* function definition -------------------------------------------------------*/
ZBUS_CHAN_DECLARE(ctrl_mode_chan);
ZBUS_CHAN_DECLARE(ctrl_data_chan);

/* function declaration ------------------------------------------------------*/
int ctrl_mode_next();

int ctrl_enable();
int ctrl_disable();

#endif  // CTRL_H_
