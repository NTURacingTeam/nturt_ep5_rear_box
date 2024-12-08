#ifndef CTRL_H_
#define CTRL_H_

// zephyr includes
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/rear_box/ctrl.h>

/* types ---------------------------------------------------------------------*/
struct ctrl_data {
  float speed;

  struct {
    float torque_cmd;
  } fl, fr, rl, rr;
};

/* function definition -------------------------------------------------------*/
ZBUS_CHAN_DECLARE(ctrl_data_chan);

/* function declaration ------------------------------------------------------*/
enum ctrl_mode ctrl_mode_get();

void ctrl_mode_set(enum ctrl_mode mode);

void ctrl_enable();
void ctrl_disable();

#endif  // CTRL_H_
