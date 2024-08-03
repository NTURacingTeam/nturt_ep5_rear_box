#include "states/status_ctrl.h"

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/err.h>

// project includes
#include "define.h"
#include "msg.h"
#include "states.h"

LOG_MODULE_REGISTER(states_status_ctrl);

/* macro ---------------------------------------------------------------------*/
#define INV_STAT_MASK (INV_STAT_READY | INV_STAT_ENABLED | INV_STAT_FAULT)
#define INV_OKAY_MASK (INV_STAT_READY | INV_STAT_ENABLED)

/// @brief Inverter direction bit to be set or not for forward.
#define INV_DIR_L true
#define INV_DIR_R false

/* type ----------------------------------------------------------------------*/
enum inv_ctrl_bit {
  INV_CTRL_ENABLE = 3,
  INV_CTRL_FAULT_RESET = 5,
  INV_CTRL_DIR = 11,
};

enum inv_stat_bit {
  INV_STAT_READY = BIT(1),
  INV_STAT_ENABLED = BIT(2),
  INV_STAT_FAULT = BIT(3),
  INV_STAT_HV = BIT(4),
};

/* static function declaration -----------------------------------------------*/
static void status_data_chan_cb(const struct zbus_channel *chan);

/* static variable -----------------------------------------------------------*/
ZBUS_CHAN_DEFINE(status_cmd_chan, struct status_cmd, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

ZBUS_LISTENER_DEFINE(states_status_data_chan_listener, status_data_chan_cb);
ZBUS_CHAN_ADD_OBS(status_data_chan, states_status_data_chan_listener, 0);

/* function definition -------------------------------------------------------*/
void status_enable(struct status_ctrl *status, bool enable) {
  WRITE_BIT(status->cmd.inv_ctrl_word.fl, INV_CTRL_ENABLE, enable);
  WRITE_BIT(status->cmd.inv_ctrl_word.fr, INV_CTRL_ENABLE, enable);
  WRITE_BIT(status->cmd.inv_ctrl_word.rl, INV_CTRL_ENABLE, enable);
  WRITE_BIT(status->cmd.inv_ctrl_word.rr, INV_CTRL_ENABLE, enable);

  int ret;
  ret = zbus_chan_pub(&status_cmd_chan, &status->cmd, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish states command: %s", strerror(-ret));
  }
}

void status_inv_fault_reset(struct status_ctrl *status) {
  SET_BIT(status->cmd.inv_ctrl_word.fl, INV_CTRL_FAULT_RESET);
  SET_BIT(status->cmd.inv_ctrl_word.fr, INV_CTRL_FAULT_RESET);
  SET_BIT(status->cmd.inv_ctrl_word.rl, INV_CTRL_FAULT_RESET);
  SET_BIT(status->cmd.inv_ctrl_word.rr, INV_CTRL_FAULT_RESET);

  int ret;
  ret = zbus_chan_pub(&status_cmd_chan, &status->cmd, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish states command: %s", strerror(-ret));
  }

  k_work_reschedule(&status->inv_fault_reset_work, K_MSEC(100));
}

void status_inv_fault_reset_work(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct status_ctrl *status =
      CONTAINER_OF(dwork, struct status_ctrl, inv_fault_reset_work);

  CLEAR_BIT(status->cmd.inv_ctrl_word.fl, INV_CTRL_FAULT_RESET);
  CLEAR_BIT(status->cmd.inv_ctrl_word.fr, INV_CTRL_FAULT_RESET);
  CLEAR_BIT(status->cmd.inv_ctrl_word.rl, INV_CTRL_FAULT_RESET);
  CLEAR_BIT(status->cmd.inv_ctrl_word.rr, INV_CTRL_FAULT_RESET);

  int ret;
  ret = zbus_chan_pub(&status_cmd_chan, &status->cmd, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish states command: %s", strerror(-ret));
  }
}

void status_inv_dir(struct status_ctrl *status, bool dir) {
  WRITE_BIT(status->cmd.inv_ctrl_word.fl, INV_CTRL_DIR, INV_DIR_L ^ !dir);
  WRITE_BIT(status->cmd.inv_ctrl_word.fr, INV_CTRL_DIR, INV_DIR_R ^ !dir);
  WRITE_BIT(status->cmd.inv_ctrl_word.rl, INV_CTRL_DIR, INV_DIR_L ^ !dir);
  WRITE_BIT(status->cmd.inv_ctrl_word.rr, INV_CTRL_DIR, INV_DIR_R ^ !dir);

  int ret;
  ret = zbus_chan_pub(&status_cmd_chan, &status->cmd, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish states command: %s", strerror(-ret));
  }
}

/* static function definition ------------------------------------------------*/
static void status_data_chan_cb(const struct zbus_channel *chan) {
  const struct status_data *data = zbus_chan_const_msg(chan);

  bool okay;

  okay = data->acc == 1;
  err_set_errors(ERR_CODE_STAT_ACC, okay);

#if IS_ENABLED(FRONT_INVERTER)
  okay = data->inv.fl & INV_STAT_MASK == INV_OKAY_MASK;
  err_set_errors(ERR_CODE_STAT_INV_FL, okay);

  okay = data->inv.fr & INV_STAT_MASK == INV_OKAY_MASK;
  err_set_errors(ERR_CODE_STAT_INV_FR, okay);
#endif  // FRONT_INVERTER

  okay = (data->inv.rl & INV_STAT_MASK) == INV_OKAY_MASK;
  err_set_errors(ERR_CODE_STAT_INV_RL, okay);

  okay = (data->inv.rr & INV_STAT_MASK) == INV_OKAY_MASK;
  err_set_errors(ERR_CODE_STAT_INV_RR, okay);
}
