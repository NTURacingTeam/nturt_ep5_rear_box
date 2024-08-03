#include "ctrl.h"

// glibc includes
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>

// simulink includes
#include "vehicle_control.h"

// project includes
#include "define.h"
#include "msg.h"
#include "sensors.h"
#include "states.h"

LOG_MODULE_REGISTER(ctrl);

/* macro ---------------------------------------------------------------------*/
/// @brief State machine initial state. Used to prevent state entry and exit
/// functions to be called repeatedly since self transition is allowed and will
/// re-execute the entry and exit functions. If @ref CTRL_MDOE_INITIAL was set
/// to @ref CTRL_MODE_LOW, its entry and exit functions would be called twice
/// when control module is started.
#define CTRL_MDOE_INITIAL NUM_CTRL_MODES

#define CTRL_THREAD_PERIOD K_MSEC(10)

/* type ----------------------------------------------------------------------*/
struct ctrl {
  struct smf_ctx smf_ctx;
  enum ctrl_mode mode;
  bool enabled;

  struct ctrl_data data;

  struct sensor_data sensor_data;
  struct inv_data inv_data;
  struct imu_data imu_data;
};

/* static function declaration -----------------------------------------------*/
static void ctrl_thread(void *arg1, void *arg2, void *arg3);

static void control_step(struct ctrl_data *data);

static void states_run(void *obj);

static void low_entry(void *obj);
static void low_exit(void *obj);

static void high_entry(void *obj);
static void high_exit(void *obj);

static void reversed_entry(void *obj);
static void reversed_exit(void *obj);

/* static varaibles ----------------------------------------------------------*/
static const struct device *dash_leds = DEVICE_DT_GET(DT_NODELABEL(dash_leds));

static struct ctrl ctrl;

const static struct zbus_channel *const ctrl_input_chans[] = {
    &sensor_data_chan,
    &inv_data_chan,
    &imu_data_chan,
};

static void *const ctrl_input_data[] = {
    &ctrl.sensor_data,
    &ctrl.inv_data,
    &ctrl.imu_data,
};

static const struct smf_state smf_states[] = {
    [CTRL_MODE_LOW] =
        SMF_CREATE_STATE(low_entry, states_run, low_exit, NULL, NULL),
    [CTRL_MODE_HIGH] =
        SMF_CREATE_STATE(high_entry, states_run, high_exit, NULL, NULL),
    [CTRL_MODE_REVERSED] =
        SMF_CREATE_STATE(reversed_entry, states_run, reversed_exit, NULL, NULL),
    [CTRL_MDOE_INITIAL] = SMF_CREATE_STATE(NULL, states_run, NULL, NULL, NULL),
};

ZBUS_CHAN_DEFINE(ctrl_mode_chan, enum ctrl_mode, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

ZBUS_CHAN_DEFINE(ctrl_data_chan, struct ctrl, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(0));

ZBUS_SUBSCRIBER_DEFINE(ctrl_input_sub, 10);
ZBUS_CHAN_ADD_OBS(sensor_data_chan, ctrl_input_sub, 0);
ZBUS_CHAN_ADD_OBS(inv_data_chan, ctrl_input_sub, 0);
ZBUS_CHAN_ADD_OBS(imu_data_chan, ctrl_input_sub, 0);

K_THREAD_DEFINE(ctrl_thread_tid, 4096, ctrl_thread, &ctrl, NULL, NULL,
                CONFIG_NTURT_CTRL_THREAD_PRIORITY, 0, 1);

/* function definition -------------------------------------------------------*/
int ctrl_mode_next() {
  if (ctrl.enabled) {
    LOG_ERR("Can only change control mode when control is disabled");

    return -EINVAL;
  }

  ctrl.mode = (ctrl.mode + 1) % NUM_CTRL_MODES;
  smf_run_state(&ctrl.smf_ctx);

  return 0;
}

int ctrl_enable() {
  ctrl.enabled = true;

  return 0;
}

int ctrl_disable() {
  ctrl.enabled = false;

  return 0;
}

/* static function definition ------------------------------------------------*/
static void ctrl_thread(void *arg1, void *arg2, void *arg3) {
  (void)arg2;
  (void)arg3;

  struct ctrl *ctrl = arg1;

  int ret;
  k_timepoint_t next;

  states_inv_dir(true);

  smf_set_initial(&ctrl->smf_ctx, &smf_states[CTRL_MDOE_INITIAL]);
  smf_run_state(&ctrl->smf_ctx);

  while (true) {
    next = sys_timepoint_calc(CTRL_THREAD_PERIOD);

    while (true) {
      const struct zbus_channel *chan;
      bool read[ARRAY_SIZE(ctrl_input_chans)] = {0};

      ret = zbus_sub_wait(&ctrl_input_sub, &chan, K_NO_WAIT);
      if (ret == 0) {
        for (size_t i = 0; i < ARRAY_SIZE(ctrl_input_chans); i++) {
          if (chan == ctrl_input_chans[i]) {
            if (read[i]) {
              break;
            }

            ret = zbus_chan_read(chan, ctrl_input_data[i], K_MSEC(5));
            if (ret < 0) {
              LOG_ERR("Failed to read from %s: %s", zbus_chan_name(chan),
                      strerror(-ret));
            }

            read[i] = true;
            break;
          }
        }

      } else if (ret == -ENOMSG) {
        break;
      } else {
        LOG_ERR("Failed to wait for ctrl_input_sub: %s", strerror(-ret));
      }
    }

    // update speed
    /// @todo: Kalman filter
    ctrl->data.speed = (ctrl->inv_data.rl.speed + ctrl->inv_data.rr.speed) / 2;

    if (ctrl->enabled) {
      control_step(&ctrl->data);
    }

    ret = zbus_chan_pub(&ctrl_data_chan, &ctrl->data, K_MSEC(5));
    if (ret < 0) {
      LOG_ERR("Failed to write to ctrl_data_chan: %s", strerror(-ret));
    }

    k_sleep(sys_timepoint_timeout(next));
  }
}

static void control_step(struct ctrl_data *data) {
  /// @todo: use lookup table to map
  rtU.gas = ctrl.sensor_data.apps.travel;
  rtU.whl_spd_rl = ctrl.inv_data.rl.speed;
  rtU.whl_spd_rr = ctrl.inv_data.rr.speed;
  rtU.v_x = data->speed;
  rtU.a_y = ctrl.imu_data.accel.y;
  rtU.yaw_rate = ctrl.imu_data.gyro.z;

  vehicle_control_step();

  data->rl.torque_cmd = rtY.trq_rl;
  data->rr.torque_cmd = rtY.trq_rr;
}

static void states_run(void *obj) {
  struct ctrl *ctrl = obj;

  smf_set_state(&ctrl->smf_ctx, &smf_states[ctrl->mode]);
  zbus_chan_pub(&ctrl_mode_chan, &ctrl->mode, K_MSEC(5));
}

static void low_entry(void *obj) {
  (void)obj;

  vehicle_control_initialize();
}
static void low_exit(void *obj) {
  (void)obj;

  // vehicle_control_terminate();
}

static void high_entry(void *obj) {
  (void)obj;

  vehicle_control_initialize();
  led_on(dash_leds, LED_NUM_MODE);
}
static void high_exit(void *obj) {
  (void)obj;

  // vehicle_control_terminate();
  led_off(dash_leds, LED_NUM_MODE);
}

static void reversed_entry(void *obj) {
  (void)obj;

  vehicle_control_initialize();
  states_inv_dir(false);
  led_blink(dash_leds, LED_NUM_MODE, 0, 0);
}
static void reversed_exit(void *obj) {
  (void)obj;

  // vehicle_control_terminate();
  states_inv_dir(true);
  led_off(dash_leds, LED_NUM_MODE);
}
