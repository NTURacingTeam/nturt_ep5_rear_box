#include "nturt/sensors.h"

// glibc includes
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// zephyr includes
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>

// project includes
#include "nturt/init.h"
#include "nturt/nturt.h"
#include "nturt/states.h"

LOG_MODULE_REGISTER(sensors);

/* macros --------------------------------------------------------------------*/
/// @brief APPS1 minimum value in degrees.
#define APPS1_DEG_MIN 0.0F

/// @brief APPS1 maximum value in degrees.
#define APPS1_DEG_MAX 20.0F

/// @brief APPS1 value scale for normalization in degrees.
#define APPS1_DEG_SCALE (APPS1_DEG_MAX - APPS1_DEG_MIN)

/// @brief APPS2 minimum value in degrees.
#define APPS2_DEG_MIN 0.0F

/// @brief APPS2 maximum value in degrees.
#define APPS2_DEG_MAX -20.0F

/// @brief APPS2 value scale for normalization in degrees.
#define APPS2_DEG_SCALE (APPS2_DEG_MAX - APPS2_DEG_MIN)

/// @brief APPS value tolerance before it's considered out of range.
#define APPS_VAL_TOL 0.05F

/// @brief APPS plausibility check value tolerance.
#define APPS_PLAUS_VAL_TOL 0.1F

/// @brief APPS plausibility check time tolerance.
#define APPS_PLAUS_TIME_TOL K_MSEC(100)

/// @brief Maximum brake pressure sensor value in hPa.
#define BSE_PRES_MAX 68948.0F

#define PEDAL_PLAUS_ENG_THRES 0.25F

#define PEDAL_PLAUS_DISENG_THRES 0.05F

/// @brief Sensors thread period.
#define SENSORS_THREAD_PERIOD K_MSEC(10)

/**
 * @brief Desginated initializer for @ref sensor_tol.
 *
 * @param NAME Name of the sensor tolerant read structure.
 * @param NODE_ID Node ID of the sensor device.
 * @param TOL Tolerance, i.e. number of consecutive failed reads before setting
 * error.
 * @param ERR Error code corresponding to the sensor.
 */
#define SENSOR_TOL_DEFINE(NAME, NODE_ID, TOL, ERR) \
  struct sensor_tol NAME = {                       \
      .dev = DEVICE_DT_GET(NODE_ID),               \
      .tol = TOL,                                  \
      .err = ERR,                                  \
      .fail = 0,                                   \
  }

/* types ---------------------------------------------------------------------*/
enum pedal_plaus_state {
  PLAUS_INACTIVE,
  PLAUS_ACTIVE,
};

/// @brief Sensor tolerant read structure.
struct sensor_tol {
  /// @brief Sensor device.
  const struct device *dev;

  /// @brief Tolerance, i.e. number of consecutive failed reads before setting
  /// error.
  const int tol;

  /// @brief Error code corresponding to the sensor.
  const enum err_code err;

  /// @brief Number of consecutive failed reads.
  int fail;
};

/* static function declaration -----------------------------------------------*/
static void buttons_cb(struct input_event *evt);

/// @brief Initialization function for sensors module.
static int init();

static void pedal_plaus_inactive_entry(void *state);

static void pedal_plaus_inactive_run(void *state);

static void pedal_plaus_active_entry(void *state);

static void pedal_plaus_active_run(void *state);

static void sensors_thread(void *arg1, void *arg2, void *arg3);

/**
 * @brief Read sensor data with tolerance. Sets error if the sensor fails
 * consecutively. Does nothing if the sensor error is set and returns false.
 *
 * @param tol Sensor tolerant read structure.
 * @param chan Sensor channel.
 * @param val Sensor value.
 * @return 0 if read successful, negative error code otherwise.
 */
static int sensor_tol_chan_read(struct sensor_tol *tol,
                                enum sensor_channel chan,
                                struct sensor_value *val);

static void set_apps_zero_pos();

static int apps_val_check(float raw, float min, float max, float *val);

/* static varaibles ----------------------------------------------------------*/
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));

static const struct gpio_dt_spec accel_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(accel_micro), gpios);
static const struct gpio_dt_spec break_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(break_micro), gpios);

static struct sensor_data sensor_data;

static bool apps_set_zero = false;

const static struct smf_state pedal_plaus_states[] = {
    [PLAUS_INACTIVE] = SMF_CREATE_STATE(
        pedal_plaus_inactive_entry, pedal_plaus_inactive_run, NULL, NULL, NULL),
    [PLAUS_ACTIVE] = SMF_CREATE_STATE(pedal_plaus_active_entry,
                                      pedal_plaus_active_run, NULL, NULL, NULL),
};

static SENSOR_TOL_DEFINE(steer, DT_NODELABEL(steer), 5, ERR_CODE_STEER);

static SENSOR_TOL_DEFINE(apps1, DT_NODELABEL(apps1), 5, ERR_CODE_APPS1);
static SENSOR_TOL_DEFINE(apps2, DT_NODELABEL(apps2), 5, ERR_CODE_APPS2);

static SENSOR_TOL_DEFINE(break_press_f, DT_NODELABEL(break_press_f), 5,
                         ERR_CODE_BSE_F);
static SENSOR_TOL_DEFINE(break_press_r, DT_NODELABEL(break_press_r), 5,
                         ERR_CODE_BSE_R);

static SENSOR_TOL_DEFINE(susp_dive_r, DT_NODELABEL(susp_dive_r), 5,
                         ERR_CODE_SUSP_DIVE);
static SENSOR_TOL_DEFINE(susp_roll_r, DT_NODELABEL(susp_roll_r), 5,
                         ERR_CODE_SUSP_ROLL);

ZBUS_CHAN_DEFINE(sensor_data_chan, struct sensor_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

SYS_INIT(init, APPLICATION, INIT_SENSORS_INIT_PRIORITY);

// K_THREAD_DEFINE(sensors_thread_tid, 1024, sensors_thread, NULL, NULL, NULL,
// 10,
//                 0, 1);

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt) {
  if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_ACCEL ||
      evt->code != INPUT_KEY_BREAK) {
    return;
  }

  if (evt->code == INPUT_KEY_ACCEL) {
    sensor_data.accel_micro = evt->value;

    if (!evt->value && !apps_set_zero) {
      set_apps_zero_pos();
    }
  } else {
    sensor_data.break_micro = evt->value;
    led_set_brightness(leds, LED_NUM_BREAK_LIGHT, evt->value ? 100 : 0);
  }
}

static int init() {
  if (!device_is_ready(steer.dev)) {
    states_set_errors(ERR_CODE_STEER, true);
    LOG_ERR("Steering encoder not ready");
  }

  if (!device_is_ready(apps1.dev)) {
    states_set_errors(ERR_CODE_APPS1, true);
    LOG_ERR("APPS1 not ready");
  }

  if (!device_is_ready(apps2.dev)) {
    states_set_errors(ERR_CODE_APPS2, true);
    LOG_ERR("APPS2 not ready");
  }

  if (!device_is_ready(break_press_f.dev)) {
    states_set_errors(ERR_CODE_BSE_F, true);
    LOG_ERR("BSE front not ready");
  }

  if (!device_is_ready(break_press_r.dev)) {
    states_set_errors(ERR_CODE_BSE_R, true);
    LOG_ERR("BSE rear not ready");
  }

  if (!device_is_ready(susp_dive_r.dev)) {
    states_set_errors(ERR_CODE_SUSP_DIVE, true);
    LOG_ERR("Suspension dive sensor not ready");
  }

  if (!device_is_ready(susp_roll_r.dev)) {
    states_set_errors(ERR_CODE_SUSP_ROLL, true);
    LOG_ERR("Suspension roll sensor not ready");
  }

  // set APPS to zero position
  if (!(states_get_errors() & (ERR_CODE_APPS1 | ERR_CODE_APPS2))) {
    if (!gpio_pin_get_dt(&accel_micro)) {
      set_apps_zero_pos();
    } else {
      states_set_errors(ERR_CODE_APPS_PLAUS, true);
      LOG_WRN("Accelerator engaged during startup");
    }
  }

  // set break light initial state
  led_set_brightness(leds, LED_NUM_BREAK_LIGHT, gpio_pin_get_dt(&break_micro));

  return 0;
}

static void pedal_plaus_inactive_entry(void *state) {
  (void)state;

  states_set_errors(ERR_CODE_PEDAL_PLAUS, false);
}

static void pedal_plaus_inactive_run(void *state) {
  if (sensor_data.break_micro && sensor_data.apps > PEDAL_PLAUS_ENG_THRES) {
    sensor_data.apps = 0;
    smf_set_state(SMF_CTX(state), &pedal_plaus_states[PLAUS_ACTIVE]);
  }
}

static void pedal_plaus_active_entry(void *state) {
  (void)state;

  states_set_errors(ERR_CODE_PEDAL_PLAUS, true);
}

static void pedal_plaus_active_run(void *state) {
  if (sensor_data.apps < PEDAL_PLAUS_DISENG_THRES) {
    smf_set_state(SMF_CTX(state), &pedal_plaus_states[PLAUS_INACTIVE]);
  } else {
    sensor_data.apps = 0;
  }
}

static void sensors_thread(void *arg1, void *arg2, void *arg3) {
  (void)arg1;
  (void)arg2;
  (void)arg3;

  int ret;
  k_timepoint_t next;
  struct sensor_value val;

  float apps1_val, apps2_val;
  k_timepoint_t apps_plaus_time = sys_timepoint_calc(APPS_PLAUS_TIME_TOL);

  static struct smf_ctx pedal_plaus;

  smf_set_initial(&pedal_plaus, &pedal_plaus_states[PLAUS_INACTIVE]);

  while (true) {
    next = sys_timepoint_calc(SENSORS_THREAD_PERIOD);

    ret = sensor_tol_chan_read(&steer, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensor_data.steer = sensor_value_to_float(&val);
    }

    ret = sensor_tol_chan_read(&apps1, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensor_data.apps1_raw = sensor_value_to_float(&val);

      if (apps_set_zero) {
        ret = apps_val_check(sensor_data.apps1_raw, APPS1_DEG_MIN,
                             APPS1_DEG_MAX, &apps1_val);
        if (ret < 0) {
          states_set_errors(ERR_CODE_APPS1, true);
          LOG_ERR("APPS1 value out of range: %f", apps1_val);
        }
      }
    }

    ret = sensor_tol_chan_read(&apps2, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensor_data.apps2_raw = sensor_value_to_float(&val);

      if (apps_set_zero) {
        ret = apps_val_check(sensor_data.apps2_raw, APPS2_DEG_MIN,
                             APPS2_DEG_MAX, &apps2_val);
        if (ret < 0) {
          states_set_errors(ERR_CODE_APPS2, true);
          LOG_ERR("APPS2 value out of range: %f", apps2_val);
        }
      }
    }

    // apps plausibility check
    if (apps_set_zero &&
        !(states_get_errors() & (ERR_CODE_APPS1 | ERR_CODE_APPS2))) {
      if (fabsf(apps1_val - apps2_val) > APPS_PLAUS_VAL_TOL) {
        if (sys_timepoint_expired(apps_plaus_time)) {
          sensor_data.apps = 0;
          states_set_errors(ERR_CODE_APPS_PLAUS, true);
          LOG_ERR("APPS plausibility check failed: %f, %f", apps1_val,
                  apps2_val);
        }
      } else {
        sensor_data.apps = (apps1_val + apps2_val) / 2;
        apps_plaus_time = sys_timepoint_calc(APPS_PLAUS_TIME_TOL);
      }
    }

    ret = sensor_tol_chan_read(&break_press_f, SENSOR_CHAN_PRESS, &val);
    if (ret == 0) {
      sensor_data.bse_f_raw = sensor_value_to_float(&val);
    }

    // bse plaussibility check done by the driver
    ret = sensor_tol_chan_read(&break_press_r, SENSOR_CHAN_PRESS, &val);
    if (ret == 0) {
      sensor_data.bse_r_raw = sensor_value_to_float(&val);
    }

    if (!(states_get_errors() & (ERR_CODE_BSE_F | ERR_CODE_BSE_R))) {
      sensor_data.bse =
          (sensor_data.bse_f_raw + sensor_data.bse_r_raw) / 2 / BSE_PRES_MAX;
    } else {
      sensor_data.bse = 0;
    }

    // pedal plausibility check
    if (apps_set_zero &&
        !(states_get_errors() &
          (ERR_CODE_APPS1 | ERR_CODE_APPS2 | ERR_CODE_APPS_PLAUS))) {
      smf_run_state(&pedal_plaus);
    }

    ret = sensor_tol_chan_read(&susp_dive_r, SENSOR_CHAN_DISTANCE, &val);
    if (ret == 0) {
      sensor_data.susp_dive_r = sensor_value_to_float(&val);
    }

    ret = sensor_tol_chan_read(&susp_roll_r, SENSOR_CHAN_DISTANCE, &val);
    if (ret == 0) {
      sensor_data.susp_roll_r = sensor_value_to_float(&val);
    }

    ret = zbus_chan_pub(&sensor_data_chan, &sensor_data, K_MSEC(5));
    if (ret < 0) {
      LOG_ERR("Failed to publish sensor data: %s", strerror(-ret));
    }

    k_sleep(sys_timepoint_timeout(next));
  }
}

static int apps_val_check(float raw, float min, float max, float *val) {
  *val = (raw - min) / (max - min);

  if (*val < -APPS_VAL_TOL) {
    return -EINVAL;
  } else if (*val < 0) {
    *val = 0;
  } else if (*val > 1 + APPS_VAL_TOL) {
    return -EINVAL;
  } else if (*val > 1) {
    *val = 1;
  }

  return 0;
}

static int sensor_tol_chan_read(struct sensor_tol *tol,
                                enum sensor_channel chan,
                                struct sensor_value *val) {
  if (states_get_errors() & tol->err) {
    return -ENODEV;
  }

  int ret;
  if ((ret = sensor_sample_fetch_chan(tol->dev, chan)) < 0 ||
      (ret = sensor_channel_get(tol->dev, chan, val)) < 0) {
    if (++tol->fail == tol->tol) {
      states_set_errors(tol->err, true);
      LOG_ERR("Sensor %s failed after %d consecutive tries", tol->dev->name,
              tol->fail);
    }

    return ret;
  } else {
    tol->fail = 0;
    return 0;
  }
}

static void set_apps_zero_pos() {
  if (states_get_errors() & (ERR_CODE_APPS1 | ERR_CODE_APPS2)) {
    return;
  }

  struct sensor_value val;
  sensor_sample_fetch_chan(apps1.dev, SENSOR_CHAN_ROTATION);
  sensor_channel_get(apps1.dev, SENSOR_CHAN_ROTATION, &val);

  val.val1 = -val.val1;
  val.val2 = -val.val2;
  sensor_attr_set(apps1.dev, SENSOR_CHAN_ROTATION, SENSOR_ATTR_OFFSET, &val);

  sensor_sample_fetch_chan(apps2.dev, SENSOR_CHAN_ROTATION);
  sensor_channel_get(apps2.dev, SENSOR_CHAN_ROTATION, &val);

  val.val1 = -val.val1;
  val.val2 = -val.val2;
  sensor_attr_set(apps2.dev, SENSOR_CHAN_ROTATION, SENSOR_ATTR_OFFSET, &val);

  states_set_errors(ERR_CODE_APPS_PLAUS, false);
  apps_set_zero = true;
  LOG_INF("Set APPS to zero position");
}
