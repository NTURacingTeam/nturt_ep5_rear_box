#include "sensors.h"

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
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/err.h>
#include <nturt/sensors.h>

// project includes
#include "define.h"

LOG_MODULE_REGISTER(sensors);

/* macro ---------------------------------------------------------------------*/
/// @brief APPS1 minimum value in degrees.
#define APPS1_DEG_MIN 0.0F

/// @brief APPS1 maximum value in degrees.
#define APPS1_DEG_MAX 12.0F

/// @brief APPS1 value scale for normalization in degrees.
#define APPS1_DEG_SCALE (APPS1_DEG_MAX - APPS1_DEG_MIN)

/// @brief APPS2 minimum value in degrees.
#define APPS2_DEG_MIN 0.0F

/// @brief APPS2 maximum value in degrees.
#define APPS2_DEG_MAX -12.0F

/// @brief APPS2 value scale for normalization in degrees.
#define APPS2_DEG_SCALE (APPS2_DEG_MAX - APPS2_DEG_MIN)

/// @brief APPS value tolerance before it's considered out of range.
#define APPS_VAL_TOL 0.05F

/// @brief APPS plausibility check value tolerance.
#define APPS_PLAUS_VAL_TOL 0.1F

/// @brief APPS plausibility check time tolerance.
#define APPS_PLAUS_TIME_TOL K_MSEC(100)

/// @brief Maximum brake pressure sensor value in hPa.
#define BSE_PRES_MAX 70000.0F

#define PEDAL_PLAUS_ENG_THRES 0.25F

#define PEDAL_PLAUS_DISENG_THRES 0.05F

/// @brief Sensors thread period.
#define SENSORS_THREAD_PERIOD K_MSEC(10)

/* type ----------------------------------------------------------------------*/
enum pedal_plaus_state {
  PLAUS_INACTIVE,
  PLAUS_ACTIVE,
};

struct apps {
  float val1, val2;
  bool set_zero;
  k_timepoint_t plaus_time;
};

struct pedals {
  struct smf_ctx smf_ctx;
  struct apps apps;
};

struct sensors {
  struct pedals pedals;
  struct sensor_data data;
};

/* static function declaration -----------------------------------------------*/
static void buttons_cb(struct input_event *evt);

/// @brief Initialization function for sensors module.
static int init();

static void pedal_plaus_inactive_run(void *obj);

static void pedal_plaus_active_entry(void *obj);
static void pedal_plaus_active_run(void *obj);
static void pedal_plaus_active_exit(void *obj);

static void sensors_thread(void *arg1, void *arg2, void *arg3);

static float encoder_val_offset(float val);
static void encoder_set_zero_pos(const struct device *dev);

static void apps_set_zero_pos(struct pedals *pedals);

static int apps_val_check(float raw, float min, float max, float *val);

/* static varaibles ----------------------------------------------------------*/
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));
static const struct device *dash_leds = DEVICE_DT_GET(DT_NODELABEL(dash_leds));

static const struct gpio_dt_spec apps_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(apps_micro), gpios);
static const struct gpio_dt_spec bse_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bse_micro), gpios);

const static struct smf_state pedal_states[] = {
    [PLAUS_INACTIVE] =
        SMF_CREATE_STATE(NULL, pedal_plaus_inactive_run, NULL, NULL, NULL),
    [PLAUS_ACTIVE] =
        SMF_CREATE_STATE(pedal_plaus_active_entry, pedal_plaus_active_run,
                         pedal_plaus_active_exit, NULL, NULL),
};

static struct sensors sensors;

static SENSOR_TOL_DEFINE(steer, DT_NODELABEL(steer), 5, ERR_CODE_STEER);

static SENSOR_TOL_DEFINE(apps1, DT_NODELABEL(apps1), 5, ERR_CODE_APPS1);
static SENSOR_TOL_DEFINE(apps2, DT_NODELABEL(apps2), 5, ERR_CODE_APPS2);

static SENSOR_TOL_DEFINE(bse_f, DT_NODELABEL(bse_f), 5, ERR_CODE_BSE_F);
static SENSOR_TOL_DEFINE(bse_r, DT_NODELABEL(bse_r), 5, ERR_CODE_BSE_R);

static SENSOR_TOL_DEFINE(susp_dive_r, DT_NODELABEL(susp_dive_r), 5,
                         ERR_CODE_SUSP_DIVE);
static SENSOR_TOL_DEFINE(susp_roll_r, DT_NODELABEL(susp_roll_r), 5,
                         ERR_CODE_SUSP_ROLL);

ZBUS_CHAN_DEFINE(sensor_data_chan, struct sensor_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);

SYS_INIT(init, APPLICATION, CONFIG_NTURT_SENSORS_INIT_PRIORITY);

K_THREAD_DEFINE(sensors_thread_tid, 1024, sensors_thread, &sensors, NULL, NULL,
                CONFIG_NTURT_SENSORS_THREAD_PRIORITY, 0, 1);

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt) {
  if (evt->type != INPUT_EV_KEY) {
    return;
  }

  switch (evt->code) {
    case INPUT_KEY_APPS:
      sensors.data.apps.engaged = evt->value;

      if (!evt->value && !sensors.pedals.apps.set_zero) {
        apps_set_zero_pos(&sensors.pedals);
      }
      break;

    case INPUT_KEY_BSE:
      sensors.data.bse.engaged = evt->value;
      led_set_brightness(leds, LED_NUM_BRAKE_LIGHT, evt->value ? 100 : 0);
      break;

    case INPUT_KEY_HOME:
      encoder_set_zero_pos(steer.dev);

      LOG_INF("Set steering to zero position");
      break;

    default:
      break;
  }
}

static int init() {
  if (!device_is_ready(steer.dev)) {
    err_set_errors(ERR_CODE_STEER, true);
    LOG_ERR("Steering encoder not ready");
  }

  if (!device_is_ready(apps1.dev)) {
    err_set_errors(ERR_CODE_APPS1, true);
    LOG_ERR("APPS1 not ready");
  }

  if (!device_is_ready(apps2.dev)) {
    err_set_errors(ERR_CODE_APPS2, true);
    LOG_ERR("APPS2 not ready");
  }

  if (!device_is_ready(bse_f.dev)) {
    err_set_errors(ERR_CODE_BSE_F, true);
    LOG_ERR("BSE front not ready");
  }

  if (!device_is_ready(bse_r.dev)) {
    err_set_errors(ERR_CODE_BSE_R, true);
    LOG_ERR("BSE rear not ready");
  }

  if (!device_is_ready(susp_dive_r.dev)) {
    err_set_errors(ERR_CODE_SUSP_DIVE, true);
    LOG_ERR("Suspension dive sensor not ready");
  }

  if (!device_is_ready(susp_roll_r.dev)) {
    err_set_errors(ERR_CODE_SUSP_ROLL, true);
    LOG_ERR("Suspension roll sensor not ready");
  }

  sensors.data.apps.engaged = gpio_pin_get_dt(&apps_micro);

  // set APPS to zero position
  if (!(err_get_errors() & (ERR_CODE_APPS1 | ERR_CODE_APPS2))) {
    if (!sensors.data.apps.engaged) {
      apps_set_zero_pos(&sensors.pedals);
    } else {
      err_set_errors(ERR_CODE_APPS_PLAUS, true);
      LOG_WRN("Accelerator engaged during startup");
    }
  }

  sensors.data.bse.engaged = gpio_pin_get_dt(&bse_micro);

  // set break light initial state
  led_set_brightness(leds, LED_NUM_BRAKE_LIGHT,
                     sensors.data.bse.engaged ? 100 : 0);

  smf_set_initial(SMF_CTX(&sensors.pedals), &pedal_states[PLAUS_INACTIVE]);

  return 0;
}

static void pedal_plaus_inactive_run(void *obj) {
  struct pedals *pedals = obj;
  struct sensors *sensors = CONTAINER_OF(pedals, struct sensors, pedals);

  if (sensors->data.bse.engaged &&
      sensors->data.apps.travel > PEDAL_PLAUS_ENG_THRES) {
    sensors->data.apps.travel = 0;
    smf_set_state(SMF_CTX(pedals), &pedal_states[PLAUS_ACTIVE]);
  }
}

static void pedal_plaus_active_entry(void *obj) {
  (void)obj;

  err_set_errors(ERR_CODE_PEDAL_PLAUS, true);
  led_on(dash_leds, LED_NUM_PEDAL_PLAUS);
}
static void pedal_plaus_active_run(void *obj) {
  struct pedals *pedals = obj;
  struct sensors *sensors = CONTAINER_OF(pedals, struct sensors, pedals);

  if (sensors->data.apps.travel < PEDAL_PLAUS_DISENG_THRES) {
    smf_set_state(SMF_CTX(pedals), &pedal_states[PLAUS_INACTIVE]);
  } else {
    sensors->data.apps.travel = 0;
  }
}
static void pedal_plaus_active_exit(void *obj) {
  (void)obj;

  err_set_errors(ERR_CODE_PEDAL_PLAUS, false);
  led_off(dash_leds, LED_NUM_PEDAL_PLAUS);
}

static void sensors_thread(void *arg1, void *arg2, void *arg3) {
  (void)arg2;
  (void)arg3;

  struct sensors *sensors = arg1;

  int ret;
  k_timepoint_t next;
  struct sensor_value val;

  sensors->pedals.apps.plaus_time = sys_timepoint_calc(APPS_PLAUS_TIME_TOL);

  while (true) {
    next = sys_timepoint_calc(SENSORS_THREAD_PERIOD);

    ret = sensor_tol_chan_read(&steer, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensors->data.steer = encoder_val_offset(sensor_value_to_float(&val));
    }

    // amt21 encoders are unstable when read repeatly (steer and apps1 encoder
    // share the same bus)
    k_sleep(K_MSEC(1));

    struct pedals *pedals = &sensors->pedals;
    struct apps *apps = &pedals->apps;

    ret = sensor_tol_chan_read(&apps1, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensors->data.apps.raw[0] =
          encoder_val_offset(sensor_value_to_float(&val));

      if (apps->set_zero) {
        ret = apps_val_check(sensors->data.apps.raw[0], APPS1_DEG_MIN,
                             APPS1_DEG_MAX, &apps->val1);
        if (ret < 0) {
          sensor_tol_fail_report(&apps1);
          LOG_WRN("APPS1 value out of range: %f",
                  (double)sensors->data.apps.raw[0]);
        }
      }
    }

    ret = sensor_tol_chan_read(&apps2, SENSOR_CHAN_ROTATION, &val);
    if (ret == 0) {
      sensors->data.apps.raw[1] =
          encoder_val_offset(sensor_value_to_float(&val));

      if (apps->set_zero) {
        ret = apps_val_check(sensors->data.apps.raw[1], APPS2_DEG_MIN,
                             APPS2_DEG_MAX, &apps->val2);
        if (ret < 0) {
          sensor_tol_fail_report(&apps2);
          LOG_WRN("APPS2 value out of range: %f",
                  (double)sensors->data.apps.raw[1]);
        }
      }
    }

    // apps plausibility check
    if (apps->set_zero &&
        !(err_get_errors() &
          (ERR_CODE_APPS1 | ERR_CODE_APPS2 | ERR_CODE_APPS_PLAUS))) {
      if (fabsf(apps->val1 - apps->val2) > APPS_PLAUS_VAL_TOL) {
        LOG_WRN("APPS plausibility check failed: %f, %f", (double)apps->val1,
                (double)apps->val2);

        if (sys_timepoint_expired(apps->plaus_time)) {
          sensors->data.apps.travel = 0;
          err_set_errors(ERR_CODE_APPS_PLAUS, true);
          LOG_ERR("APPS plausibility failed lasted over 100 ms");
        }

      } else {
        sensors->data.apps.travel = (apps->val1 + apps->val2) / 2;
        sensors->pedals.apps.plaus_time =
            sys_timepoint_calc(APPS_PLAUS_TIME_TOL);
      }
    }

    ret = sensor_tol_chan_read(&bse_f, SENSOR_CHAN_PRESS, &val);
    if (ret == 0) {
      sensors->data.bse.raw[0] = sensor_value_to_float(&val);
    }

    // bse plaussibility check done by device driver
    ret = sensor_tol_chan_read(&bse_r, SENSOR_CHAN_PRESS, &val);
    if (ret == 0) {
      sensors->data.bse.raw[0] = sensor_value_to_float(&val);
    }

    if (!(err_get_errors() & (ERR_CODE_BSE_F | ERR_CODE_BSE_R))) {
      sensors->data.bse.travel =
          (sensors->data.bse.raw[0] + sensors->data.bse.raw[1]) / 2 /
          BSE_PRES_MAX;
    } else {
      sensors->data.bse.travel = 0;
    }

    // pedal plausibility check
    if (sensors->pedals.apps.set_zero &&
        !(err_get_errors() &
          (ERR_CODE_APPS1 | ERR_CODE_APPS2 | ERR_CODE_APPS_PLAUS))) {
      smf_run_state(SMF_CTX(&sensors->pedals));
    }

    ret = sensor_tol_chan_read(&susp_dive_r, SENSOR_CHAN_DISTANCE, &val);
    if (ret == 0) {
      sensors->data.susp.dive = sensor_value_to_float(&val);
    }

    ret = sensor_tol_chan_read(&susp_roll_r, SENSOR_CHAN_DISTANCE, &val);
    if (ret == 0) {
      sensors->data.susp.roll = sensor_value_to_float(&val);
    }

    ret = zbus_chan_pub(&sensor_data_chan, &sensors->data, K_MSEC(5));
    if (ret < 0) {
      LOG_ERR("Failed to publish sensor data: %s", strerror(-ret));
    }

    k_sleep(sys_timepoint_timeout(next));
  }
}

static int apps_val_check(float raw, float min, float max, float *_val) {
  float val = (raw - min) / (max - min);

  if (val < -APPS_VAL_TOL) {
    return -EINVAL;
  } else if (val < 0) {
    *_val = 0;
  } else if (val > 1 + APPS_VAL_TOL) {
    return -EINVAL;
  } else if (val > 1) {
    *_val = 1;
  }

  *_val = val;
  return 0;
}

static void apps_set_zero_pos(struct pedals *pedals) {
  if (err_get_errors() & (ERR_CODE_APPS1 | ERR_CODE_APPS2)) {
    return;
  }

  encoder_set_zero_pos(apps1.dev);
  encoder_set_zero_pos(apps2.dev);

  err_set_errors(ERR_CODE_APPS_PLAUS, false);

  pedals->apps.set_zero = true;
  LOG_INF("Set APPS to zero position");
}

static float encoder_val_offset(float val) {
  return val > 180 ? val - 360 : val;
}

static void encoder_set_zero_pos(const struct device *dev) {
  struct sensor_value val;
  sensor_sample_fetch_chan(dev, SENSOR_CHAN_ROTATION);
  sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val);

  val.val1 = -val.val1;
  val.val2 = -val.val2;
  sensor_attr_set(dev, SENSOR_CHAN_ROTATION, SENSOR_ATTR_OFFSET, &val);
}
