#include "pedals.h"

// glibc includes
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/sys/util.h>

// nturt includes
#include <nturt/err.h>
#include <nturt/sensors.h>

// project includes
#include "dt-bindings/rear_box.h"
#include "sensors.h"

LOG_MODULE_REGISTER(sensors_pedals);

/* macro ---------------------------------------------------------------------*/
/// @brief APPS1 minimum value in degrees.
#define APPS1_DEG_MIN 0.0F

/// @brief APPS1 maximum value in degrees.
#define APPS1_DEG_MAX -13.0F

/// @brief APPS1 value scale for normalization in degrees.
#define APPS1_DEG_SCALE (APPS1_DEG_MAX - APPS1_DEG_MIN)

/// @brief APPS2 minimum value in degrees.
#define APPS2_DEG_MIN 0.0F

/// @brief APPS2 maximum value in degrees.
#define APPS2_DEG_MAX 12.0F

/// @brief APPS2 value scale for normalization in degrees.
#define APPS2_DEG_SCALE (APPS2_DEG_MAX - APPS2_DEG_MIN)

/// @brief APPS value tolerance before it's considered out of range in
/// percentage of sensor value range.
#define APPS_VAL_TOL ((float)CONFIG_APPS_VAL_TOL / 100.0F)

/// @brief APPS travel threshold before reported threshold is not zero in
/// percentage of sensor value range.
#define APPS_THRES ((float)CONFIG_APPS_THRES / 100.0F)

/// @brief Tolerance of APPS travel difference to trigger APPS plausibility.
#define APPS_PLAUS_VAL_TOL 0.15F

/// @brief Time tolerance before triggering APPS plausibility.
#define APPS_PLAUS_TIME_TOL K_MSEC(100)

/// @brief Maximum brake pressure sensor value in hPa.
#define BSE_PRES_MAX 70000.0F

/// @brief BSE travel threshold before reported threshold is not zero in
/// percentage of sensor value range.
#define BSE_THRES ((float)CONFIG_BSE_THRES / 100.0F)

/// @brief Threshold of APPS travel to engage pedal plausibility.
#define PEDAL_PLAUS_ENG_THRES 0.25F

/// @brief Threshold of APPS travel to disengage pedal plausibility.
#define PEDAL_PLAUS_DISENG_THRES 0.05F

/* type ----------------------------------------------------------------------*/
enum {
  PLAUS_INACTIVE,
  PLAUS_ACTIVE,
};

/* static function declaration -----------------------------------------------*/
static void buttons_cb(struct input_event *evt, void* user_data);

static void apps_init(struct apps *apps);

static void apps_update(struct apps *apps);

static int apps_travel_cal(float raw, float min, float max, float *travel);
static void apps_set_zero_pos(struct apps *apps);

#if IS_ENABLED(CONFIG_APPS_PLAUS)
static void apps_plaus_inactive_run(void *obj);

static void apps_plaus_active_entry(void *obj);
static void apps_plaus_active_run(void *obj);
#endif  // CONFIG_APPS_PLAUS

static void bse_init(struct bse *bse);

static void bse_update(struct bse *bse);

static int bse_val_cal(float raw, float *val);

#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
static void pedal_plaus_inactive_run(void *obj);

static void pedal_plaus_active_entry(void *obj);
static void pedal_plaus_active_run(void *obj);
static void pedal_plaus_active_exit(void *obj);
#endif  // CONFIG_PEDAL_PLAUS

/* static variable -----------------------------------------------------------*/
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));

#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
static const struct device *dash_leds = DEVICE_DT_GET(DT_NODELABEL(dash_leds));
#endif  // CONFIG_PEDAL_PLAUS

#if IS_ENABLED(CONFIG_APPS_MICRO)
static const struct gpio_dt_spec apps_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(apps_micro), gpios);
#endif  // CONFIG_APPS_MICRO

#if IS_ENABLED(CONFIG_BSE_MICRO)
static const struct gpio_dt_spec bse_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bse_micro), gpios);
#endif  // CONFIG_BSE_MICRO

static struct pedals *pedals;

SENSOR_TOL_DEFINE(bse_f, DT_NODELABEL(bse_f), CONFIG_SENSORS_TOL_THRES,
                  CONFIG_SNESORS_TOL_WEIGHT, ERR_CODE_BSE_F);
SENSOR_TOL_DEFINE(bse_r, DT_NODELABEL(bse_r), CONFIG_SENSORS_TOL_THRES,
                  CONFIG_SNESORS_TOL_WEIGHT, ERR_CODE_BSE_R);

INPUT_CALLBACK_DEFINE(NULL, buttons_cb, NULL);

#if IS_ENABLED(CONFIG_APPS_PLAUS)
static const struct smf_state apps_plaus_states[] = {
    [PLAUS_INACTIVE] =
        SMF_CREATE_STATE(NULL, apps_plaus_inactive_run, NULL, NULL, NULL),
    [PLAUS_ACTIVE] = SMF_CREATE_STATE(apps_plaus_active_entry,
                                      apps_plaus_active_run, NULL, NULL, NULL),
};
#endif  // CONFIG_APPS_PLAUS

#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
static const struct smf_state pedal_plaus_states[] = {
    [PLAUS_INACTIVE] =
        SMF_CREATE_STATE(NULL, pedal_plaus_inactive_run, NULL, NULL, NULL),
    [PLAUS_ACTIVE] =
        SMF_CREATE_STATE(pedal_plaus_active_entry, pedal_plaus_active_run,
                         pedal_plaus_active_exit, NULL, NULL),
};
#endif  // CONFIG_PEDAL_PLAUS

/* function definition -------------------------------------------------------*/
void pedals_init(struct pedals *_pedals) {
  pedals = _pedals;

  apps_init(&pedals->apps);
  bse_init(&pedals->bse);

#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
  smf_set_initial(&pedals->plaus_smf, &pedal_plaus_states[PLAUS_INACTIVE]);
#endif  // CONFIG_PEDAL_PLAUS
}

void pedals_update(struct pedals *pedals) {
  apps_update(&pedals->apps);
  bse_update(&pedals->bse);

  // pedal plausibility check
#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
  if (pedals->apps.set_zero &&
      (amt21_is_ok(&pedals->apps.apps1) COND_CODE_1(
          CONFIG_APPS_PLAUS, (&&), (||)) amt21_is_ok(&pedals->apps.apps2)) &&
      (!(err_get_errors() & ERR_CODE_BSE_F) COND_CODE_1(
          CONFIG_BSE_PLAUS, (&&), (||)) !(err_get_errors() & ERR_CODE_BSE_R))) {
    smf_run_state(&pedals->plaus_smf);
  }
#endif  // CONFIG_PEDAL_PLAUS
}

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt, void* user_data) {
  if (evt->type != INPUT_EV_KEY) {
    return;
  }

  switch (evt->code) {
    case INPUT_APPS:
      pedals->apps.data.engaged = evt->value;
      if (!evt->value && !pedals->apps.set_zero) {
        apps_set_zero_pos(&pedals->apps);
      }
      break;

    case INPUT_BSE:
      pedals->bse.data.engaged = evt->value;
      led_set_brightness(leds, LED_NUM_BRAKE_LIGHT, evt->value ? 100 : 0);
      break;

#if IS_ENABLED(CONFIG_APPS_MICRO)
    case INPUT_APPS_MICRO:
      input_report_key(NULL, INPUT_APPS, evt->value, true, K_FOREVER);
      break;
#endif  // CONFIG_APPS_MICRO

#if IS_ENABLED(CONFIG_BSE_MICRO)
    case INPUT_BSE_MICRO:
      input_report_key(NULL, INPUT_BSE, evt->value, true, K_FOREVER);
      break;
#endif  // CONFIG_BSE_MICRO

    default:
      break;
  }
}

static void apps_init(struct apps *apps) {
  amt21_init(&apps->apps1);
  amt21_init(&apps->apps2);

#if IS_ENABLED(CONFIG_APPS_MICRO)
  input_report_key(NULL, INPUT_APPS, gpio_pin_get_dt(&apps_micro), true,
                   K_FOREVER);
#else
  // assuming APPS is not engaged during startup if no APPS micro switch
  input_report_key(NULL, INPUT_APPS, false, true, K_FOREVER);
#endif  // CONFIG_APPS_MICRO

  if (apps->data.engaged) {
    LOG_WRN("Accelerator engaged during startup");
    err_set_errors(ERR_CODE_APPS_PLAUS, true);
  }

#if IS_ENABLED(CONFIG_APPS_PLAUS)
  smf_set_initial(&apps->plaus_smf, &apps_plaus_states[PLAUS_INACTIVE]);
#endif  // CONFIG_APPS_PLAUS
}

static void apps_update(struct apps *apps) {
  int ret;

  ret = amt21_read(&apps->apps1, &apps->data.raw[0]);
  if (ret == 0 && apps->set_zero) {
    ret = apps_travel_cal(apps->data.raw[0], APPS1_DEG_MIN, APPS1_DEG_MAX,
                          &apps->travel1);
    if (ret < 0) {
      LOG_WRN("APPS1 value out of range: %f", (double)apps->data.raw[0]);
      // sensor_tol_report_fail(&apps->apps1.tol);
    }
  }

  int apps1_ret = ret;

  ret = amt21_read(&apps->apps2, &apps->data.raw[1]);
  if (ret == 0 && apps->set_zero) {
    ret = apps_travel_cal(apps->data.raw[1], APPS2_DEG_MIN, APPS2_DEG_MAX,
                          &apps->travel2);
    if (ret < 0) {
      LOG_WRN("APPS2 value out of range: %f", (double)apps->data.raw[1]);
      sensor_tol_report_fail(&apps->apps2.tol);
    }
  }

  int apps2_ret = ret;

  if (apps1_ret < 0) {
    apps->travel1 = apps->travel2;
  }

  if (apps2_ret < 0) {
    apps->travel2 = apps->travel1;
  }

  /// @todo use values of successful read
  // apps plausibility check
  if (apps->set_zero && amt21_is_ok(&apps->apps1) && amt21_is_ok(&apps->apps2)
#if IS_ENABLED(CONFIG_APPS_PLAUS)
      // && smf_run_state(&apps->plaus_smf) == 0
#endif  // CONFIG_APPS_PLAUS
  ) {
    apps->data.travel = (apps->travel1 + apps->travel2) / 2;
  }
#if !IS_ENABLED(CONFIG_APPS_PLAUS)
  else if (apps->set_zero && amt21_is_ok(&apps->apps1)) {
    apps->data.travel = apps->travel1;
  } else if (apps->set_zero && amt21_is_ok(&apps->apps2)) {
    apps->data.travel = apps->travel2;
  }
#endif  // CONFIG_APPS_PLAUS
  else {
    apps->data.travel = 0.0F;
  }

#if IS_ENABLED(CONFIG_APPS_MICRO)
  if (!apps->data.engaged) {
    apps->data.travel = 0.0F;
  }
#else
  if (apps->data.travel > 0.0F && !apps->data.engaged) {
    input_report_key(NULL, INPUT_APPS, true, true, K_FOREVER);

  } else if (apps->data.travel == 0.0F && apps->data.engaged) {
    input_report_key(NULL, INPUT_APPS, false, true, K_FOREVER);
  }
#endif  // CONFIG_APPS_MICRO
}

static int apps_travel_cal(float raw, float min, float max, float *travel) {
  float val = (raw - min) / (max - min);

  if (val < -APPS_VAL_TOL) {
    return -EINVAL;
  } else if (val < APPS_THRES) {
    *travel = 0.0F;
  } else if (val > 1 + APPS_VAL_TOL) {
    return -EINVAL;
  } else if (val > 1) {
    *travel = 1.0F;
  }

  *travel = val;
  return 0;
}

static void apps_set_zero_pos(struct apps *apps) {
  int ret;

  ret = amt21_set_zero_pos(&apps->apps1);
  if (ret == 0) {
    LOG_INF("Set apps1 encoder zero position");
  } else {
    LOG_ERR("Failed to set zero position for apps1 encoder: %s",
            strerror(-ret));
  }

  ret = amt21_set_zero_pos(&apps->apps2);
  if (ret == 0) {
    LOG_INF("Set apps2 encoder zero position");
  } else {
    LOG_ERR("Failed to set zero position for apps2 encoder: %s",
            strerror(-ret));
  }

  apps->set_zero = true;
}

#if IS_ENABLED(CONFIG_APPS_PLAUS)

static void apps_plaus_inactive_run(void *obj) {
  struct apps *apps = obj;

  if (fabsf(apps->travel1 - apps->travel2) > APPS_PLAUS_VAL_TOL) {
    smf_set_state(&apps->plaus_smf, &apps_plaus_states[PLAUS_ACTIVE]);
  }
}

static void apps_plaus_active_entry(void *obj) {
  struct apps *apps = obj;

  apps->plaus_time = sys_timepoint_calc(APPS_PLAUS_TIME_TOL);
}
static void apps_plaus_active_run(void *obj) {
  struct apps *apps = obj;

  if (fabsf(apps->travel1 - apps->travel2) < APPS_PLAUS_VAL_TOL) {
    smf_set_state(&apps->plaus_smf, &apps_plaus_states[PLAUS_INACTIVE]);
    return;
  }

  LOG_WRN("APPS plausibility check failed: %f(%f), %f(%f)",
          (double)apps->travel1, (double)apps->data.raw[0],
          (double)apps->travel2, (double)apps->data.raw[1]);

  if (sys_timepoint_expired(apps->plaus_time)) {
    err_set_errors(ERR_CODE_APPS_PLAUS, true);
    LOG_ERR("APPS plausibility check failed lasted over 100 ms");

    smf_set_terminate(&apps->plaus_smf, -EINVAL);
  }
}

#endif  // CONFIG_APPS_PLAUS

static void bse_init(struct bse *bse) {
  if (!device_is_ready(bse_f.dev)) {
    err_set_errors(ERR_CODE_BSE_F, true);
    LOG_ERR("BSE front not ready");
  }

  if (!device_is_ready(bse_r.dev)) {
    err_set_errors(ERR_CODE_BSE_R, true);
    LOG_ERR("BSE rear not ready");
  }

#if IS_ENABLED(CONFIG_BSE_MICRO)
  input_report_key(NULL, INPUT_BSE, gpio_pin_get_dt(&bse_micro), true,
                   K_FOREVER);
#else
  bse_update(bse);

  // bse_update() will report BSE key if BSE micro is engaged, so only report
  // if not engaged
  if (!bse->data.engaged) {
    input_report_key(NULL, INPUT_BSE, false, true, K_FOREVER);
  }
#endif  // CONFIG_BSE_MICRO
}

static void bse_update(struct bse *bse) {
  int ret;
  struct sensor_value val;

  // bse error checking done by device driver
  ret = sensor_tol_chan_read(&bse_f, SENSOR_CHAN_PRESS, &val);
  if (ret == 0) {
    bse->data.raw[0] = sensor_value_to_float(&val);
    bse_val_cal(bse->data.raw[0], &bse->val1);
  }

  ret = sensor_tol_chan_read(&bse_r, SENSOR_CHAN_PRESS, &val);
  if (ret == 0) {
    bse->data.raw[1] = sensor_value_to_float(&val);
    bse_val_cal(bse->data.raw[1], &bse->val2);
  }

  // bse plausibility check
  if (IS_ENABLED(CONFIG_BSE_F) && IS_ENABLED(CONFIG_BSE_R) &&
      !(err_get_errors() & (ERR_CODE_BSE_F | ERR_CODE_BSE_R))) {
    bse->data.travel = (bse->val1 + bse->val2) / 2;
  }
#if !IS_ENABLED(CONFIG_BSE_PLAUS)
  else if (IS_ENABLED(CONFIG_BSE_F) && !(err_get_errors() & ERR_CODE_BSE_F)) {
    bse->data.travel = bse->val1;
  } else if (IS_ENABLED(CONFIG_BSE_R) && !(err_get_errors() & ERR_CODE_BSE_R)) {
    bse->data.travel = bse->val2;
  }
#endif  // CONFIG_BSE_PLAUS
  else {
    bse->data.travel = 0;
  }

#if !IS_ENABLED(CONFIG_BSE_MICRO)
  if (bse->data.travel > 0 && !bse->data.engaged) {
    input_report_key(NULL, INPUT_BSE, true, true, K_FOREVER);

  } else if (bse->data.travel == 0 && bse->data.engaged) {
    input_report_key(NULL, INPUT_BSE, false, true, K_FOREVER);
  }
#endif  // CONFIG_BSE_MICRO
}

static int bse_val_cal(float raw, float *val) {
  *val = raw / BSE_PRES_MAX;

  if (*val < BSE_THRES) {
    *val = 0.0F;
  } else if (*val > 1) {
    *val = 1.0F;
  }

  return 0;
}

#if IS_ENABLED(CONFIG_PEDAL_PLAUS)

static void pedal_plaus_inactive_run(void *obj) {
  struct pedals *pedals = obj;

  if (pedals->bse.data.engaged &&
      pedals->apps.data.travel > PEDAL_PLAUS_ENG_THRES) {
    smf_set_state(SMF_CTX(pedals), &pedal_plaus_states[PLAUS_ACTIVE]);
  }
}

static void pedal_plaus_active_entry(void *obj) {
  (void)obj;

  err_set_errors(ERR_CODE_PEDAL_PLAUS, true);
  led_on(dash_leds, LED_NUM_PEDAL_PLAUS);

  LOG_WRN("Pedal plausibility check triggered");
}
static void pedal_plaus_active_run(void *obj) {
  struct pedals *pedals = obj;

  if (pedals->apps.data.travel < PEDAL_PLAUS_DISENG_THRES) {
    smf_set_state(SMF_CTX(pedals), &pedal_plaus_states[PLAUS_INACTIVE]);
  }
}
static void pedal_plaus_active_exit(void *obj) {
  (void)obj;

  err_set_errors(ERR_CODE_PEDAL_PLAUS, false);
  led_off(dash_leds, LED_NUM_PEDAL_PLAUS);
}

#endif  // CONFIG_PEDAL_PLAUS
