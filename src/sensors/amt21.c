#include "amt21.h"

// glibc includes
#include <stddef.h>

// zephyr includes
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/amt21.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// ntu includes
#include <nturt/sensors.h>

LOG_MODULE_REGISTER(sensors_amt21);

/* macro ---------------------------------------------------------------------*/
/// @brief AMT21 start up time before ready to be read after setting zero
/// position.
#define AMT21_START_UP_TIME K_MSEC(200)

/* static function declaration -----------------------------------------------*/
static int amt21_ext_cmd_send(struct amt21* amt21, enum amt21_attribute attr);

static void reset_cb(struct k_timer* timer);

/* function definition -------------------------------------------------------*/
int amt21_init(struct amt21* amt21) {
  int ret = sensor_tol_init(&amt21->tol);
  if (ret < 0) {
    return ret;
  }

  k_timer_init(&amt21->set_zero_pos_timer, reset_cb, NULL);

  return 0;
}

bool amt21_is_ok(struct amt21* amt21) {
  return sensor_tol_is_ok(&amt21->tol) && !atomic_get(&amt21->starting);
}

int amt21_read(struct amt21* amt21, float* _val) {
  if (!sensor_tol_is_ok(&amt21->tol)) {
    return -ENODEV;
  } else if (atomic_get(&amt21->starting)) {
    return -EAGAIN;
  }

  struct sensor_value val;
  int ret = sensor_tol_chan_read(&amt21->tol, SENSOR_CHAN_ROTATION, &val);
  if (ret < 0) {
    return ret;
  }

  *_val = sensor_value_to_float(&val);
  *_val -= 360.0F * (*_val > 180.0F);

  return 0;
}

int amt21_reset(struct amt21* amt21) {
  return amt21_ext_cmd_send(amt21, SENSOR_ATTR_AMT21_RESET);
}

int amt21_set_zero_pos(struct amt21* amt21) {
  return amt21_ext_cmd_send(amt21, SENSOR_ATTR_AMT21_SET_ZERO_POS);
}

/* static function definition ------------------------------------------------*/
static int amt21_ext_cmd_send(struct amt21* amt21, enum amt21_attribute attr) {
  if (!sensor_tol_is_ok(&amt21->tol)) {
    return -ENODEV;
  } else if (!atomic_cas(&amt21->starting, false, true)) {
    return -EAGAIN;
  }

  int ret = sensor_attr_set(amt21->tol.dev, SENSOR_CHAN_ALL, attr, NULL);
  if (ret < 0) {
    goto err;
  }

  k_timer_start(&amt21->set_zero_pos_timer, AMT21_START_UP_TIME, K_NO_WAIT);

  return 0;

err:
  sensor_tol_report_error(&amt21->tol);
  atomic_set(&amt21->starting, false);

  return ret;
}

static void reset_cb(struct k_timer* timer) {
  struct amt21* amt21 = CONTAINER_OF(timer, struct amt21, set_zero_pos_timer);

  atomic_set(&amt21->starting, false);
}
