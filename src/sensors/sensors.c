#include "sensors.h"

#include "amt21.h"
#include "pedals.h"

// glibc includes
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// zephyr includes
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/err.h>
#include <nturt/sensors.h>

// project includes
#include "dt-bindings/rear_box.h"

LOG_MODULE_REGISTER(sensors);

/* macro ---------------------------------------------------------------------*/
/// @brief Sensors thread period.
#define SENSORS_THREAD_PERIOD K_MSEC(10)

/* type ----------------------------------------------------------------------*/
struct sensors {
  struct pedals pedals;
  struct amt21 steer;
  struct sensor_tol susp_dive_r, susp_roll_r;

  struct sensor_data data;
};

/* static function declaration -----------------------------------------------*/
/// @brief Initialization function for sensors module.
static int init();

static void sensors_thread(void *arg1, void *arg2, void *arg3);

/* static varaibles ----------------------------------------------------------*/
#if IS_ENABLED(CONFIG_SENSORS_POWER)
static const struct device *v_24 = DEVICE_DT_GET(DT_NODELABEL(v_24));
static const struct device *i_24 = DEVICE_DT_GET(DT_NODELABEL(i_24));
static const struct device *i_5 = DEVICE_DT_GET(DT_NODELABEL(i_5));
#endif  // CONFIG_SENSORS_POWER

static struct sensors sensors = {
    .pedals = SENSORS_PEDALS_INITIALIZER(),
    .steer = SENSORS_AMT21_INITIALIZER(DT_NODELABEL(steer), ERR_CODE_STEER),
    .susp_dive_r = SENSOR_TOL_INITIALIZER(
        DT_NODELABEL(susp_dive_r), CONFIG_SENSORS_TOL_THRES,
        CONFIG_SNESORS_TOL_WEIGHT, ERR_CODE_SUSP_DIVE),
    .susp_roll_r = SENSOR_TOL_INITIALIZER(
        DT_NODELABEL(susp_roll_r), CONFIG_SENSORS_TOL_THRES,
        CONFIG_SNESORS_TOL_WEIGHT, ERR_CODE_SUSP_ROLL),
};

ZBUS_CHAN_DEFINE(sensor_data_chan, struct sensor_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

SYS_INIT(init, APPLICATION, CONFIG_NTURT_SENSORS_INIT_PRIORITY);

K_THREAD_DEFINE(sensors_thread_tid, 2048, sensors_thread, &sensors, NULL, NULL,
                CONFIG_NTURT_SENSORS_THREAD_PRIORITY, 0, 1);

/* function definition -------------------------------------------------------*/
bool sensors_apps_engaged() { return sensors.pedals.apps.data.engaged; }

bool sensors_bse_engaged() { return sensors.pedals.bse.data.engaged; }

void steer_set_zero_pos() {
  int ret = amt21_set_zero_pos(&sensors.steer);
  if (ret == 0) {
    LOG_INF("Set steer encoder zero position");
  } else {
    LOG_ERR("Failed to set zero position for steer encoder: %s",
            strerror(-ret));
  }
}

/* static function definition ------------------------------------------------*/
static int init() {
  amt21_init(&sensors.steer);
  pedals_init(&sensors.pedals);
  sensor_tol_init(&sensors.susp_dive_r);
  sensor_tol_init(&sensors.susp_roll_r);

  return 0;
}

static void sensors_thread(void *arg1, void *arg2, void *arg3) {
  (void)arg2;
  (void)arg3;

  struct sensors *sensors = arg1;

  int ret;
  k_timepoint_t next;
  struct sensor_value val;

  while (true) {
    next = sys_timepoint_calc(SENSORS_THREAD_PERIOD);

#if IS_ENABLED(CONFIG_SENSORS_POWER)
    if ((ret = sensor_sample_fetch_chan(v_24, SENSOR_CHAN_VOLTAGE)) < 0 ||
        (ret = sensor_channel_get(v_24, SENSOR_CHAN_VOLTAGE, &val)) < 0) {
      LOG_ERR("Failed to get v_24 data: %s", strerror(-ret));
    } else {
      sensors->data.power.v_24 = sensor_value_to_float(&val);
    }

    if ((ret = sensor_sample_fetch_chan(i_24, SENSOR_CHAN_CURRENT)) < 0 ||
        (ret = sensor_channel_get(i_24, SENSOR_CHAN_CURRENT, &val)) < 0) {
      LOG_ERR("Failed to get i_24 data: %s", strerror(-ret));
    } else {
      sensors->data.power.i_24 = sensor_value_to_float(&val);
    }

    if ((ret = sensor_sample_fetch_chan(i_5, SENSOR_CHAN_CURRENT)) < 0 ||
        (ret = sensor_channel_get(i_5, SENSOR_CHAN_CURRENT, &val)) < 0) {
      LOG_ERR("Failed to get i_5 data: %s", strerror(-ret));
    } else {
      sensors->data.power.i_5 = sensor_value_to_float(&val);
    }
#endif  // CONFIG_SENSORS_POWER

    amt21_read(&sensors->steer, &sensors->data.steer);

    pedals_update(&sensors->pedals);
    memcpy(&sensors->data.apps, &sensors->pedals.apps.data,
           sizeof(sensors->data.apps));
    memcpy(&sensors->data.bse, &sensors->pedals.bse.data,
           sizeof(sensors->data.bse));

    ret =
        sensor_tol_chan_read(&sensors->susp_dive_r, SENSOR_CHAN_DISTANCE, &val);
    if (ret == 0) {
      sensors->data.susp.dive = sensor_value_to_float(&val);
    }

    ret =
        sensor_tol_chan_read(&sensors->susp_roll_r, SENSOR_CHAN_DISTANCE, &val);
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
