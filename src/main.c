// glibc includes
#include <errno.h>
#include <stdint.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/auxdisplay.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// project includes
#include "dt-bindings/rear_box.h"

LOG_MODULE_REGISTER(main);

// static const struct device *v_24 = DEVICE_DT_GET(DT_NODELABEL(v_24));
// static const struct device *i_24 = DEVICE_DT_GET(DT_NODELABEL(i_24));
// static const struct device *i_5 = DEVICE_DT_GET(DT_NODELABEL(i_5));

// static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));
// static const struct device *dash_leds =
// DEVICE_DT_GET(DT_NODELABEL(dash_leds));

// static const struct device *steer = DEVICE_DT_GET(DT_NODELABEL(steer));
// static const struct device *apps1 = DEVICE_DT_GET(DT_NODELABEL(apps1));
// static const struct device *apps2 = DEVICE_DT_GET(DT_NODELABEL(apps2));

// static const struct device *bse_f = DEVICE_DT_GET(DT_NODELABEL(bse_f));

// static const struct device *susp_roll_r =
//     DEVICE_DT_GET(DT_NODELABEL(susp_roll_r));

// static const struct device *error_disp =
//     DEVICE_DT_GET(DT_NODELABEL(error_disp));

int main() {
  struct sensor_value val;
  int ret;

  while (true) {
    // if ((ret = sensor_sample_fetch_chan(v_24, SENSOR_CHAN_VOLTAGE)) < 0 ||
    //     (ret = sensor_channel_get(v_24, SENSOR_CHAN_VOLTAGE, &val)) < 0) {
    //   LOG_ERR("Failed to get v_24 data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("v_24 val: %f", (double)sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(i_24, SENSOR_CHAN_CURRENT)) < 0 ||
    //     (ret = sensor_channel_get(i_24, SENSOR_CHAN_CURRENT, &val)) < 0) {
    //   LOG_ERR("Failed to get i_24 data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("i_24 val: %f", (double)sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(i_5, SENSOR_CHAN_CURRENT)) < 0 ||
    //     (ret = sensor_channel_get(i_5, SENSOR_CHAN_CURRENT, &val)) < 0) {
    //   LOG_ERR("Failed to get i_5 data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("i_5 val: %f", (double)sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(susp_roll_r, SENSOR_CHAN_DISTANCE)) <
    //         0 ||
    //     (ret = sensor_channel_get(susp_roll_r, SENSOR_CHAN_DISTANCE, &val)) <
    //         0) {
    //   LOG_ERR("Failed to get susp_roll_r data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("susp_roll_r val: %f1", sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(steer, SENSOR_CHAN_ROTATION)) < 0 ||
    //     (ret = sensor_channel_get(steer, SENSOR_CHAN_ROTATION, &val)) < 0) {
    //   LOG_ERR("Failed to get steer data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("steer val: %f", sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(apps1, SENSOR_CHAN_ROTATION)) < 0 ||
    //     (ret = sensor_channel_get(apps1, SENSOR_CHAN_ROTATION, &val)) < 0) {
    //   LOG_ERR("Failed to get apps1 data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("apps1 val: %f", sensor_value_to_float(&val));
    // }

    // if ((ret = sensor_sample_fetch_chan(apps2, SENSOR_CHAN_ROTATION)) < 0 ||
    //     (ret = sensor_channel_get(apps2, SENSOR_CHAN_ROTATION, &val)) < 0) {
    //   LOG_ERR("Failed to get apps2 data: %s", strerror(-ret));
    // } else {
    //   LOG_INF("apps2 val: %f", sensor_value_to_float(&val));
    // }

    k_sleep(K_MSEC(500));
  }

  return 0;
}
