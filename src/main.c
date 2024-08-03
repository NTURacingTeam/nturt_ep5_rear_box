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
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(main);

static const struct gpio_dt_spec rtd_sound =
    GPIO_DT_SPEC_GET(DT_NODELABEL(rtd_sound), gpios);

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));
static const struct device *pwmleds = DEVICE_DT_GET(DT_NODELABEL(pwmleds));

static const struct device *steer = DEVICE_DT_GET(DT_NODELABEL(steer));
static const struct device *apps1 = DEVICE_DT_GET(DT_NODELABEL(apps1));
static const struct device *apps2 = DEVICE_DT_GET(DT_NODELABEL(apps2));

static const struct device *bse_f = DEVICE_DT_GET(DT_NODELABEL(bse_f));
static const struct device *wheel_speed_fl =
    DEVICE_DT_GET(DT_NODELABEL(wheel_speed_fl));

static const struct device *susp_roll_r =
    DEVICE_DT_GET(DT_NODELABEL(susp_roll_r));

static const struct device *error_disp =
    DEVICE_DT_GET(DT_NODELABEL(error_disp));

int main() {
  struct sensor_value val;
  int ret = 0;
  int i = 0;

  while (true) {
    // char buf[4];
    // if (i < 10) {
    //   snprintf(buf, sizeof(buf), ".%d", i);
    // } else {
    //   snprintf(buf, sizeof(buf), "%2d.", i);
    // }
    // auxdisplay_write(error_disp, buf, strlen(buf));

    i = (i + 1) % 100;

    // ret = sensor_sample_fetch_chan(susp_roll_r, SENSOR_CHAN_DISTANCE);
    // if (ret < 0) {
    //   LOG_ERR("Failed to fetch susp_roll_r data: %s", strerror(-ret));
    // } else {
    //   ret = sensor_channel_get(susp_roll_r, SENSOR_CHAN_DISTANCE, &val);
    //   if (ret < 0) {
    //     LOG_ERR("Failed to get susp_roll_r data: %s", strerror(-ret));
    //   }
    //   LOG_INF("susp_roll_r val: %f1", sensor_value_to_float(&val));
    // }

    // if (sensor_sample_fetch_chan(steer, SENSOR_CHAN_ROTATION) == 0 &&
    //     sensor_channel_get(steer, SENSOR_CHAN_ROTATION, &val) == 0) {
    //   LOG_INF("steer val: %f1", sensor_value_to_float(&val));
    // }

    // k_sleep(K_MSEC(1));

    // if (sensor_sample_fetch_chan(apps1, SENSOR_CHAN_ROTATION) == 0 &&
    //     sensor_channel_get(apps1, SENSOR_CHAN_ROTATION, &val) == 0) {
    //   LOG_INF("apps1 val: %f1", sensor_value_to_float(&val));
    // }

    // k_sleep(K_MSEC(1));

    // if (sensor_sample_fetch_chan(apps2, SENSOR_CHAN_ROTATION) == 0 &&
    //     sensor_channel_get(apps2, SENSOR_CHAN_ROTATION, &val) == 0) {
    //   LOG_INF("apps2 val: %f1", sensor_value_to_float(&val));
    // }

    k_sleep(K_MSEC(100));
  }

  return 0;
}

static void buttons_cb(struct input_event *evt) {
  LOG_INF("Button event: %d, value: %d", evt->code, evt->value);
}

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);
