// glibc includes
#include <errno.h>
#include <stdint.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// project includes
#include "nturt/display.h"
#include "nturt/nturt.h"

LOG_MODULE_REGISTER(main);

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));
static const struct device *pwmleds = DEVICE_DT_GET(DT_NODELABEL(pwmleds));

static const struct device *steer = DEVICE_DT_GET(DT_NODELABEL(steer));
static const struct device *apps1 = DEVICE_DT_GET(DT_NODELABEL(apps1));

static const struct device *break_press_r =
    DEVICE_DT_GET(DT_NODELABEL(break_press_r));
static const struct device *wheel_speed_fl =
    DEVICE_DT_GET(DT_NODELABEL(wheel_speed_fl));

int main() {
  struct sensor_value val;
  int ret = 0;
  int i = 0;

  while (true) {
    // display_num(i, DISPLAY_BASE_10);
    // led_set_brightness(pwmleds, 1, i);
    i = (i + 1) % 100;

    ret = sensor_sample_fetch_chan(steer, SENSOR_CHAN_ROTATION);
    if (ret < 0) {
      LOG_ERR("Failed to fetch steer data: %s", strerror(-ret));
    } else {
      ret = sensor_channel_get(steer, SENSOR_CHAN_ROTATION, &val);
      if (ret < 0) {
        LOG_ERR("Failed to get steer data: %s", strerror(-ret));
      }
      LOG_INF("steer val: %f1", sensor_value_to_float(&val));
    }

    k_sleep(K_MSEC(1));

    ret = sensor_sample_fetch_chan(apps1, SENSOR_CHAN_ROTATION);
    if (ret < 0) {
      LOG_ERR("Failed to fetch apps1 data: %s", strerror(-ret));
    } else {
      ret = sensor_channel_get(apps1, SENSOR_CHAN_ROTATION, &val);
      if (ret < 0) {
        LOG_ERR("Failed to get apps1 data: %s", strerror(-ret));
      }
      LOG_INF("apps1 val: %f1", sensor_value_to_float(&val));
    }

    k_sleep(K_MSEC(100));
  }

  return 0;
}

static void buttons_cb(struct input_event *evt) {
  if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_BUILTIN) {
    led_set_brightness(leds, LED_NUM_RTD_SOUND, evt->value ? 100 : 0);
  }
}

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);
