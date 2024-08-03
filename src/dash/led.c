#define DT_DRV_COMPAT nturt_dash_leds

#include "dash.h"

// glibc includes
#include <stddef.h>
#include <stdint.h>

// zephyr incldues
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// lib includes
#include <canopennode.h>

// nturt includes
#include <nturt/msg.h>
#include <nturt/nturt.h>

LOG_MODULE_REGISTER(led_dash, CONFIG_LED_LOG_LEVEL);

#define DASH_LED_SDO_TIMEOUT_MS 10

/* type ----------------------------------------------------------------------*/
enum dash_led_cmd {
  DASH_LED_CMD_OFF = 0x00,
  DASH_LED_CMD_ON = 0x01,
  DASH_LED_CMD_BLINK = 0x02,
};

struct dash_led_data {
  struct canopen *co;
  uint8_t leds;
};

struct dash_led_config {
  int index;

  int node_id;
};

/* static varaible -----------------------------------------------------------*/
TX_OD_DECLARE(0x2120);

/* function definition -------------------------------------------------------*/
int dash_led_init(const struct device *dev, struct canopen *co) {
  struct dash_led_data *data = dev->data;

  data->co = co;

  return 0;
}

static int dash_led_send_cmd(const struct device *dev, int led,
                             enum dash_led_cmd cmd) {
  struct dash_led_data *data = dev->data;
  const struct dash_led_config *config = dev->config;

  uint32_t mask;
  switch (led) {
    case LED_NUM_RTD:
      mask = GENMASK(1, 0);
      break;
    case LED_NUM_RUNNING:
      mask = GENMASK(3, 2);
      break;
    case LED_NUM_PEDAL_PLAUS:
      mask = GENMASK(5, 4);
      break;
    case LED_NUM_ERR:
      mask = GENMASK(7, 6);
      break;
    default:
      LOG_ERR("Invalid LED index %d", led);
      return -EINVAL;
  }

  data->leds = data->leds & ~mask;
  uint8_t field = FIELD_PREP(mask, cmd);
  data->leds = data->leds | field;
  TX_OD_SET(0x2120, 0, data->leds, u8);

  return 0;
}

int dash_led_blink(const struct device *dev, uint32_t led, uint32_t delay_on,
                   uint32_t delay_off) {
  (void)delay_on;
  (void)delay_off;

  return dash_led_send_cmd(dev, led, DASH_LED_CMD_BLINK);
}

static int dash_led_set_brightness(const struct device *dev, uint32_t led,
                                   uint8_t value) {
  return dash_led_send_cmd(dev, led,
                           value > 0 ? DASH_LED_CMD_ON : DASH_LED_CMD_OFF);
}

static int dash_led_on(const struct device *dev, uint32_t led) {
  return dash_led_send_cmd(dev, led, DASH_LED_CMD_ON);
}

static int dash_led_off(const struct device *dev, uint32_t led) {
  return dash_led_send_cmd(dev, led, DASH_LED_CMD_OFF);
}

static const struct led_driver_api dash_led_api = {
    .blink = dash_led_blink,
    .set_brightness = dash_led_set_brightness,
    .on = dash_led_on,
    .off = dash_led_off,
};

#define DASH_LED_INIT(inst)                                      \
  static struct dash_led_data dash_led_data_##inst = {};         \
                                                                 \
  static const struct dash_led_config dash_led_config_##inst = { \
      .index = DT_INST_PROP(inst, index),                        \
      .node_id = DT_INST_PROP(inst, node_id),                    \
  };                                                             \
                                                                 \
  DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &dash_led_data_##inst, \
                        &dash_led_config_##inst, POST_KERNEL,    \
                        CONFIG_LED_INIT_PRIORITY, &dash_led_api);

DT_INST_FOREACH_STATUS_OKAY(DASH_LED_INIT)
