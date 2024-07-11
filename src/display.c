#include "nturt/display.h"

// glibc includes
#include <errno.h>
#include <stdint.h>
#include <string.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(display);

#define SECOND(X, Y) Y
#define NUM_MAX(BASE) (LISTIFY(DISPLAY_DIGIT_NUM, SECOND, (*), BASE) - 1)

static const struct device *disp0 = DEVICE_DT_GET(DT_NODELABEL(disp0));
static const struct device *disp1 = DEVICE_DT_GET(DT_NODELABEL(disp1));

static const uint8_t digits[16] = {
    CHAR_0, CHAR_1, CHAR_2, CHAR_3, CHAR_4, CHAR_5, CHAR_6, CHAR_7,
    CHAR_8, CHAR_9, CHAR_A, CHAR_b, CHAR_C, CHAR_d, CHAR_E, CHAR_F,
};

int display_char(const uint8_t chars[2]) {
  int ret = 0;

  ret = gpio_port_set_bits(disp0, chars[0]);
  if (ret < 0) {
    goto err;
  }

  ret = gpio_port_set_bits(disp1, chars[1]);
  if (ret < 0) {
    goto err;
  }

  return 0;

err:
  LOG_ERR("Fail to set gpio port: %s", strerror(-ret));
  return ret;
}

int display_off() { return display_char((uint8_t[2]){CHAR_OFF, CHAR_OFF}); }

int display_num(int num, int base) {
  if (!(base == DISPLAY_BASE_10 || base == DISPLAY_BASE_16) || num < 0 ||
      num > NUM_MAX(base)) {
    return -EINVAL;
  }

  uint8_t chars[2];
  for (int i = DISPLAY_DIGIT_NUM - 1; i >= 0; i--) {
    if (num > 0 || i == DISPLAY_DIGIT_NUM - 1) {
      chars[i] = digits[num % base];
      num /= base;
    } else {
      chars[i] = CHAR_OFF;
    }
  }

  return display_char(chars);
}
