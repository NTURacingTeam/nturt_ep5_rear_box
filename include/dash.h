#ifndef DASH_H_
#define DASH_H_

// zephyr includes
#include <zephyr/device.h>

// libs includes
#include <canopennode.h>

int dash_keys_init(const struct device* dev, struct canopen* co);

int dash_led_init(const struct device* dev, struct canopen* co);

#endif  // DASH_H_
