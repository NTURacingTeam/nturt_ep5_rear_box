#include "sys.h"

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

// project includes
#include "dt-bindings/rear_box.h"

LOG_MODULE_REGISTER(sys);

/* static varaible -----------------------------------------------------------*/
static const struct device* leds = DEVICE_DT_GET(DT_NODELABEL(leds));

/* function definition -------------------------------------------------------*/
void sys_reset() {
  k_sched_lock();

  LOG_INF("System reset");
  log_panic();

  led_on(leds, LED_NUM_RTD_SOUND);
  k_busy_wait(200 * 1000);
  led_off(leds, LED_NUM_RTD_SOUND);
  k_busy_wait(100 * 1000);
  led_on(leds, LED_NUM_RTD_SOUND);
  k_busy_wait(200 * 1000);
  led_off(leds, LED_NUM_RTD_SOUND);

  sys_reboot(SYS_REBOOT_COLD);
}
