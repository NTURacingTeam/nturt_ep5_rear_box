#include "states/rtd_snd.h"

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// project includes
#include "define.h"
#include "states.h"

LOG_MODULE_REGISTER(states_rtd_snd);

/* static variable -----------------------------------------------------------*/
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));

/* function definition -------------------------------------------------------*/
void rtd_snd_play(struct rtd_snd *rtd_snd) {
  rtd_snd->on = false;
  rtd_snd->count = 0;

  k_work_schedule(&rtd_snd->update_work, K_NO_WAIT);
}

void rtd_snd_stop(struct rtd_snd *rtd_snd) {
  led_off(leds, LED_NUM_RTD_SOUND);
  k_work_cancel_delayable(&rtd_snd->update_work);
}

void rtd_snd_update_work(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct rtd_snd *rtd_snd = CONTAINER_OF(dwork, struct rtd_snd, update_work);

  if (!rtd_snd->on) {
    rtd_snd->on = true;
    led_on(leds, LED_NUM_RTD_SOUND);
    k_work_reschedule(&rtd_snd->update_work, K_MSEC(200));

  } else {
    if (++rtd_snd->count < 3) {
      rtd_snd->on = false;
      k_work_reschedule(&rtd_snd->update_work, K_MSEC(100));
    } else {
      states_cmd(STATES_CMD_RUN);
    }

    led_off(leds, LED_NUM_RTD_SOUND);
  }
}
