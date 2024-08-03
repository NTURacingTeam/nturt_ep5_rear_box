#include "sys.h"

// glibc includes
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

// project includes
#include "define.h"

LOG_MODULE_REGISTER(sys);

/* macro ---------------------------------------------------------------------*/
/// @brief Number of retry to mount file system.
#define INIT_FS_MOUNT_RETRY 3

/* static function declaration -----------------------------------------------*/
/// @brief Mount file system.
static int mount_fs();

/// @brief Initialize RTC time to dummy time if power had lost and set posix
/// realtime clock time to RTC time.
static int set_posix_clock();

/* static varaible -----------------------------------------------------------*/
static const struct device* leds = DEVICE_DT_GET(DT_NODELABEL(leds));

static const struct device* rtc = DEVICE_DT_GET(DT_NODELABEL(rtc));

SYS_INIT(mount_fs, APPLICATION, CONFIG_NTURT_SYS_INIT_PRIORITY);
SYS_INIT(set_posix_clock, APPLICATION,
         UTIL_INC(CONFIG_NTURT_SYS_INIT_PRIORITY));

/* function definition -------------------------------------------------------*/
int sys_set_time(time_t time) {
  int ret;

  struct timespec ts = {
      .tv_sec = time,
      .tv_nsec = 0,
  };
  ret = clock_settime(CLOCK_REALTIME, &ts);
  if (ret < 0) {
    LOG_ERR("Fail to set posix real-time clock time: %s", strerror(errno));
    return -errno;
  }

  ret = rtc_set_time(rtc, (struct rtc_time*)gmtime(&time));
  if (ret < 0) {
    LOG_ERR("Fail to set RTC time: %s", strerror(-ret));
    return ret;
  }

  return 0;
}

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

/* static function definition ------------------------------------------------*/
static int mount_fs(void) {
  static struct fs_littlefs lfsfs;
  static struct fs_mount_t mp = {
      .type = FS_LITTLEFS,
      .fs_data = &lfsfs,
      .flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
      .storage_dev = CONFIG_SDMMC_VOLUME_NAME,
      .mnt_point = "/" CONFIG_SDMMC_VOLUME_NAME ":",
  };

  int ret;
  for (int i = INIT_FS_MOUNT_RETRY; i >= 0; i--) {
    ret = fs_mount(&mp);
    if (ret < 0) {
      LOG_ERR("Fail to mount %s: %s", CONFIG_SDMMC_VOLUME_NAME, strerror(-ret));
      LOG_INF("Retry mounting, %d times left", i);
    } else {
      break;
    }
  }

  return 0;
}

static int set_posix_clock() {
  int ret;

  if (!device_is_ready(rtc)) {
    LOG_ERR("RTC device %s is not ready", rtc->name);
    return -ENODEV;
  }

  struct rtc_time time;
  ret = rtc_get_time(rtc, &time);
  if (ret == -ENODATA) {
    LOG_WRN("RTC time not set, initializing to dummy value");
    struct rtc_time dummy_time = {
        .tm_year = 2024 - 1900,
        .tm_mon = 0,
        .tm_mday = 1,
        .tm_wday = 0,
        .tm_hour = 0,
        .tm_min = 0,
        .tm_sec = 0,
    };

    ret = rtc_set_time(rtc, &dummy_time);
    if (ret < 0) {
      LOG_ERR("Fail to set RTC time: %s", strerror(-ret));
      return ret;
    }

    ret = rtc_get_time(rtc, &time);
  }

  if (ret < 0) {
    LOG_ERR("Fail to get RTC time: %s", strerror(-ret));
    return ret;
  }

  struct timespec ts = {
      .tv_sec = mktime(rtc_time_to_tm(&time)),
      .tv_nsec = 0,
  };

  char time_str[] = "1970-01-01T00:00:00";
  strftime(time_str, sizeof(time_str), "%FT%T", rtc_time_to_tm(&time));
  LOG_INF("Set posix real-time clock to %lld.%06ld (%s)", ts.tv_sec, ts.tv_nsec,
          time_str);

  ret = clock_settime(CLOCK_REALTIME, &ts);
  if (ret < 0) {
    LOG_ERR("Fail to set posix real-time clock time: %s", strerror(errno));
    return errno;
  }

  return 0;
}
