#include "nturt/states.h"

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>

// project includes
#include "nturt/display.h"
#include "nturt/init.h"
#include "nturt/message.h"
#include "nturt/nturt.h"

LOG_MODULE_REGISTER(states);

/* macro ---------------------------------------------------------------------*/
/// @brief States thread period.
#define STATES_THREAD_PERIOD K_MSEC(10)

/// @brief Period between error display updates.
#define ERR_DISP_PERIOD K_MSEC(500)

/* types ---------------------------------------------------------------------*/
enum states {
  STATE_ERROR = 0,
  STATE_READY,
  STATE_RUNNING,
};

struct states_ctx {
  struct smf_ctx ctx;
};

/* static function declaration -----------------------------------------------*/
static void error_chan_cb(const struct zbus_channel *chan);

/// @brief Initialization function for states module.
static int init();

static void ready_run(void *ctx);

static void running_entry(void *ctx);
static void running_run(void *ctx);
static void running_exit(void *ctx);

static void error_run(void *ctx);

static void states_thread(void *arg1, void *arg2, void *arg3);

/* static varaibles ----------------------------------------------------------*/
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));

/// @brief State machine states.
static const struct smf_state states[] = {
    [STATE_ERROR] = SMF_CREATE_STATE(NULL, error_run, NULL, NULL, NULL),
    [STATE_READY] = SMF_CREATE_STATE(NULL, ready_run, NULL, NULL, NULL),
    [STATE_RUNNING] =
        SMF_CREATE_STATE(running_entry, running_run, running_exit, NULL, NULL),
};

static bool initialized = false;

/// @brief Static atomic variable to store errors.
static atomic_t __errors = ATOMIC_INIT(0);

SYS_INIT(init, APPLICATION, INIT_STATS_INIT_PRIORITY);

K_THREAD_DEFINE(states_thread_tid, 1024, states_thread, NULL, NULL, NULL, 10, 0,
                1);

ZBUS_CHAN_DEFINE(error_chan, err_t, NULL, NULL,
                 ZBUS_OBSERVERS(states_error_chan_listener), ZBUS_MSG_INIT(0));

ZBUS_LISTENER_DEFINE(states_error_chan_listener, error_chan_cb);

/* function definition -------------------------------------------------------*/
void states_set_errors(err_t errors, bool set) {
  // does nothing if the errors are already set or cleared
  if (((states_get_errors() & errors) == errors && set) ||
      ((states_get_errors() & errors) == 0 && !set)) {
    return;
  }

  int ret;

  if (set) {
    atomic_or(&__errors, errors);
  } else {
    atomic_and(&__errors, ~errors);
  }

  if (initialized) {
    if (set) {
      LOG_ERR("Errors set: 0x%X, current errors: 0x%X", errors,
              states_get_errors());
      errors |= ERR_CODE_SET;
    } else {
      LOG_INF("Errors cleared: 0x%X, current errors: 0x%X", errors,
              states_get_errors());
      errors |= ERR_CODE_CLEAR;
    }

    ret = zbus_chan_pub(&error_chan, &errors, K_FOREVER);
    if (ret < 0) {
      LOG_ERR("Failed to publish errors: %s", strerror(-ret));
    }
  }
}

err_t states_get_errors() { return atomic_get(&__errors); }

/* static function definition ------------------------------------------------*/
static void error_chan_cb(const struct zbus_channel *chan) {
  err_t errors = *(err_t *)zbus_chan_const_msg(chan);
  bool set = FLAG_SET_AND_CLEAR(errors, ERR_CODE_SET);

  enum err_code code;
  ERR_CODE_FOR_EACH(errors, code) { ; }
}

static int init() {
  err_t errors = states_get_errors();
  if (errors != 0) {
    LOG_ERR("Set initial errors: 0x%X", errors);

    initialized = true;
    states_set_errors(errors, true);
  }

  return 0;
}

static void error_run(void *state) {
  // show error code
}

static void ready_run(void *state) {
  // blink rtd light via CAN
}

static void running_entry(void *state) {
  // play rtd sound
  for (int i = 3; i >= 0; i--) {
    led_on(leds, LED_NUM_RTD_SOUND);
    k_sleep(K_MSEC(200));
    led_off(leds, LED_NUM_RTD_SOUND);

    if (i != 0) {
      k_sleep(K_MSEC(100));
    }
  }

  // enable vechicle control
}

static void running_run(void *state) {
  // control vehicle
}

static void running_exit(void *state) {
  // shutdown vehicle control
}

static void states_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  k_timepoint_t next;
  k_timepoint_t err_disp_next = sys_timepoint_calc(ERR_DISP_PERIOD);
  int err_disp_val = 0;

  while (true) {
    next = sys_timepoint_calc(STATES_THREAD_PERIOD);

    // check rtd condition

    // update error display
    if (sys_timepoint_expired(err_disp_next)) {
      err_t errors = states_get_errors();
      if (errors == 0) {
        err_disp_val = 0;
        display_off();
      } else {
        do {
          err_disp_val = __builtin_ffs(errors & ~BIT_MASK(err_disp_val));
        } while (err_disp_val == 0);

        display_num(err_disp_val, DISPLAY_BASE_10);
      }

      err_disp_next = sys_timepoint_calc(ERR_DISP_PERIOD);
    }

    // run state machine

    k_sleep(sys_timepoint_timeout(next));
  }
}
