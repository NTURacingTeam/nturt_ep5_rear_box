#include "states.h"

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/err.h>

// project includes
#include "ctrl.h"
#include "define.h"
#include "msg.h"
#include "states/rtd_snd.h"
#include "states/status_ctrl.h"

LOG_MODULE_REGISTER(states);

/* macro ---------------------------------------------------------------------*/
/// @brief Size of work buffer for states update callback.
#define STATES_WORK_BUF_SIZE 10

/* type ----------------------------------------------------------------------*/
struct states_update_args {
  enum states_update_type type;
  union {
    err_t err;
    bool button;
  };
};

struct states {
  struct smf_ctx smf_ctx;
  state_t state;

  struct {
    err_t err;
    bool apps;
    bool bse;
    bool rtd_button;
  } cond;

  enum states_update_type cmd;

  struct status_ctrl status_ctrl;
  struct rtd_snd rtd_snd;
};

/* static function declaration -----------------------------------------------*/
static void buttons_cb(struct input_event *evt);

static void err_chan_cb(const struct zbus_channel *chan);

static int states_update(struct states_update_args *args);
static void states_update_work(struct k_work *work);

/// @brief Initialization function for states module.
static int init();

static void root_entry(void *obj);
static void root_run(void *obj);
static void root_exit(void *obj);

static void err_free_entry(void *obj);
static void err_free_run(void *obj);
static void err_free_exit(void *obj);

static void ready_entry(void *obj);
static void ready_exit(void *obj);

static void rtd_blink_entry(void *obj);
static void rtd_blink_run(void *obj);
static void rtd_blink_exit(void *obj);

static void rtd_steady_entry(void *obj);
static void rtd_steady_run(void *obj);
static void rtd_steady_exit(void *obj);

static void rtd_sound_entry(void *obj);
static void rtd_sound_run(void *obj);
static void rtd_sound_exit(void *obj);

static void running_entry(void *obj);
static void running_run(void *obj);
static void running_exit(void *obj);

static void error_entry(void *obj);
static void error_run(void *obj);
static void error_exit(void *obj);

/* static varaible -----------------------------------------------------------*/
static const struct device *dash_leds = DEVICE_DT_GET(DT_NODELABEL(dash_leds));

static const struct gpio_dt_spec apps_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(apps_micro), gpios);
static const struct gpio_dt_spec bse_micro =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bse_micro), gpios);

/// @brief States state machine.
static const struct smf_state smf_states[] = {
    [STATE_ROOT] = SMF_CREATE_STATE(root_entry, root_run, root_exit, NULL,
                                    &smf_states[STATE_ERR_FREE]),
    [STATE_ERR_FREE] =
        SMF_CREATE_STATE(err_free_entry, err_free_run, err_free_exit,
                         &smf_states[STATE_ROOT], &smf_states[STATE_READY]),
    [STATE_READY] = SMF_CREATE_STATE(ready_entry, NULL, ready_exit,
                                     &smf_states[STATE_ERR_FREE], NULL),
    [STATE_RTD_BLINK] =
        SMF_CREATE_STATE(rtd_blink_entry, rtd_blink_run, rtd_blink_exit,
                         &smf_states[STATE_READY], NULL),
    [STATE_RTD_STEADY] =
        SMF_CREATE_STATE(rtd_steady_entry, rtd_steady_run, rtd_steady_exit,
                         &smf_states[STATE_READY], NULL),
    [STATE_RTD_SOUND] =
        SMF_CREATE_STATE(rtd_sound_entry, rtd_sound_run, rtd_sound_exit,
                         &smf_states[STATE_READY], NULL),
    [STATE_RUNNING] = SMF_CREATE_STATE(running_entry, running_run, running_exit,
                                       &smf_states[STATE_ERR_FREE], NULL),
    [STATE_ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit,
                                     &smf_states[STATE_ROOT], NULL),
};

static struct states states = {
    .status_ctrl = STATUS_CTRL_INITIALIZER(),
    .rtd_snd = RTD_SND_INITIALIZER(),
};

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);

ZBUS_CHAN_DEFINE(state_chan, state_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(0));

ZBUS_LISTENER_DEFINE(states_err_chan_listener, err_chan_cb);
ZBUS_CHAN_ADD_OBS(err_chan, states_err_chan_listener, 0);

WORK_CTX_BUF_DEFINE(states_update_work_ctx, STATES_WORK_BUF_SIZE,
                    states_update_work, &states, struct states_update_args);

SYS_INIT(init, APPLICATION, CONFIG_NTURT_STATES_INIT_PRIORITY);

/* function definition -------------------------------------------------------*/
int states_cmd(enum states_update_type type) {
  struct states_update_args args = {
      .type = type,
  };

  return states_update(&args);
}

int states_inv_dir(bool dir) {
  if (states.state & BIT(STATE_RUNNING)) {
    LOG_ERR("Cannot change inverter direction while running");

    return -EINVAL;
  }

  status_inv_dir(&states.status_ctrl, dir);
  return 0;
}

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt) {
  if (evt->type != INPUT_EV_KEY) {
    return;
  }

  struct states_update_args args;

  switch (evt->code) {
    case INPUT_KEY_APPS:
      args.type = STATES_COND_APPS;
      break;

    case INPUT_KEY_BSE:
      args.type = STATES_COND_BSE;
      break;

    case INPUT_KEY_RTD:
      args.type = STATES_COND_RTD_BUTTON;
      break;

    case INPUT_KEY_CTRL_MODE:
      if (evt->value) {
        states_cmd(STATES_CMD_DISABLE);
        states_cmd(STATES_CMD_MODE_CHANGE);
      }

      return;

    default:
      return;
  }
  args.button = evt->value;

  states_update(&args);
}

static void err_chan_cb(const struct zbus_channel *chan) {
  (void)chan;

  struct states_update_args args = {
      .type = STATES_COND_ERR,
      .err = err_get_errors(),
  };

  states_update(&args);
}

static int states_update(struct states_update_args *args) {
  struct work_ctx *ctx =
      work_ctx_alloc(states_update_work_ctx, STATES_WORK_BUF_SIZE);
  if (ctx != NULL) {
    memcpy(ctx->args, args, sizeof(*args));
    k_work_submit(&ctx->work);

    return 0;
  } else {
    LOG_ERR("States update queue full, dropping states update request: %d",
            args->type);

    return -ENOMEM;
  }
}

static void states_update_work(struct k_work *work) {
  struct states *states = WORK_CTX(work);
  struct states_update_args *args = WORK_CTX_ARGS(work);

  switch (args->type) {
    case STATES_COND_ERR:
      states->cond.err = args->err;
      break;

    case STATES_COND_APPS:
      states->cond.apps = args->button;
      break;

    case STATES_COND_BSE:
      states->cond.bse = args->button;
      break;

    case STATES_COND_RTD_BUTTON:
      states->cond.rtd_button = args->button;
      break;

    case STATES_CMD_START ... STATES_CMD_END:
      states->cmd = args->type;
      break;

    default:
      return;
  }

  smf_run_state(&states->smf_ctx);
  zbus_chan_pub(&state_chan, &states->state, K_MSEC(5));
}

static int init() {
  // err module initializes later than states module, so initial errors will be
  // set by err_chan_cb().

  states.cond.apps = gpio_pin_get_dt(&apps_micro);
  states.cond.bse = gpio_pin_get_dt(&bse_micro);

  smf_set_initial(&states.smf_ctx, &smf_states[STATE_ROOT]);
  smf_run_state(&states.smf_ctx);
  zbus_chan_pub(&state_chan, &states.state, K_MSEC(5));

  return 0;
}

static void root_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_ROOT);
}
static void root_run(void *obj) {
  struct states *states = obj;

  switch (states->cmd) {
    case STATES_CMD_FAULT_RESET:
      status_inv_fault_reset(&states->status_ctrl);
      break;

    case STATES_CMD_MODE_CHANGE:
      ctrl_mode_next();
      break;

    default:
      break;
  }

  states->cmd = 0;
}
static void root_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_ROOT);
}

static void err_free_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_ERR_FREE);
}
static void err_free_run(void *obj) {
  struct states *states = obj;

  if (states->cond.err & ERR_CODE_FATAL_MASK) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_ERROR]);
  }
}
static void err_free_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_ERR_FREE);
}

static void ready_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_READY);

  if (states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_STEADY]);
  } else {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_BLINK]);
  }
}
static void ready_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_READY);
}

static void rtd_blink_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_RTD_BLINK);

  led_blink(dash_leds, LED_NUM_RTD, 0, 0);
}
static void rtd_blink_run(void *obj) {
  struct states *states = obj;

  if (!states->cond.apps && states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_STEADY]);
  }
}
static void rtd_blink_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_RTD_BLINK);

  led_off(dash_leds, LED_NUM_RTD);
}

static void rtd_steady_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_RTD_STEADY);

  // reset rtd button state to avoid immediate transition even if button is
  // pressed
  states->cond.rtd_button = false;
  led_on(dash_leds, LED_NUM_RTD);
}
static void rtd_steady_run(void *obj) {
  struct states *states = obj;

  if (states->cond.apps || !states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_BLINK]);
  } else if (states->cond.rtd_button) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_SOUND]);
  }
}
static void rtd_steady_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_RTD_STEADY);

  led_off(dash_leds, LED_NUM_RTD);
}

static void rtd_sound_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_RTD_SOUND);

  rtd_snd_play(&states->rtd_snd);
}
static void rtd_sound_run(void *obj) {
  struct states *states = obj;

  if (states->cond.apps || !states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_BLINK]);
  } else if (states->cmd == STATES_CMD_RUN) {
    states->cmd = 0;

    smf_set_state(&states->smf_ctx, &smf_states[STATE_RUNNING]);
  }
}
static void rtd_sound_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_RTD_SOUND);

  rtd_snd_stop(&states->rtd_snd);
}

static void running_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_RUNNING);

  led_on(dash_leds, LED_NUM_RUNNING);
  status_enable(&states->status_ctrl, true);
  ctrl_enable();
}
static void running_run(void *obj) {
  struct states *states = obj;

  if (states->cmd == STATES_CMD_DISABLE) {
    states->cmd = 0;

    smf_set_state(&states->smf_ctx, &smf_states[STATE_READY]);
  }
}
static void running_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_RUNNING);

  led_off(dash_leds, LED_NUM_RUNNING);
  status_enable(&states->status_ctrl, false);
  ctrl_disable();
}

static void error_entry(void *obj) {
  struct states *states = obj;

  states->state |= BIT(STATE_ERROR);

  led_on(dash_leds, LED_NUM_ERR);
}
static void error_run(void *obj) {
  struct states *states = obj;

  if (!(states->cond.err & ERR_CODE_FATAL_MASK)) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_ERR_FREE]);
  }
}
static void error_exit(void *obj) {
  struct states *states = obj;

  states->state &= ~BIT(STATE_ERROR);

  led_off(dash_leds, LED_NUM_ERR);
}
