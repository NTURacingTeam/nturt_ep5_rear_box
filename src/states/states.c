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
#include "dt-bindings/rear_box.h"
#include "msg.h"
#include "sensors.h"
#include "states/rtd_snd.h"
#include "states/status_ctrl.h"

LOG_MODULE_REGISTER(states);

/* macro ---------------------------------------------------------------------*/
#if IS_ENABLED(CONFIG_DANGER_MODE)
#define STATE_ERROR_MASK (0)
#else
#define STATE_ERROR_MASK                                                   \
  (COND_CODE_1(IS_ENABLED(CONFIG_APPS_PLAUS), (ERR_CODE_APPS_MASK), (0)) | \
   (COND_CODE_1(IS_ENABLED(CONFIG_BSE_F), (ERR_CODE_BSE_F), (0)) |         \
    COND_CODE_1(IS_ENABLED(CONFIG_BSE_R), (ERR_CODE_BSE_R), (0))))
#endif  // CONFIG_DANGER_MODE

/* type ----------------------------------------------------------------------*/
struct states_update_args {
  enum states_update_type type;
  union {
    err_t err;
    bool button;

    enum ctrl_mode mode;
  };
};

struct states {
  struct smf_ctx smf_ctx;
  states_t states;

  struct {
    err_t err;
    bool apps;
    bool bse;
  } cond;
  enum states_update_type cmd;

  struct k_mutex mutex;

  struct status_ctrl status_ctrl;
  struct rtd_snd rtd_snd;
};

/* static function declaration -----------------------------------------------*/
static void buttons_cb(struct input_event *evt, void *user_data);

static void err_chan_cb(const struct zbus_channel *chan);

/// @brief Initialization function for states module.
static int init();

static void states_update(struct states_update_args *args);

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

INPUT_CALLBACK_DEFINE(NULL, buttons_cb, NULL);

ZBUS_LISTENER_DEFINE(states_err_chan_listener, err_chan_cb);
ZBUS_CHAN_ADD_OBS(err_chan, states_err_chan_listener, 0);

SYS_INIT(init, APPLICATION, CONFIG_STATES_INIT_PRIORITY);

/* function definition -------------------------------------------------------*/
states_t states_get_states() { return states.states; }

void states_cmd(enum states_update_type cmd) {
  __ASSERT(!k_is_in_isr(), "Cannot call states_cmd() from ISR");

  struct states_update_args args = {
      .type = cmd,
  };

  states_update(&args);
}

void states_inv_dir(bool dir) {
  __ASSERT(!(states.states & BIT(STATE_RUNNING)),
           "Cannot change inverter direction while running");

  status_inv_dir(&states.status_ctrl, dir);
}

void states_inv_reset() {
  __ASSERT(!(states.states & BIT(STATE_RUNNING)),
           "Cannot reset inverter fault while running");

  status_inv_fault_reset(&states.status_ctrl);
}

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt, void *user_data) {
  if (evt->type != INPUT_EV_KEY) {
    return;
  }

  struct states_update_args args;

  switch (evt->code) {
    case INPUT_APPS:
      args.type = STATES_COND_APPS;
      break;

    case INPUT_BSE:
      args.type = STATES_COND_BSE;
      break;

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

static int init() {
  // err module initializes later than states module, so initial errors will be
  // set by err_chan_cb().

  k_mutex_init(&states.mutex);
  states.cond.apps = sensors_apps_engaged();
  states.cond.bse = sensors_bse_engaged();

  smf_set_initial(&states.smf_ctx, &smf_states[STATE_ROOT]);
  smf_run_state(&states.smf_ctx);

  return 0;
}

static void states_update(struct states_update_args *args) {
  // wait forever since all underlying functions are non-blocking
  k_mutex_lock(&states.mutex, K_FOREVER);

  switch (args->type) {
    case STATES_COND_ERR:
      states.cond.err = args->err;
      break;

    case STATES_COND_APPS:
      states.cond.apps = args->button;
      break;

    case STATES_COND_BSE:
      states.cond.bse = args->button;
      break;

    case STATES_CMD_START ... STATES_CMD_END:
      states.cmd = args->type;
      break;

    default:
      return;
  }

  smf_run_state(&states.smf_ctx);

  k_mutex_unlock(&states.mutex);
}

static void root_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_ROOT);
}
static void root_run(void *obj) {
  struct states *states = obj;

  states->cmd = 0;
}
static void root_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_ROOT);
}

static void err_free_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_ERR_FREE);
}
static void err_free_run(void *obj) {
  struct states *states = obj;

  if (states->cond.err & STATE_ERROR_MASK) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_ERROR]);
  }
}
static void err_free_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_ERR_FREE);
}

static void ready_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_READY);

  if (states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_STEADY]);
  } else {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_BLINK]);
  }

  LOG_INF("Enter ready state");
}
static void ready_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_READY);
}

static void rtd_blink_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_RTD_BLINK);
}
static void rtd_blink_run(void *obj) {
  struct states *states = obj;

  if (!states->cond.apps && states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_STEADY]);
  }
}
static void rtd_blink_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_RTD_BLINK);
}

static void rtd_steady_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_RTD_STEADY);
}
static void rtd_steady_run(void *obj) {
  struct states *states = obj;

  if (states->cond.apps || !states->cond.bse) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_BLINK]);
  } else if (states->cmd == STATES_CMD_RTD) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_RTD_SOUND]);
  }
}
static void rtd_steady_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_RTD_STEADY);
}

static void rtd_sound_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_RTD_SOUND);

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

  states->states &= ~BIT(STATE_RTD_SOUND);

  rtd_snd_stop(&states->rtd_snd);
}

static void running_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_RUNNING);

  status_enable(&states->status_ctrl, true);
  ctrl_enable();

  LOG_INF("Enter running state");
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

  states->states &= ~BIT(STATE_RUNNING);

  status_enable(&states->status_ctrl, false);
  ctrl_disable();
}

static void error_entry(void *obj) {
  struct states *states = obj;

  states->states |= BIT(STATE_ERROR);

  LOG_ERR("Enter error state");
}
static void error_run(void *obj) {
  struct states *states = obj;

  if (!(states->cond.err & ERR_CODE_FATAL_MASK)) {
    smf_set_state(&states->smf_ctx, &smf_states[STATE_ERR_FREE]);
  }
}
static void error_exit(void *obj) {
  struct states *states = obj;

  states->states &= ~BIT(STATE_ERROR);
}
