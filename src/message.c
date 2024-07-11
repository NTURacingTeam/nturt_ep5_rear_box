#include "nturt/message.h"

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

// lib includes
#include <canopennode.h>

// project includes
#include "nturt/init.h"
#include "nturt/nturt.h"
#include "nturt/states.h"
#include "nturt/sys.h"
#include "nturt/util.h"

LOG_MODULE_REGISTER(message);

/* macro ---------------------------------------------------------------------*/
/// @brief Size of work buffer for EMCY callback.
#define EMCY_WORK_BUF_SIZE 5

#define CO_EMCY_COB_ID_BASE 0x80

/// @brief CANopen time reference in POSIX epoch time in seconds.
#define CO_SEC_REF 441763200UL

/// @brief Number of seconds in a day.
#define SEC_OF_DAY 86400

#define NODE_ID CO_NODE_ID_FB

#define BAUDRATE 500

/// @brief Define the monitored node list.
#define NODE_MON_LIST                                                \
  NODE_MON_LISTIFY(CO_NODE_ID_RB, CO_NODE_ID_ACC, CO_NODE_ID_INV_RL, \
                   CO_NODE_ID_INV_RR)

/* types ---------------------------------------------------------------------*/
/// @brief Arguments for bottom half of @ref emcy_cb.
struct emcy_cb_args {
  uint16_t ident;
  uint16_t errorCode;
  uint8_t errorRegister;
  uint8_t errorBit;
  uint32_t infoCode;
};

/* static function declaration -----------------------------------------------*/
/// @brief Callback function when receiving CANopen EMCY object.
static void emcy_cb(uint16_t ident, uint16_t errorCode, uint8_t errorRegister,
                    uint8_t errorBit, uint32_t infoCode);

/// @brief Bottom half of @ref emcy_cb.
static void emcy_work(struct k_work *work);

/// @brief Callback function when receiving CANopen TIME object.
static void time_cb(void *arg);

/// @brief Bottom half of @ref time_cb.
static void time_work(struct k_work *work);

/// @brief Callback function when monitored node NMT state changed.
static void hb_mnt_changed_cb(uint8_t id, uint8_t idx,
                              CO_NMT_internalState_t state, void *arg);

/// @brief Callback function when monitored node heartbeat started for the first
/// time after timeout.
static void hb_started_cb(uint8_t id, uint8_t idx, void *arg);

/// @brief Callback function when monitored node heartbeat timed out for the
/// first time.
static void hb_timeout_cb(uint8_t id, uint8_t idx, void *arg);

/// @brief Callback function when monitored node reset.
static void hb_reset_cb(uint8_t id, uint8_t idx, void *arg);

/// @brief Callback function when receiving from error channel.
static void error_chan_cb(const struct zbus_channel *chan);

/// @brief Initialization function for message module.
static int init();

static enum err_code node_mon_id_to_err_code(enum co_node_id id);

static void node_mon_update_err_code(enum co_node_id id, bool err);

/* static varaibles ----------------------------------------------------------*/
CANOPEN_STORAGE_DEFINE(
    storage_entries,
    CANOPEN_STORAGE_ENTRY(OD_PERSIST_COMM, 0x02,
                          CO_storage_cmd | CO_storage_restore),
    CANOPEN_STORAGE_ENTRY(OD_EEPROM, 0x03,
                          CO_storage_cmd | CO_storage_restore), );

static struct canopen co = {
    .can = DEVICE_DT_GET(DT_ALIAS(canopen_can)),
    .green_led = GPIO_DT_SPEC_GET(DT_ALIAS(canopen_green_led), gpios),
    .red_led = GPIO_DT_SPEC_GET(DT_ALIAS(canopen_red_led), gpios),
    .node_id = NODE_ID,
    .bitrate = BAUDRATE,
    .nmt_control = CO_NMT_ERR_REG_MASK | CO_NMT_STARTUP_TO_OPERATIONAL |
                   CO_NMT_ERR_ON_BUSOFF_HB | CO_NMT_ERR_ON_ERR_REG |
                   CO_NMT_ERR_FREE_TO_OPERATIONAL,
    .storage_entries = storage_entries,
    .storage_entries_count = ARRAY_SIZE(storage_entries),
};

static bool time_set = false;

WORK_CTX_BUF_DEFINE(emcy_ctx, EMCY_WORK_BUF_SIZE, emcy_work,
                    struct emcy_cb_args);

static K_WORK_DEFINE(time_work_data, time_work);

SYS_INIT(init, APPLICATION, INIT_MSG_INIT_PRIORITY);

ZBUS_LISTENER_DEFINE(msg_error_chan_listener, error_chan_cb);
ZBUS_CHAN_ADD_OBS(error_chan, msg_error_chan_listener, 0);

NODE_MON_DEFINE(node_mon_states, NODE_MON_LIST);

ZBUS_CHAN_DEFINE(imu_data_chan, struct msg_imu, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
REC_GROUP_DEFINE(imu_data, msg_imu, imu_data_chan);
REC_OD_EXT_DEFINE(od_imu_accel_ext, msg_imu, &imu_data, REC_OD_VAR(accel_x, 1),
                  REC_OD_VAR(accel_y, 2), REC_OD_VAR(accel_z, 3));

/* static function definition ------------------------------------------------*/
static void emcy_cb(uint16_t ident, uint16_t errorCode, uint8_t errorRegister,
                    uint8_t errorBit, uint32_t infoCode) {
  struct work_ctx *ctx = work_ctx_alloc(emcy_ctx, EMCY_WORK_BUF_SIZE);
  if (ctx != NULL) {
    struct emcy_cb_args *args = ctx->args;
    args->ident = ident;
    args->errorCode = errorCode;
    args->errorRegister = errorRegister;
    args->errorBit = errorBit;
    args->infoCode = infoCode;

    k_work_submit(&ctx->work);
  } else {
    LOG_ERR(
        "EMCY process queue full, dropping EMCY object from id: %d, err: %d.",
        ident, errorCode);
  }
}

static void emcy_work(struct k_work *work) {
  struct emcy_cb_args *args = WORK_CTX_ARGS(work);
  bool set = args->errorCode != CO_EMC_NO_ERROR;

  // frame id == 0 when EMCY from this node
  if (args->ident == 0) {
    // filter out manufacturer and heartbeat consumer errors since they are
    // handled by states module and heartbeat consumer callbacks, respectively
    if (args->errorBit >= CO_EM_MANUFACTURER_START ||
        args->errorBit == CO_EM_HEARTBEAT_CONSUMER ||
        args->errorBit == CO_EM_HB_CONSUMER_REMOTE_RESET) {
      return;
    }

    static const uint8_t err_mask[CO_EM_MANUFACTURER_START / 8] = {0};
    bool err_status_set =
        memcmp(co.CO->em->errorStatusBits, err_mask, sizeof(err_mask));
    err_t errors = states_get_errors();

    // have to check both set and err_status_set since we could only access
    // error status bits now and it might have been set and cleared before we
    // could process it
    if (set && err_status_set && !(errors & ERR_CODE_CAN)) {
      states_set_errors(ERR_CODE_CAN, true);
    } else if (!set && !err_status_set && errors & ERR_CODE_CAN) {
      states_set_errors(ERR_CODE_CAN, false);
    }
    return;
  }

  int id = args->ident - CO_EMCY_COB_ID_BASE;

  if (set) {
    LOG_ERR(
        "Received EMCY object from node %d: error set: %d, reg: %d, error bit: "
        "%d, info: %d",
        id, args->errorCode, args->errorRegister, args->errorBit,
        args->infoCode);
  } else {
    LOG_INF(
        "Received EMCY object from node %d: error cleared, reg: %d, error bit: "
        "%d, info: %d",
        id, args->errorRegister, args->errorBit, args->infoCode);
  }

  int idx = CO_HBconsumer_getIdxByNodeId(co.CO->HBcons, id);
  if (idx != -1) {
    if (args->errorRegister != 0) {
      node_mon_states[idx] |= NODE_MON_ERR;
    } else {
      node_mon_states[idx] &= ~NODE_MON_ERR;
    }

    node_mon_update_err_code(id, node_mon_states[idx]);
  }
}

static void time_cb(void *arg) {
  (void)arg;

  k_work_submit(&time_work_data);
}

static void time_work(struct k_work *work) {
  (void)work;

  int ret;

  CO_NMT_internalState_t NMTstate = CO_NMT_getInternalState(co.CO->NMT);
  if (NMTstate != CO_NMT_PRE_OPERATIONAL && NMTstate != CO_NMT_OPERATIONAL) {
    return;
  }

  CO_TIME_process(co.CO->TIME, true, 0);
  LOG_INF("Received TIME object, days: %d, ms: %d", co.CO->TIME->days,
          co.CO->TIME->ms);

  if (time_set) {
    return;
  }

  time_t time =
      ((time_t)co.CO->TIME->days * SEC_OF_DAY + co.CO->TIME->ms / 1000) +
      CO_SEC_REF;

  char time_str[] = "1970-01-01T00:00:00";
  strftime(time_str, sizeof(time_str), "%FT%T", gmtime(&time));
  LOG_INF("Setting system time to %lld (%s)", time, time_str);

  ret = sys_set_time(time);
  if (ret < 0) {
    LOG_ERR("Failed to set time: %s", strerror(-ret));
  } else {
    time_set = true;
  }
}

static void hb_mnt_changed_cb(uint8_t id, uint8_t idx,
                              CO_NMT_internalState_t state, void *arg) {
  (void)arg;

  if (state != CO_NMT_OPERATIONAL) {
    node_mon_states[idx] |= NODE_MON_NOT_OPT;
    LOG_WRN("Node %d NMT state changed to non-operational", id);

  } else {
    node_mon_states[idx] &= ~NODE_MON_NOT_OPT;
    LOG_INF("Node %d NMT state changed to operational", id);
  }

  node_mon_update_err_code(id, node_mon_states[idx]);
}

static void hb_started_cb(uint8_t id, uint8_t idx, void *arg) {
  (void)arg;

  node_mon_states[idx] &= ~NODE_MON_TIMEOUT;
  LOG_INF("Node %d heartbeat active", id);

  node_mon_update_err_code(id, node_mon_states[idx]);
}

static void hb_timeout_cb(uint8_t id, uint8_t idx, void *arg) {
  (void)arg;

  node_mon_states[idx] |= NODE_MON_TIMEOUT;
  LOG_ERR("Node %d heartbeat timeout", id);

  node_mon_update_err_code(id, node_mon_states[idx]);
}

static void hb_reset_cb(uint8_t id, uint8_t idx, void *arg) {
  (void)arg;

  node_mon_states[idx] = NODE_MON_INITIAL;
  LOG_INF("Node %d reset", id);

  node_mon_update_err_code(id, node_mon_states[idx]);
}

static void error_chan_cb(const struct zbus_channel *chan) {
  err_t err = *(err_t *)zbus_chan_const_msg(chan);
  bool set = FLAG_SET_AND_CLEAR(err, ERR_CODE_SET);

  enum err_code code;
  ERR_CODE_FOR_EACH(err, code) {
    CO_error(co.CO->em, set, ERR_CODE_TO_CO_ERR_STATUS(code),
             ERR_CODE_TO_CO_ERR_CODE(code), 0);
  }
}

static int init() {
  int ret;

  NODE_MON_INIT(NODE_MON_LIST);

  ret = canopen_init(&co);
  if (ret < 0) {
    LOG_ERR("Failed to initialize CANopen: %s", strerror(-ret));
    states_set_errors(ERR_CODE_CAN, true);
    return ret;
  }

  CO_EM_initCallbackRx(co.CO->em, emcy_cb);
  CO_TIME_initCallbackPre(co.CO->TIME, NULL, time_cb);
  NODE_MON_REG_HBCON_CB(co.CO->HBcons, NODE_MON_LIST);
  OD_extension_init(OD_ENTRY_H1000, &od_imu_accel_ext);
  return 0;
}

static enum err_code node_mon_id_to_err_code(enum co_node_id id) {
  switch (id) {
    case CO_NODE_ID_RB:
      return ERR_CODE_NODE_RB;

    case CO_NODE_ID_ACC:
      return ERR_CODE_NODE_ACC;

    case CO_NODE_ID_INV_FL:
      return ERR_CODE_NODE_INV_FL;

    case CO_NODE_ID_INV_FR:
      return ERR_CODE_NODE_INV_FR;

    case CO_NODE_ID_INV_RL:
      return ERR_CODE_NODE_INV_RL;

    case CO_NODE_ID_INV_RR:
      return ERR_CODE_NODE_INV_RR;

    default:
      return 0;
  }
}

static void node_mon_update_err_code(enum co_node_id id, bool err) {
  enum err_code node_err_code = node_mon_id_to_err_code(id);
  err_t errors = states_get_errors();

  if (err && !(errors & node_err_code)) {
    states_set_errors(node_err_code, true);
  } else if (!err && errors & node_err_code) {
    states_set_errors(node_err_code, false);
  }
}
