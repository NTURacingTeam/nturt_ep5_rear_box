#include "msg.h"

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// lib includes
#include <canopennode.h>

// nturt includes
#include <nturt/convert.h>
#include <nturt/err.h>
#include <nturt/msg.h>

// project includes
#include "ctrl.h"
#include "dash.h"
#include "sensors.h"
#include "states.h"
#include "sys.h"

LOG_MODULE_REGISTER(msg);

/* macro ---------------------------------------------------------------------*/
/// @brief CANopen time reference in POSIX epoch time in seconds.
#define CO_SEC_REF 441763200UL

/// @brief Number of seconds in a day.
#define SEC_OF_DAY 86400

/* type ----------------------------------------------------------------------*/
struct msg {
  struct canopen co;

  bool time_set;
  struct k_work time_work;
};

/* static function declaration -----------------------------------------------*/
/// @brief Callback function when receiving button events.
static void buttons_cb(struct input_event *evt);

/// @brief Callback function when receiving CANopen TIME object.
static void time_cb(void *arg);

/// @brief Bottom half of @ref time_cb.
static void time_work(struct k_work *work);

/// @brief Callback function when receiving node status data.
static void status_rx_cb(const void *data);

/// @brief Callback function when receiving inverter data.
static void inv_rx_cb(const void *data);

/// @brief Callback function when receiving IMU data.
static void imu_rx_cb(const void *data);

/// @brief Callback function when receiving from error channel.
static void err_chan_cb(const struct zbus_channel *chan);

/// @brief Callback function when receiving from state channel.
static void state_chan_cb(const struct zbus_channel *chan);

/// @brief Callback function when receiving from control mode channel.
static void ctrl_mode_chan_cb(const struct zbus_channel *chan);

/// @brief Callback function when receiving from states command channel.
static void status_cmd_chan_cb(const struct zbus_channel *chan);

/// @brief Callback function when receiving from sensor data channel.
static void sensor_data_chan_cb(const struct zbus_channel *chan);

/// @brief Callback function when receiving from control channel.
static void ctrl_data_chan_cb(const struct zbus_channel *chan);

/// @brief Initialization function for message module.
static int init();

/* static varaible -----------------------------------------------------------*/
static const struct device *dash_buttons =
    DEVICE_DT_GET(DT_NODELABEL(dash_buttons));
static const struct device *dash_leds = DEVICE_DT_GET(DT_NODELABEL(dash_leds));

#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE
CANOPEN_STORAGE_DEFINE(
    storage_entries,
    CANOPEN_STORAGE_ENTRY(OD_PERSIST_COMM, 0x02,
                          CO_storage_cmd | CO_storage_restore),
    // CANOPEN_STORAGE_ENTRY(OD_EEPROM, 0x03,
    //                       CO_storage_cmd | CO_storage_restore),
);
#endif  // CO_CONFIG_STORAGE

static struct msg msg = {
    .co =
        {
            .can_dev = DEVICE_DT_GET(DT_ALIAS(canopen_can)),
            .green_led = GPIO_DT_SPEC_GET(DT_ALIAS(canopen_green_led), gpios),
            .red_led = GPIO_DT_SPEC_GET(DT_ALIAS(canopen_red_led), gpios),
            .node_id = MSG_NODE_ID,
            .bitrate = MSG_CAN_BAUDRATE,
            .nmt_control =
                CO_NMT_ERR_REG_MASK | CO_NMT_STARTUP_TO_OPERATIONAL |
                /* CO_NMT_ERR_ON_BUSOFF_HB | CO_NMT_ERR_ON_ERR_REG | */
                CO_NMT_ERR_FREE_TO_OPERATIONAL,
#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE
            .storage_entries = storage_entries,
            .storage_entries_count = ARRAY_SIZE(storage_entries),
#endif  // CO_CONFIG_STORAGE
        },

    .time_set = false,
    .time_work = Z_WORK_INITIALIZER(time_work),
};

INPUT_CALLBACK_DEFINE(NULL, buttons_cb);

ZBUS_CHAN_DEFINE(status_data_chan, struct status_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(imu_data_chan, struct imu_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(inv_data_chan, struct inv_data, NULL, NULL,
                 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

ZBUS_LISTENER_DEFINE(msg_err_chan_listener, err_chan_cb);
ZBUS_CHAN_ADD_OBS(err_chan, msg_err_chan_listener, 0);

ZBUS_LISTENER_DEFINE(msg_state_chan_listener, state_chan_cb);
ZBUS_CHAN_ADD_OBS(state_chan, msg_state_chan_listener, 0);

ZBUS_LISTENER_DEFINE(msg_ctrl_mode_chan_listener, ctrl_mode_chan_cb);
ZBUS_CHAN_ADD_OBS(ctrl_mode_chan, msg_ctrl_mode_chan_listener, 0);

ZBUS_LISTENER_DEFINE(msg_status_cmd_chan_listener, status_cmd_chan_cb);
ZBUS_CHAN_ADD_OBS(status_cmd_chan, msg_status_cmd_chan_listener, 0);

ZBUS_LISTENER_DEFINE(msg_sensor_data_chan_listener, sensor_data_chan_cb);
ZBUS_CHAN_ADD_OBS(sensor_data_chan, msg_sensor_data_chan_listener, 0);

ZBUS_LISTENER_DEFINE(msg_control_chan_listener, ctrl_data_chan_cb);
ZBUS_CHAN_ADD_OBS(ctrl_data_chan, msg_control_chan_listener, 0);

RX_GROUP(status_raw, status_rx_cb, RX_VAR(uint8_t, acc),
#if IS_ENABLED(FRONT_INVERTER)
         RX_VAR(uint16_t, inv_fl), RX_VAR(uint16_t, inv_fr),
#endif  // FRONT_INVERTER
         RX_VAR(uint16_t, inv_rl), RX_VAR(uint16_t, inv_rr));
#if IS_ENABLED(FRONT_INVERTER)
RX_OD_BIND(0x2191, status_raw, RX_OD_VAR(inv_fl, 0));
RX_OD_BIND(0x21A1, status_raw, RX_OD_VAR(inv_fr, 0));
#endif  // FRONT_INVERTER
RX_OD_BIND(0x21B1, status_raw, RX_OD_VAR(inv_rl, 0));
RX_OD_BIND(0x21C1, status_raw, RX_OD_VAR(inv_rr, 0));

RX_GROUP(inv_raw, inv_rx_cb,
#if IS_ENABLED(FRONT_INVERTER)
         RX_VAR(int16_t, speed_fl), RX_VAR(int16_t, torque_fl),
         RX_VAR(uint16_t, voltage_fl), RX_VAR(int16_t, current_fl),
         RX_VAR(int16_t, speed_fr), RX_VAR(int16_t, torque_fr),
         RX_VAR(uint16_t, voltage_fr), RX_VAR(int16_t, current_fr),
#else
         RX_VAR(uint16_t, speed_fl), RX_VAR(uint16_t, speed_fr),
#endif  // FRONT_INVERTER
         RX_VAR(int16_t, speed_rl), RX_VAR(int16_t, torque_rl),
         RX_VAR(uint16_t, voltage_rl), RX_VAR(int16_t, current_rl),
         RX_VAR(int16_t, speed_rr), RX_VAR(int16_t, torque_rr),
         RX_VAR(uint16_t, voltage_rr), RX_VAR(int16_t, current_rr));
#if IS_ENABLED(FRONT_INVERTER)
RX_OD_BIND(0x2194, inv_raw, RX_OD_VAR(speed_fl, 1), RX_OD_VAR(torque_fl, 2));
RX_OD_BIND(0x2195, inv_raw, RX_OD_VAR(voltage_fl, 1), RX_OD_VAR(current_fl, 2));
RX_OD_BIND(0x21A4, inv_raw, RX_OD_VAR(speed_fr, 1), RX_OD_VAR(torque_fr, 2));
RX_OD_BIND(0x21A5, inv_raw, RX_OD_VAR(voltage_fr, 1), RX_OD_VAR(current_fr, 2));
#else
RX_OD_BIND(0x2300, inv_raw, RX_OD_VAR(speed_fl, 0));
RX_OD_BIND(0x2301, inv_raw, RX_OD_VAR(speed_fr, 0));
#endif  // FRONT_INVERTER
RX_OD_BIND(0x21B4, inv_raw, RX_OD_VAR(speed_rl, 1), RX_OD_VAR(torque_rl, 2));
RX_OD_BIND(0x21B5, inv_raw, RX_OD_VAR(voltage_rl, 1), RX_OD_VAR(current_rl, 2));
RX_OD_BIND(0x21C4, inv_raw, RX_OD_VAR(speed_rr, 1), RX_OD_VAR(torque_rr, 2));
RX_OD_BIND(0x21C5, inv_raw, RX_OD_VAR(voltage_rr, 1), RX_OD_VAR(current_rr, 2));

RX_GROUP(imu_raw, imu_rx_cb, RX_VAR(int16_t, accel_x), RX_VAR(int16_t, accel_y),
         RX_VAR(int16_t, accel_z), RX_VAR(int16_t, gyro_x),
         RX_VAR(int16_t, gyro_y), RX_VAR(int16_t, gyro_z));
RX_OD_BIND(0x2320, imu_raw, RX_OD_VAR(accel_x, 1), RX_OD_VAR(accel_y, 2),
           RX_OD_VAR(accel_z, 3));
RX_OD_BIND(0x2321, imu_raw, RX_OD_VAR(gyro_x, 1), RX_OD_VAR(gyro_y, 2),
           RX_OD_VAR(gyro_z, 3));

TX_OD_DECLARE(0x20C0, 0x2100, 0x2101, 0x2102, 0x2140, 0x2190, 0x2192, 0x21A0,
              0x21A2, 0x21B0, 0x21B2, 0x21C0, 0x21C2, 0x2311);

SYS_INIT(init, APPLICATION, CONFIG_NTURT_MSG_INIT_PRIORITY);

/* static function definition ------------------------------------------------*/
static void buttons_cb(struct input_event *evt) {
  if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_POWER && evt->value) {
    for (int i = 0; i < msg.co.CO->HBcons->numberOfMonitoredNodes; i++) {
      uint32_t id;
      OD_get_u32(OD_ENTRY_H1016, i + 1, &id, true);
      id = (id >> 16) & 0xFF;

      CO_NMT_sendCommand(msg.co.CO->NMT, CO_NMT_RESET_NODE, id);
    }

    sys_reset();
  }
}

static void time_cb(void *arg) {
  struct msg *msg = arg;

  k_work_submit(&msg->time_work);
}

static void time_work(struct k_work *work) {
  struct msg *msg = CONTAINER_OF(work, struct msg, time_work);

  int ret;

  CO_NMT_internalState_t NMTstate = CO_NMT_getInternalState(msg->co.CO->NMT);
  if (NMTstate != CO_NMT_PRE_OPERATIONAL && NMTstate != CO_NMT_OPERATIONAL) {
    return;
  }

  CO_TIME_process(msg->co.CO->TIME, true, 0);
  LOG_INF("Received TIME object, days: %d, ms: %d", msg->co.CO->TIME->days,
          msg->co.CO->TIME->ms);

  if (msg->time_set) {
    return;
  }

  time_t time = ((time_t)msg->co.CO->TIME->days * SEC_OF_DAY +
                 msg->co.CO->TIME->ms / 1000) +
                CO_SEC_REF;

  char time_str[] = "1970-01-01T00:00:00";
  strftime(time_str, sizeof(time_str), "%FT%T", gmtime(&time));
  LOG_INF("Setting system time to %lld (%s)", time, time_str);

  ret = sys_set_time(time);
  if (ret < 0) {
    LOG_ERR("Failed to set time: %s", strerror(-ret));
  } else {
    msg->time_set = true;
  }
}

static void status_rx_cb(const void *_data) {
  const struct status_raw *data = _data;

  struct status_data status_data = {
      .acc = data->acc,
      .inv =
          {
#if IS_ENABLED(FRONT_INVERTER)
              .fl = data->inv_fl,
              .fr = data->inv_fr,
#endif  // FRONT_INVERTER
              .rl = data->inv_rl,
              .rr = data->inv_rr,
          },
  };

  int ret;
  ret = zbus_chan_pub(&status_data_chan, &status_data, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish inverter data: %s", strerror(-ret));
  }
}

static void inv_rx_cb(const void *_data) {
  const struct inv_raw *data = _data;

  struct inv_data inv_data = {
#if IS_ENABLED(FRONT_INVERTER)
      .fl.speed = INV_SPEED_CAN_TO_PHY(data->speed_fl),
      .fl.torque = INV_TORQUE_CAN_TO_PHY(data->torque_fl),
      .fl.voltage = INV_VOLTAGE_CAN_TO_PHY(data->voltage_fl),
      .fl.current = INV_CURRENT_CAN_TO_PHY(data->current_fl),

      .fr.speed = INV_SPEED_CAN_TO_PHY(data->speed_fr),
      .fr.torque = INV_TORQUE_CAN_TO_PHY(data->torque_fr),
      .fr.voltage = INV_VOLTAGE_CAN_TO_PHY(data->voltage_fr),
      .fr.current = INV_CURRENT_CAN_TO_PHY(data->current_fr),
#else
      .fl.speed = WHEEL_SPEED_CAN_TO_PHY(data->speed_fl),
      .fr.speed = WHEEL_SPEED_CAN_TO_PHY(data->speed_fr),
#endif  // FRONT_INVERTER

      .rl.speed = INV_SPEED_CAN_TO_PHY(data->speed_rl),
      .rl.torque = INV_TORQUE_CAN_TO_PHY(data->torque_rl),
      .rl.voltage = INV_VOLTAGE_CAN_TO_PHY(data->voltage_rl),
      .rl.current = INV_CURRENT_CAN_TO_PHY(data->current_rl),

      .rr.speed = INV_SPEED_CAN_TO_PHY(data->speed_rr),
      .rr.torque = INV_TORQUE_CAN_TO_PHY(data->torque_rr),
      .rr.voltage = INV_VOLTAGE_CAN_TO_PHY(data->voltage_rr),
      .rr.current = INV_CURRENT_CAN_TO_PHY(data->current_rr),
  };

  int ret;
  ret = zbus_chan_pub(&inv_data_chan, &inv_data, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish inverter data: %s", strerror(-ret));
  }
}

static void imu_rx_cb(const void *_data) {
  const struct imu_raw *data = _data;

  struct imu_data imu_data = {
      .accel.x = IMU_ACCEL_CAN_TO_PHY(data->accel_x),
      .accel.y = IMU_ACCEL_CAN_TO_PHY(data->accel_y),
      .accel.z = IMU_ACCEL_CAN_TO_PHY(data->accel_z),

      .gyro.x = IMU_GYRO_CAN_TO_PHY(data->gyro_x),
      .gyro.y = IMU_GYRO_CAN_TO_PHY(data->gyro_y),
      .gyro.z = IMU_GYRO_CAN_TO_PHY(data->gyro_z),
  };

  int ret;
  ret = zbus_chan_pub(&imu_data_chan, &imu_data, K_MSEC(5));
  if (ret < 0) {
    LOG_ERR("Failed to publish imu data: %s", strerror(-ret));
  }
}

static void err_chan_cb(const struct zbus_channel *chan) {
  TX_OD_SET(0x20C0, 2, err_get_errors(), u32);
}

static void state_chan_cb(const struct zbus_channel *chan) {
  const state_t *state = zbus_chan_const_msg(chan);

  TX_OD_SET(0x20C0, 1, *state, u16);
}

static void ctrl_mode_chan_cb(const struct zbus_channel *chan) {
  const enum ctrl_mode *mode = zbus_chan_const_msg(chan);

  TX_OD_SET(0x20C0, 3, *mode, u8);
}

static void status_cmd_chan_cb(const struct zbus_channel *chan) {
  const struct status_cmd *data = zbus_chan_const_msg(chan);

#if IS_ENABLED(FRONT_INVERTER)

  TX_OD_SET(0x2190, 0, data->inv_ctrl_word.fl, u16);
  TX_OD_SET(0x21A0, 0, data->inv_ctrl_word.fr, u16);
#endif  // FRONT_INVERTER
  TX_OD_SET(0x21B0, 0, data->inv_ctrl_word.rl, u16);
  TX_OD_SET(0x21C0, 0, data->inv_ctrl_word.rr, u16);
}

static void sensor_data_chan_cb(const struct zbus_channel *chan) {
  const struct sensor_data *data = zbus_chan_const_msg(chan);

  if (msg.co.CO == NULL || !msg.co.CO->CANmodule->CANnormal) {
    return;
  }

  CO_LOCK_OD(msg.co.CO->CANmodule);

  TX_OD_SEND(0x2100, 0, STEER_PHY_TO_CAN(data->steer), i16);

  TX_OD_SEND(0x2101, 1, PEDAL_TRAV_PHY_TO_CAN(data->apps.travel), u8);
  TX_OD_SEND(0x2101, 2, APPS_RAW_PHY_TO_CAN(data->apps.raw[0]), i8);
  TX_OD_SEND(0x2101, 3, APPS_RAW_PHY_TO_CAN(data->apps.raw[1]), i8);
  TX_OD_SEND(0x2101, 4, data->apps.engaged, u8);

  TX_OD_SEND(0x2102, 1, PEDAL_TRAV_PHY_TO_CAN(data->bse.travel), u8);
  TX_OD_SEND(0x2102, 2, BSE_RAW_PHY_TO_CAN(data->bse.raw[0]), u8);
  TX_OD_SEND(0x2102, 3, BSE_RAW_PHY_TO_CAN(data->bse.raw[0]), u8);
  TX_OD_SEND(0x2102, 4, data->bse.engaged, u8);

  TX_OD_SEND(0x2311, 1, SUSP_PHY_TO_CAN(data->susp.dive), u8);
  TX_OD_SEND(0x2311, 2, SUSP_PHY_TO_CAN(data->susp.roll), u8);

  CO_UNLOCK_OD(msg.co.CO->CANmodule);
}

static void ctrl_data_chan_cb(const struct zbus_channel *chan) {
  const struct ctrl_data *data = zbus_chan_const_msg(chan);

  if (msg.co.CO == NULL || !msg.co.CO->CANmodule->CANnormal) {
    return;
  }

  CO_LOCK_OD(msg.co.CO->CANmodule);

  TX_OD_SEND(0x2140, 0, data->speed, i16);

#if IS_ENABLED(FRONT_INVERTER)
  TX_OD_SEND(0x2192, 0, data->fl.torque_cmd, i16);
  TX_OD_SEND(0x21A2, 0, data->fr.torque_cmd, i16);
#endif  // FRONT_INVERTER
  TX_OD_SEND(0x21B2, 0, data->rl.torque_cmd, i16);
  TX_OD_SEND(0x21C2, 0, data->rr.torque_cmd, i16);

  CO_UNLOCK_OD(msg.co.CO->CANmodule);
}

static int init() {
  int ret;

  // must be called before canopen_init
  ret = dash_keys_init(dash_buttons, &msg.co);
  if (ret < 0) {
    LOG_ERR("Failed to initialize dash buttons: %s", strerror(-ret));
    return ret;
  }

  ret = dash_led_init(dash_leds, &msg.co);
  if (ret < 0) {
    LOG_ERR("Failed to initialize dash leds: %s", strerror(-ret));
    return ret;
  }

  ret = msg_init(&msg.co);
  if (ret < 0) {
    LOG_ERR("Failed to initialize message module: %s", strerror(-ret));
    return ret;
  }

  CO_TIME_initCallbackPre(msg.co.CO->TIME, &msg, time_cb);

  return 0;
}
