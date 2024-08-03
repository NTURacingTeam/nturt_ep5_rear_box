#ifndef MSG_H_
#define MSG_H_

// glibc includes
#include <stdint.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

// nturt includes
#include <nturt/msg.h>

/**
 * @addtogroup Msg Message
 *
 * @{
 */

/* macro ---------------------------------------------------------------------*/
/**
 * @addtogroup MsgMacro Message Macros
 *
 * @{
 */

/// @brief CANopen node ID of the rear box.
#define MSG_NODE_ID CO_NODE_ID_RB

/**
 * @} // MsgMacro
 */

/* type ----------------------------------------------------------------------*/
struct status_data {
  uint8_t acc;

  struct {
    uint8_t fl;
    uint8_t fr;
    uint8_t rl;
    uint8_t rr;
  } inv;
};

/// @brief Inverter data structure.
struct inv_data {
  struct {
    float speed;
    float torque;
    float voltage;
    float current;
  } fl, fr, rl, rr;
};

/// @brief IMU data structure.
struct imu_data {
  struct {
    float x;
    float y;
    float z;
  } accel, gyro;
};

/* exported variable ---------------------------------------------------------*/
ZBUS_CHAN_DECLARE(status_data_chan);
ZBUS_CHAN_DECLARE(inv_data_chan);
ZBUS_CHAN_DECLARE(imu_data_chan);

/**
 * @} // Msg
 */

#endif  // MSG_H_
