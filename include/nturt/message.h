#ifndef NTURT_MESSAGE_H_
#define NTURT_MESSAGE_H_

// zephyr includes
#include <zephyr/sys/util.h>

// lib includes
#include <CANopen.h>

// project includes
#include "nturt/states.h"
#include "nturt/util.h"

/**
 * @addtogroup Message Message
 *
 * @{
 */

/* macro ---------------------------------------------------------------------*/
/**
 * @addtogroup ErrorMacro
 *
 * @{
 */

/**
 * @addtogroup ErrorCOMacro CANopen Error Macros
 *
 * @{
 */

/// @brief Prefix for CANopen error code names.
#define ERR_CO_ERR_CODE_PREFIX CO_EMC_

/// @brief Prefix for CANopenNode error status bit names.
#define ERR_CO_ERR_STATUS_PREFIX CO_EM_

#define _ERR_CO_ERR_CODE_DECLARE(I, X) \
  CONCAT(ERR_CO_ERR_CODE_PREFIX, X) = CO_EMC_DEVICE_SPECIFIC + I

/**
 * @brief Declare the values of CANopen error codes using enum.
 *
 * @note The values for the error codes are 0xFF00 (CANopen device specific) +
 * the index of the error code in the list.
 *
 * @param LIST List of error codes defined by @ref ERROR_CODE_LISTIFY.
 * @return Enum of CANopen error codes.
 */
#define ERR_CO_ERR_CODE_DECLARE(LIST) \
  enum CO_EM_error_code { FOR_EACH_IDX(_ERR_CO_ERR_CODE_DECLARE, (, ), LIST) }

#define _ERR_CO_ERR_STATUS_DECLARE(I, X) \
  CONCAT(ERR_CO_ERR_STATUS_PREFIX, X) = CO_EM_MANUFACTURER_START + I

/**
 * @brief Declare the values of CANopenNode error status bits using enum.
 *
 * @note The values for the error status bits are 0x30
 * (CO_EM_MANUFACTURER_START) + the index of the error code in the list.
 *
 * @param LIST List of error codes defined by @ref ERROR_CODE_LISTIFY.
 * @return Enum of CANopenNode error status bits.
 */
#define ERR_CO_ERR_STATUS_DECLARE(LIST) \
  enum CO_EM_status_bit { FOR_EACH_IDX(_ERR_CO_ERR_STATUS_DECLARE, (, ), LIST) }

/**
 * @brief Convert the value of error code to the value of CANopen error code.
 *
 * @param ERR Error code.
 * @return CANopen error code.
 */
#define ERR_CODE_TO_CO_ERR_CODE(ERR) \
  (__builtin_ffs(ERR) + CO_EMC_DEVICE_SPECIFIC)

/**
 * @brief Convert the value of CANopen error code to the value of error code.
 *
 * @param ERR CANopen error code.
 * @return Error code.
 */
#define ERR_CO_ERR_CODE_TO_CODE(ERR) BIT(ERR - CO_EMC_DEVICE_SPECIFIC)

/**
 * @brief Convert the value of error code to the value of CANopenNode error
 * status bit.
 *
 * @param ERR Error code.
 * @return CANopenNode error status bit.
 */
#define ERR_CODE_TO_CO_ERR_STATUS(ERR) \
  (__builtin_ffs(ERR) + CO_EM_MANUFACTURER_START)

/**
 * @brief Convert the value of CANopenNode error status bit to the value of
 * error code.
 *
 * @param ERR CANopenNode error status bit.
 * @return Error code.
 */
#define ERR_CO_ERR_STATUS_TO_CODE(ERR) BIT(ERR - CO_EM_MANUFACTURER_START)

/**
 * @} // ErrorCOMacro
 */

/**
 * @} // ErrorMacro
 */

/**
 * @addtogroup MessgesMacro
 *
 * @{
 */

/**
 * @addtogroup NodeMonitorMacro Node Monitor Macros
 *
 * @{
 */

/// @brief Node monitor states.
#define NODE_MON_ERR BIT(0)

#define NODE_MON_NOT_OPT BIT(1)

#define NODE_MON_TIMEOUT BIT(2)

/// @brief Initial state of monitored node.
#define NODE_MON_INITIAL (NODE_MON_NOT_OPT | NODE_MON_TIMEOUT)

/**
 * @brief Listify monitored node ID. this should be in the same ID value and
 * order as OD entry 0x1016.
 *
 * @param ... List of node ID.
 * @return List of monitored nodes.
 */
#define NODE_MON_LISTIFY(...) __VA_ARGS__

#define NODE_MON_DEFINE(NAME, LIST)                        \
  static uint8_t NAME[] = {                                \
      [0 ... NUM_VA_ARGS_LESS_1(LIST)] = NODE_MON_INITIAL, \
  };

#define _NODE_MON_INIT(ID) states_set_errors(node_mon_id_to_err_code(ID), true)

/**
 * @brief Initialize monitored node, setting errors for each node.
 *
 * @param LIST List of monitored nodes.
 */
#define NODE_MON_INIT(LIST) FOR_EACH(_NODE_MON_INIT, (;), LIST)

#define _NODE_MON_REG_CB(I, ID, HBCON)                                       \
  CO_HBconsumer_initCallbackNmtChanged(HBCON, I, NULL, hb_mnt_changed_cb);   \
  CO_HBconsumer_initCallbackHeartbeatStarted(HBCON, I, NULL, hb_started_cb); \
  CO_HBconsumer_initCallbackTimeout(HBCON, I, NULL, hb_timeout_cb);          \
  CO_HBconsumer_initCallbackRemoteReset(HBCON, I, NULL, hb_reset_cb)

/**
 * @brief Register heartbeat consumer callbacks for monitored node.
 *
 * @param HBCON Heartbeat consumer object.
 * @param LIST List of monitored nodes.
 */
#define NODE_MON_REG_HBCON_CB(HBCON, LIST) \
  FOR_EACH_IDX_FIXED_ARG(_NODE_MON_REG_CB, (;), HBCON, LIST)

/**
 * @} // NodeMonitorMacro
 */

/**
 * @addtogroup ReceptionMacro Reception Macros
 *
 * @{
 */

/**
 * @brief Define a RPDO reception variable.
 *
 * @param TYPE Type of the variable, must be the same as defined in the object
 * dictionary.
 * @param NAME Name of the variable, does not need to be the same as defined in
 * the object dictionary.
 */
#define REC_VAR(TYPE, NAME) TYPE, NAME

#define _REC_GROUP_VAR(TYPE, NAME) TYPE NAME

#define _REC_GROUP_ENUM(IDX, TYPE, NAME, GROUP) __##GROUP##_##NAME = BIT(IDX)

/**
 * @brief Group multiple RPDO reception variables together into a struct named
 * @p NAME and declare other necessary enums for receiving RPDOs. The struct
 * contains a member @p cb which is a callback function that is called when all
 * variables in the group are received.
 *
 * @param NAME Name of the RPDO reception group.
 */
#define REC_GROUP(NAME, ...)                                               \
  struct NAME {                                                            \
    FOR_EACH_PAIR(_REC_GROUP_VAR, (;), __VA_ARGS__);                       \
    void (*cb)(const struct NAME *);                                       \
    uint32_t __updated;                                                    \
  };                                                                       \
                                                                           \
  enum {                                                                   \
    FOR_EACH_PAIR_IDX_FIXED_ARG(_REC_GROUP_ENUM, (, ), NAME, __VA_ARGS__), \
    __##NAME##_mask = BIT_MASK(NUM_VA_ARGS(__VA_ARGS__) / 2),              \
  }

/**
 * @brief Define an instance of a RPDO reception group and its callback function
 * that publishes itself to a zbus channel.
 *
 * @param NAME Name of the instance.
 * @param GROUP Group of RPDO reception group defined by @ref REC_GROUP.
 * @param CHAN Zbus channel to publish the instance.
 */
#define REC_GROUP_DEFINE(NAME, GROUP, CHAN)                      \
  static void __##NAME##_cb(const struct GROUP *data) {                \
    int ret;                                                     \
                                                                 \
    ret = zbus_chan_pub(&CHAN, data, K_MSEC(5));                 \
    if (ret < 0) {                                               \
      LOG_ERR("Failed to publish" #NAME ": %s", strerror(-ret)); \
    }                                                            \
  }                                                              \
                                                                 \
  static struct GROUP NAME = {                                   \
      .cb = __##NAME##_cb,                                       \
  }

/**
 * @brief Declare the subindex of a RPDO reception variable in the object
 * dictionary.
 *
 * @param NAME Name of the reception variable.
 * @param SUBIDX Subindex of the reception variable in the object dictionary.
 */
#define REC_OD_VAR(NAME, SUBIDX) NAME, SUBIDX

#define _REC_OD_WRITE_CASE(VAR, SUBIDX, GROUP)                \
  case SUBIDX:                                                \
    memcpy(&dest->VAR, stream->dataOrig, stream->dataLength); \
    dest->__updated &= __##GROUP##_##VAR;                     \
    break

/**
 * @brief Define the write function for a object dictionary extension.
 *
 * @param NAME Name of the function.
 * @param GROUP Group of RPDO reception variables defined by @ref REC_VAR_GROUP.
 * @param ... List of RPDO reception variables defined by @ref REC_OD_VAR.
 */
#define REC_OD_WRITE_DEFINE(NAME, GROUP, ...)                               \
  static ODR_t NAME(OD_stream_t *stream, const void *buf, OD_size_t size,   \
                    OD_size_t *size_written) {                              \
    if (stream == NULL || buf == NULL || size_written == NULL) {            \
      return ODR_DEV_INCOMPAT;                                              \
    }                                                                       \
                                                                            \
    ODR_t ret = OD_writeOriginal(stream, buf, size, size_written);          \
                                                                            \
    if (ret != ODR_OK) {                                                    \
      return ret;                                                           \
    }                                                                       \
                                                                            \
    struct GROUP *dest = stream->object;                                    \
    switch (stream->subIndex) {                                             \
      FOR_EACH_PAIR_FIXED_ARG(_REC_OD_WRITE_CASE, (;), GROUP, __VA_ARGS__); \
                                                                            \
      default:                                                              \
        return ODR_OK;                                                      \
    }                                                                       \
                                                                            \
    if ((dest->__updated & __##GROUP##_mask) == __##GROUP##_mask) {         \
      if (dest->cb != NULL) {                                               \
        dest->cb(dest);                                                     \
      }                                                                     \
                                                                            \
      dest->__updated = 0;                                                  \
    }                                                                       \
                                                                            \
    return ODR_OK;                                                          \
  }

/**
 * @brief Define an object dictionary extension for a RPDO reception group and
 * bind the RPDO reception variables to the object dictionary entry.
 *
 * @param NAME Name of the object dictionary extension.
 * @param GROUP Group of RPDO reception variables defined by @ref REC_VAR_GROUP.
 * @param GROUP_VAR Variable of the RPDO reception group defined by @ref
 * REC_GROUP_DEFINE.
 * @param ... List of RPDO reception variables in the group defined by @ref
 * REC_OD_VAR to be bound to the object dictionary entry.
 */
#define REC_OD_EXT_DEFINE(NAME, GROUP, GROUP_VAR, ...)       \
  REC_OD_WRITE_DEFINE(__od_write_##NAME, GROUP, __VA_ARGS__) \
                                                             \
  OD_extension_t NAME = {                                    \
      .object = GROUP_VAR,                                   \
      .read = OD_readOriginal,                               \
      .write = __od_write_##NAME,                            \
  }

/**
 * @} // ReceptionMacro
 */

/**
 * @} // MessgesMacro
 */

/* types ---------------------------------------------------------------------*/
/// @brief CANopen node ID.
enum co_node_id {
  CO_NODE_ID_FB = 0x01,
  CO_NODE_ID_RB,

  CO_NODE_ID_RPI = 0x04,

  CO_NODE_ID_IMU = 0x08,

  CO_NODE_ID_ACC = 0x10,
  CO_NODE_ID_INV_FL,
  CO_NODE_ID_INV_FR,
  CO_NODE_ID_INV_RL,
  CO_NODE_ID_INV_RR,
};

ERR_CO_ERR_CODE_DECLARE(ERR_CODE_LIST);

ERR_CO_ERR_STATUS_DECLARE(ERR_CODE_LIST);

REC_GROUP(msg_imu, REC_VAR(int16_t, accel_x), REC_VAR(int16_t, accel_y),
          REC_VAR(int16_t, accel_z));

/* exported variables --------------------------------------------------------*/
ZBUS_CHAN_DECLARE(imu_data_chan);

/**
 * @} // Message
 */

#endif  // NTURT_MESSAGE_H_
