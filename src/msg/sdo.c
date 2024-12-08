#include "sdo.h"

// glibc includes
#include <string.h>

// lib includes
#include <canopennode.h>

// nturt includes
#include <nturt/err.h>
#include <nturt/rear_box/cmd.h>

// project includes
#include "ctrl.h"
#include "msg.h"
#include "sensors.h"
#include "states.h"
#include "sys.h"

/* macro ---------------------------------------------------------------------*/
#define CMD_ENTRY OD_ENTRY_H2080
#define CMD_SUBINDEX 0

#define CTRL_MODE_ENTRY OD_ENTRY_H2081
#define CTRL_MODE_SUBINDEX 0

#define STATE_ENTRY OD_ENTRY_H20F0
#define STATES_SUBINDEX 1
#define ERRORS_SUBINDEX 2

/* static function declaration -----------------------------------------------*/
static ODR_t cmd_write(OD_stream_t *stream, const void *buf, OD_size_t size,
                       OD_size_t *size_written);

static ODR_t state_read(OD_stream_t *stream, void *buf, OD_size_t size,
                        OD_size_t *size_read);

static ODR_t ctrl_mode_read(OD_stream_t *stream, void *buf, OD_size_t size,
                            OD_size_t *size_read);
static ODR_t ctrl_mode_write(OD_stream_t *stream, const void *buf,
                             OD_size_t size, OD_size_t *size_written);

/* static variable -----------------------------------------------------------*/
OD_extension_t cmd_od_ext = {
    .read = NULL,
    .write = cmd_write,
};

OD_extension_t ctrl_mode_od_ext = {
    .read = ctrl_mode_read,
    .write = ctrl_mode_write,
};

OD_extension_t state_od_ext = {
    .read = state_read,
    .write = NULL,
};

/* function definition -------------------------------------------------------*/
int sdo_init() {
  OD_extension_init(CMD_ENTRY, &cmd_od_ext);
  OD_extension_init(CTRL_MODE_ENTRY, &ctrl_mode_od_ext);
  OD_extension_init(STATE_ENTRY, &state_od_ext);

  return 0;
}

/* static function definition ------------------------------------------------*/
static ODR_t cmd_write(OD_stream_t *stream, const void *buf, OD_size_t size,
                       OD_size_t *size_written) {
  if (stream == NULL || stream->subIndex != 0 || buf == NULL ||
      size != sizeof(uint8_t) || size_written == NULL) {
    return ODR_DEV_INCOMPAT;
  }

  uint8_t data;
  memcpy(&data, buf, size);
  *size_written = size;

  switch (data) {
    case RB_CMD_RTD:
      states_cmd(STATES_CMD_RTD);
      break;

    case RB_CMD_DISABLE:
      states_cmd(STATES_CMD_DISABLE);
      break;

    case RB_CMD_SET_HOME:
      steer_set_zero_pos();
      break;

    case RB_CMD_RESET:
      msg_reset_all_nodes();
      sys_reset();
      break;

    case RB_CMD_INV_RESET:
      states_cmd(STATES_CMD_DISABLE);
      states_inv_reset();
      break;

    default:
      return ODR_DEV_INCOMPAT;
  }

  return ODR_OK;
}

static ODR_t state_read(OD_stream_t *stream, void *buf, OD_size_t size,
                        OD_size_t *size_read) {
  if (stream == NULL || buf == NULL || size_read == NULL) {
    return ODR_DEV_INCOMPAT;
  }

  switch (stream->subIndex) {
    case STATES_SUBINDEX: {
      if (size < sizeof(states_t)) {
        return ODR_DEV_INCOMPAT;
      }

      states_t data = states_get_states();
      memcpy(buf, &data, sizeof(data));
      *size_read = sizeof(data);

      break;
    }

    case ERRORS_SUBINDEX: {
      if (size < sizeof(err_t)) {
        return ODR_DEV_INCOMPAT;
      }

      err_t data = err_get_errors();
      memcpy(buf, &data, sizeof(data));
      *size_read = sizeof(data);

      break;
    }

    default:
      return ODR_DEV_INCOMPAT;
  }

  return ODR_OK;
}

static ODR_t ctrl_mode_read(OD_stream_t *stream, void *buf, OD_size_t size,
                            OD_size_t *size_read) {
  if (stream == NULL || stream->subIndex != 0 || buf == NULL ||
      size < sizeof(uint8_t) || size_read == NULL) {
    return ODR_DEV_INCOMPAT;
  }

  uint8_t data = ctrl_mode_get();
  memcpy(buf, &data, sizeof(data));
  *size_read = sizeof(data);

  return ODR_OK;
}

static ODR_t ctrl_mode_write(OD_stream_t *stream, const void *buf,
                             OD_size_t size, OD_size_t *size_written) {
  if (stream == NULL || stream->subIndex != 0 || buf == NULL ||
      size != sizeof(uint8_t) || size_written == NULL) {
    return ODR_DEV_INCOMPAT;
  }

  uint8_t data;
  memcpy(&data, buf, size);
  *size_written = size;

  states_cmd(STATES_CMD_DISABLE);
  ctrl_mode_set(data);

  return ODR_OK;
}
