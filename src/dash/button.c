#define DT_DRV_COMPAT nturt_dash_keys

#include "dash.h"

// glibc includes
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// zephyr incldues
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// lib includes
#include <canopennode.h>

LOG_MODULE_REGISTER(input_dash, CONFIG_INPUT_LOG_LEVEL);

struct dash_keys_data {
  uint32_t buttons;

  OD_extension_t ext;
};

struct dash_keys_config {
  int index;

  const uint16_t* map;

  uint32_t mask;
};

static ODR_t dash_keys_od_write(OD_stream_t* stream, const void* buf,
                                OD_size_t size, OD_size_t* size_written) {
  if (stream == NULL || buf == NULL || size_written == NULL) {
    return ODR_DEV_INCOMPAT;
  }

  ODR_t ret = OD_writeOriginal(stream, buf, size, size_written);
  if (ret != ODR_OK) {
    return ret;
  }

  const struct device* dev = stream->object;
  struct dash_keys_data* data = dev->data;
  const struct dash_keys_config* config = dev->config;

  uint8_t curr;
  memcpy(&curr, stream->dataOrig, sizeof(curr));

  uint8_t bit;
  while ((bit = __builtin_ffs(data->buttons ^ (curr & config->mask))) != 0) {
    bit--;
    data->buttons ^= BIT(bit);

    input_report_key(dev, config->map[bit], !!(data->buttons & BIT(bit)), true,
                     K_FOREVER);
  }

  return ODR_OK;
}

int dash_keys_init(const struct device* dev, struct canopen* co) {
  (void)co;

  struct dash_keys_data* data = dev->data;
  const struct dash_keys_config* config = dev->config;

  OD_entry_t* entry = OD_find(OD, config->index);
  if (entry == NULL) {
    LOG_ERR("OD entry not found: 0x%04X", config->index);
    return -ENOENT;
  }

  OD_extension_init(entry, &data->ext);

  return 0;
}

#define DASH_KEYS_INIT(inst)                                                  \
  static struct dash_keys_data dash_keys_data_##inst = {                      \
      .buttons = 0,                                                           \
      .ext =                                                                  \
          {                                                                   \
              .read = OD_readOriginal,                                        \
              .write = dash_keys_od_write,                                    \
              .object = (void*)DEVICE_DT_GET(DT_INST(inst, DT_DRV_COMPAT)),   \
          },                                                                  \
  };                                                                          \
                                                                              \
  static const int16_t map_##inst[] = {                                       \
      DT_INST_FOREACH_PROP_ELEM_SEP(inst, zephyr_code, DT_PROP_BY_IDX, (, )), \
  };                                                                          \
                                                                              \
  static const struct dash_keys_config dash_keys_config_##inst = {            \
      .index = DT_INST_PROP(inst, index),                                     \
      .map = map_##inst,                                                      \
      .mask = BIT_MASK(DT_INST_PROP_LEN(inst, zephyr_code)),                  \
  };                                                                          \
                                                                              \
  DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &dash_keys_data_##inst,             \
                        &dash_keys_config_##inst, POST_KERNEL,                \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DASH_KEYS_INIT)
