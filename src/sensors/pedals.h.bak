#ifndef SENSORS_PEDALS_H_
#define SENSORS_PEDALS_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/smf.h>

// nturt includes
#include <nturt/err.h>

// project includes
#include "amt21.h"
#include "sensors.h"

/* macro ---------------------------------------------------------------------*/
#define SENSORS_PEDALS_APPS_INITIALIZER()                                      \
  {                                                                            \
      .apps1 = SENSORS_AMT21_INITIALIZER(DT_NODELABEL(apps1), ERR_CODE_APPS1), \
      .apps2 = SENSORS_AMT21_INITIALIZER(DT_NODELABEL(apps2), ERR_CODE_APPS2), \
      .set_zero = false,                                                       \
  }

#define SENSORS_PEDALS_BSE_INITIALIZER() \
  {}

#define SENSORS_PEDALS_INITIALIZER()             \
  {                                              \
      .apps = SENSORS_PEDALS_APPS_INITIALIZER(), \
      .bse = SENSORS_PEDALS_BSE_INITIALIZER(),   \
  }

/* type ----------------------------------------------------------------------*/
struct apps_sensor {
  struct amt21 amt21;

  /// @brief Error level.
  int level;
};

struct apps {
#if IS_ENABLED(CONFIG_APPS_PLAUS)
  struct smf_ctx plaus_smf;
  k_timepoint_t plaus_time;
#endif  // CONFIG_APPS_PLAUS

  struct amt21 apps1, apps2;
  bool set_zero;

  float travel1, travel2;
  struct pedal_data data;
};

struct bse {
  float val1, val2;

  struct pedal_data data;
};

struct pedals {
#if IS_ENABLED(CONFIG_PEDAL_PLAUS)
  struct smf_ctx plaus_smf;
#endif  // CONFIG_PEDAL_PLAUS

  struct apps apps;
  struct bse bse;
};

/* function declaration ------------------------------------------------------*/
void pedals_init(struct pedals *pedals);

void pedals_update(struct pedals *pedals);

#endif  // SENSORS_PEDALS_H_
