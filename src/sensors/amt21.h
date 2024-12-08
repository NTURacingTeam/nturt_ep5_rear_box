#ifndef SENSORS_AMT21_H_
#define SENSORS_AMT21_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

// nturt includes
#include <nturt/sensors.h>

/* macro ---------------------------------------------------------------------*/
#define SENSORS_AMT21_INITIALIZER(NODE_ID, ERR)                        \
  {                                                                    \
      .tol = SENSOR_TOL_INITIALIZER(NODE_ID, CONFIG_SENSORS_TOL_THRES, \
                                    CONFIG_SNESORS_TOL_WEIGHT, ERR),   \
      .starting = ATOMIC_INIT(false),                                  \
  }

/* type ----------------------------------------------------------------------*/
struct amt21 {
  struct sensor_tol tol;

  struct k_timer set_zero_pos_timer;
  atomic_t starting;
};

/* function declaration ------------------------------------------------------*/
int amt21_init(struct amt21* amt21);

bool amt21_is_ok(struct amt21* amt21);

int amt21_read(struct amt21* amt21, float* val);

int amt21_reset(struct amt21* amt21);

int amt21_set_zero_pos(struct amt21* amt21);

#endif  // SENSORS_AMT21_H_
