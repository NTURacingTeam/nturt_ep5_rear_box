#ifndef STATUS_RTD_SND_H_
#define STATUS_RTD_SND_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/kernel.h>

/* macro ---------------------------------------------------------------------*/
#define RTD_SND_INITIALIZER()                                           \
  {                                                                     \
      .on = false,                                                      \
      .count = 0,                                                       \
      .update_work = Z_WORK_DELAYABLE_INITIALIZER(rtd_snd_update_work), \
  }

/* type ----------------------------------------------------------------------*/
struct rtd_snd {
  bool on;
  int count;
  struct k_work_delayable update_work;
};

/* function declaration ------------------------------------------------------*/
void rtd_snd_play(struct rtd_snd *rtd_snd);

void rtd_snd_stop(struct rtd_snd *rtd_snd);

void rtd_snd_update_work(struct k_work *work);

#endif  // STATUS_RTD_SND_H_
