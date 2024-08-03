// glibc includes
#include <stdio.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/auxdisplay.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

// nturt includes
#include <nturt/err.h>

/* macro ---------------------------------------------------------------------*/
#define ERR_DISP_PERIOD K_MSEC(500)

/* type ----------------------------------------------------------------------*/
struct err_disp {
  enum err_code disp_val;
  struct k_work_delayable update_work;
};

/* static function definition ------------------------------------------------*/
static void err_disp_update_work(struct k_work *work);

static int init();

/* static variable -----------------------------------------------------------*/
const static struct device *error_disp =
    DEVICE_DT_GET(DT_NODELABEL(error_disp));

static struct err_disp err_disp = {
    .update_work = Z_WORK_DELAYABLE_INITIALIZER(err_disp_update_work),
};

SYS_INIT(init, APPLICATION, UTIL_INC(CONFIG_NTURT_ERR_INIT_PRIORITY));

/* function definition -------------------------------------------------------*/
static void err_disp_update_work(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct err_disp *err_disp = CONTAINER_OF(dwork, struct err_disp, update_work);

  err_t errors = err_get_errors();
  if (errors == 0) {
    err_disp->disp_val = 0;
    auxdisplay_clear(error_disp);
  } else {
    do {
      err_disp->disp_val =
          __builtin_ffs(errors & ~BIT_MASK(err_disp->disp_val));
    } while (err_disp->disp_val == 0);

    char buf[5];
    snprintf(buf, sizeof(buf), "%2d", err_disp->disp_val);
    auxdisplay_write(error_disp, buf, 2);
  }

  k_work_reschedule(&err_disp->update_work, ERR_DISP_PERIOD);
}

static int init() {
  k_work_reschedule(&err_disp.update_work, ERR_DISP_PERIOD);

  return 0;
}
