#ifndef NTURT_NTURT_H_
#define NTURT_NTURT_H_

// zephyr includes
#include <zephyr/input/input.h>

#define BBRAM_OFFSET_RTC_SET 0
#define BBRAM_SIZE_RTC_SET 1

#define LED_NUM_BUILTIN 0
#define LED_NUM_CANOPEN_RUN 1
#define LED_NUM_CANOPEN_ERR 2
#define LED_NUM_BREAK_LIGHT 3
#define LED_NUM_RTD_SOUND 4

#define INPUT_KEY_BUILTIN INPUT_KEY_0
#define INPUT_KEY_ACCEL INPUT_KEY_A
#define INPUT_KEY_BREAK INPUT_KEY_B

#endif // NTURT_NTURT_H_
