#ifndef DEFILE_H_
#define DEFILE_H_

// zephyr includes
#include <zephyr/input/input.h>

// nturt include
#include <nturt/nturt.h>

/// @brief Rear box LED numbers.
#define LED_NUM_CANOPEN_RUN 1
#define LED_NUM_CANOPEN_ERR 2
#define LED_NUM_BRAKE_LIGHT 3
#define LED_NUM_RTD_SOUND 4

/// @brief Rear box button keys.
#define INPUT_KEY_BUILTIN INPUT_KEY_0
#define INPUT_KEY_APPS INPUT_KEY_A
#define INPUT_KEY_BSE INPUT_KEY_B

#endif  // DEFILE_H_
