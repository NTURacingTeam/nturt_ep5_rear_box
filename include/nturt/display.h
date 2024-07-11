#ifndef NTURT_DISPLAY_H_
#define NTURT_DISPLAY_H_

// glibc includes
#include <stdint.h>

// zephyr includes
#include <zephyr/sys/util.h>

#define DISPLAY_DIGIT_NUM 2

#define DISPLAY_BASE_10 10
#define DISPLAY_BASE_16 16

/*
 *    0
 *   ---
 * 5|   |1
 *   -6-
 * 4|   |2
 *   ---  .
 *    3    7
 */

#define CHAR_OFF (0)

#define CHAR_0 (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5))
#define CHAR_1 (BIT(1) | BIT(2))
#define CHAR_2 (BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(6))
#define CHAR_3 (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(6))
#define CHAR_4 (BIT(1) | BIT(2) | BIT(5) | BIT(6))
#define CHAR_5 (BIT(0) | BIT(2) | BIT(3) | BIT(5) | BIT(6))
#define CHAR_6 (BIT(0) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_7 (BIT(0) | BIT(1) | BIT(2) | BIT(5))
#define CHAR_8 (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_9 (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(5) | BIT(6))

#define CHAR_A (BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_C (BIT(0) | BIT(3) | BIT(4) | BIT(5))
#define CHAR_E (BIT(0) | BIT(3) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_F (BIT(0) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_H (BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_L (BIT(3) | BIT(4) | BIT(5))
#define CHAR_P (BIT(0) | BIT(1) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_U (BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5))

#define CHAR_b (BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_d (BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(6))
#define CHAR_h (BIT(2) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_i (BIT(2))
#define CHAR_o (BIT(2) | BIT(3) | BIT(4) | BIT(6))
#define CHAR_r (BIT(4) | BIT(6))
#define CHAR_t (BIT(3) | BIT(4) | BIT(5) | BIT(6))
#define CHAR_u (BIT(2) | BIT(3) | BIT(4))

#define CHAR_DASH (BIT(6))
#define CHAR_OVERLINE (BIT(0))
#define CHAR_UNDERSCORE (BIT(3))
#define CHAR_PIPE (BIT(4) | BIT(5))

/**
 * @brief Display two characters.
 *
 * @param chars Characters to display, must be assigned by CHAR_XXX macros.
 * @return 0 on success, negative error code on failure.
 */
int display_char(const uint8_t chars[2]);

/**
 * @brief Turn off display.
 *
 * @return 0 on success, negative error code on failure.
 */
int display_off();

/**
 * @brief Display a number.
 *
 * @param num Number to display.
 * @param base Number base, must be assigned @ref DISPLAY_BASE_10 or @ref
 * DISPLAY_BASE_16.
 * @return 0 on success, negative error code on failure.
 */
int display_num(int num, int base);

#endif  // NTURT_DISPLAY_H_
