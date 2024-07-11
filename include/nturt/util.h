#ifndef NTURT_UTIL_H_
#define NTURT_UTIL_H_

// glibc includes
#include <stddef.h>

// zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

// project includes
#include "nturt/util_loops.h"

/**
 * @addtogroup Util Util
 *
 */

/* macros --------------------------------------------------------------------*/
/**
 * @addtogroup UtilMacro Utility Macros
 *
 * @{
 */

/**
 * @brief Macro that discards all arguments and expend to zephyr @ref EMPTY.
 *
 * @param ... Variable list of arguments to discard.
 * @return zephyr @ref EMPTY.
 */
#define DISCARD(...) EMPTY

/**
 * @brief Similar to Zephyr @ref REVERSE_ARGS but reverses the arguments in
 * pairs.
 *
 * @note Only accepts even number of arguments and up to 64 pairs.
 * @param ... Variable argument list.
 */
#define REVERSE_PAIRS(...)                                             \
  Z_FOR_EACH_PAIR_ENGINE(Z_FOR_EACH_PAIR_EXEC, (, ), Z_BYPASS_PAIR, _, \
                         __VA_ARGS__)

/**
 * @brief Similar to Zephyr @ref FOR_EACH but iterates over pairs of arguments.
 *
 * @note Only accepts even number of arguments and up to 64 pairs.
 * @param F Macro to invoke.
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses; this
 * is required to enable providing a comma as separator.
 * @param ... Variable argument list. The macro @p F is invoked as <tt>F(x,
 * y)</tt> for each pair (x, y) in the list.
 */
#define FOR_EACH_PAIR(F, sep, ...) \
  Z_FOR_EACH_PAIR(F, sep, REVERSE_PAIRS(__VA_ARGS__))

/**
 * @brief Similar to Zephyr @ref FOR_EACH_IDX but iterates over pairs of
 * arguments.
 *
 * @note Only accepts even number of arguments and up to 64 pairs.
 * @param F Macro to invoke.
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses; this
 * is required to enable providing a comma as separator.
 * @param ... Variable argument list. The macro @p F is invoked as <tt>F(index,
 * x, y)</tt> for each pair (x, y) in the list.
 */
#define FOR_EACH_PAIR_IDX(F, sep, ...) \
  Z_FOR_EACH_PAIR_IDX(F, sep, REVERSE_PAIRS(__VA_ARGS__))

/**
 * @brief Similar to Zephyr @ref FOR_EACH_FIXED_ARG but iterates over pairs of
 * arguments.
 *
 * @note Only accepts even number of arguments and up to 64 pairs.
 * @param F Macro to invoke.
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses; this
 * is required to enable providing a comma as separator.
 * @param fixed_arg Fixed argument passed to @p F as the second macro parameter.
 * @param ... Variable argument list. The macro @p F is invoked as
 * <tt>F(x, y, fixed_arg)</tt> for each pair (x, y) in the list.
 */
#define FOR_EACH_PAIR_FIXED_ARG(F, sep, fixed_arg, ...) \
  Z_FOR_EACH_PAIR_FIXED_ARG(F, sep, fixed_arg, REVERSE_PAIRS(__VA_ARGS__))

/**
 * @brief Similar to Zephyr @ref FOR_EACH_IDX_FIXED_ARG but iterates over pairs
 * of arguments.
 *
 * @note Only accepts even number of arguments and up to 64 pairs.
 * @param F Macro to invoke.
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses; this
 * is required to enable providing a comma as separator.
 * @param fixed_arg Fixed argument passed to @p F as the second macro parameter.
 * @param ... Variable argument list. The macro @p F is invoked as
 * <tt>F(index, x, y, fixed_arg)</tt> for each pair (x, y) in the list.
 */
#define FOR_EACH_PAIR_IDX_FIXED_ARG(F, sep, fixed_arg, ...) \
  Z_FOR_EACH_PAIR_IDX_FIXED_ARG(F, sep, fixed_arg, REVERSE_PAIRS(__VA_ARGS__))

/**
 * @brief Same as zephyr @ref GET_GET_ARG_N but accepts macro expansion for @p N
 * .
 *
 * @param N Argument index to fetch. Counter from 1.
 * @param ... Variable list of arguments from which one argument is returned.
 * @return Nth argument.
 */
#define GET_ARG_N_FIXED(N, ...) UTIL_CAT(Z_GET_ARG_, N)(__VA_ARGS__)

/**
 * @brief Check if a flag is set and clear that flag if it so.
 *
 * @param NUM Number to check.
 * @param FLAG Flag to check.
 * @return True if the flag is set.
 */
#define FLAG_SET_AND_CLEAR(NUM, FLAG) ((NUM & FLAG) && ((NUM &= ~FLAG)))

/**
 * @brief Check if a bit is set and clear that bit if it so.
 *
 * @param NUM Number to check.
 * @param BIT_ Bit to check.
 * @return True if the bit is set.
 */
#define BIT_SET_AND_CLEAR(NUM, BIT_) FLAG_SET_AND_CLEAR(NUM, BIT(BIT_))

/**
 * @} // UtilMacro
 */

/**
 * @addtogroup WorkContext Work Context
 *
 * @{
 */

#define _WORK_CTX_BUF_DEFINE(I, WORK, ARGS) \
  [I] = {.work = Z_WORK_INITIALIZER(WORK), .args = ARGS[I]}

/**
 * @brief Define work context buffer for the bottom half of an ISR.
 *
 * @param NAME Name of the work context buffer.
 * @param SIZE Size of the work context buffer, i.e. the number of simultaneous
 * bottom half can be executed.
 * @param WORK Bottom half entry point.
 * @param ARGS_TYPE Type of the arguments for the bottom half.
 */
#define WORK_CTX_BUF_DEFINE(NAME, SIZE, WORK, ARGS_TYPE) \
  static ARGS_TYPE __work_args_##NAME[SIZE];             \
                                                         \
  static struct work_ctx NAME[] = {                      \
      LISTIFY(SIZE, _WORK_CTX_BUF_DEFINE, (, ), WORK, &__work_args_##NAME)}

/**
 * @brief Get the arguments for the bottom half.
 *
 * @param WORK Work pointer passed to the bottom half.
 * @return Arguments for the bottom half.
 */
#define WORK_CTX_ARGS(WORK) \
  (((struct work_ctx*)CONTAINER_OF(WORK, struct work_ctx, work))->args)

/**
 * @} // WorkContext
 */

/* types ---------------------------------------------------------------------*/
/**
 * @addtogroup WorkContext
 *
 * @{
 */

/// @brief Work context for the bottom half.
struct work_ctx {
  /// @brief Work for the bottom half.
  struct k_work work;

  /// @brief Arguments for the bottom half.
  void* args;
};

/**
 * @} // WorkContext
 */

/* function declaration ------------------------------------------------------*/
/**
 * @addtogroup WorkContext
 *
 * @{
 */

/**
 * @brief Allocate a work context. Return NULL if all work context are in use.
 *
 * @param buf Work context buffer to allocate from.
 * @param size Size of the work context buffer.
 * @return Allocated work context. NULL if all work context are in use.
 */
struct work_ctx* work_ctx_alloc(struct work_ctx* buf, size_t size);

/**
 * @} // WorkContext
 */

/**
 * @} // Util
 */

#endif  // NTURT_UTIL_H_
