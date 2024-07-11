#ifndef NTURT_UTIL_LOOPS_H_
#define NTURT_UTIL_LOOPS_H_

#define Z_FOR_EACH_PAIR_GET_ARG(                                               \
    _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16,     \
    _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, \
    _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, \
    _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, \
    _62, _63, _64, _65, _66, _67, _68, _69, _70, _71, _72, _73, _74, _75, _76, \
    _77, _78, _79, _80, _81, _82, _83, _84, _85, _86, _87, _88, _89, _90, _91, \
    _92, _93, _94, _95, _96, _97, _98, _99, _100, _101, _102, _103, _104,      \
    _105, _106, _107, _108, _109, _110, _111, _112, _113, _114, _115, _116,    \
    _117, _118, _119, _120, _121, _122, _123, _124, _125, _126, _127, _128, N, \
    ...)                                                                       \
  N

#define Z_FOR_EACH_PAIR_0(z_call, sep, fixed_arg0, fixed_arg1, ...)

#define Z_FOR_EACH_PAIR_1(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  z_call(0, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_2(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_1(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(1, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_3(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_2(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(2, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_4(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_3(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(3, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_5(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_4(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(4, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_6(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_5(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(5, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_7(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_6(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(6, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_8(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_7(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(7, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_9(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_8(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(8, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_10(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_9(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)    \
  __DEBRACKET sep z_call(9, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_11(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_10(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(10, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_12(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_11(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(11, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_13(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_12(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(12, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_14(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_13(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(13, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_15(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_14(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(14, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_16(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_15(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(15, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_17(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_16(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(16, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_18(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_17(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(17, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_19(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_18(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(18, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_20(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_19(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(19, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_21(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_20(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(20, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_22(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_21(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(21, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_23(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_22(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(22, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_24(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_23(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(23, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_25(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_24(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(24, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_26(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_25(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(25, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_27(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_26(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(26, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_28(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_27(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(27, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_29(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_28(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(28, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_30(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_29(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(29, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_31(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_30(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(30, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_32(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_31(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(31, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_33(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_32(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(32, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_34(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_33(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(33, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_35(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_34(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(34, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_36(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_35(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(35, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_37(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_36(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(36, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_38(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_37(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(37, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_39(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_38(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(38, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_40(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_39(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(39, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_41(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_40(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(40, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_42(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_41(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(41, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_43(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_42(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(42, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_44(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_43(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(43, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_45(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_44(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(44, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_46(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_45(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(45, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_47(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_46(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(46, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_48(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_47(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(47, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_49(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_48(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(48, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_50(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_49(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(49, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_51(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_50(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(50, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_52(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_51(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(51, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_53(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_52(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(52, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_54(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_53(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(53, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_55(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_54(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(54, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_56(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_55(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(55, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_57(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_56(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(56, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_58(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_57(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(57, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_59(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_58(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(58, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_60(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_59(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(59, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_61(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_60(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(60, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_62(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_61(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(61, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_63(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_62(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(62, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_64(z_call, sep, fixed_arg0, fixed_arg1, x, y, ...) \
  Z_FOR_EACH_PAIR_63(z_call, sep, fixed_arg0, fixed_arg1, ##__VA_ARGS__)   \
  __DEBRACKET sep z_call(63, x, y, fixed_arg0, fixed_arg1)

#define Z_FOR_EACH_PAIR_ERR(...) \
  _Static_assert(0, "Z_FOR_EACH_PAIR: invalid number of arguments")

#define Z_FOR_EACH_PAIR_ENGINE(z_call, sep, fixed_arg0, fixed_arg1, ...) \
  Z_FOR_EACH_PAIR_GET_ARG(                                               \
      __VA_ARGS__, Z_FOR_EACH_PAIR_64, Z_FOR_EACH_PAIR_ERR,              \
      Z_FOR_EACH_PAIR_63, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_62,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_61, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_60, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_59,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_58, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_57, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_56,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_55, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_54, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_53,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_52, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_51, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_50,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_49, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_48, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_47,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_46, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_45, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_44,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_43, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_42, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_41,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_40, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_39, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_38,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_37, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_36, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_35,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_34, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_33, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_32,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_31, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_30, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_29,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_28, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_27, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_26,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_25, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_24, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_23,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_22, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_21, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_20,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_19, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_18, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_17,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_16, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_15, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_14,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_13, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_12, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_11,       \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_10, Z_FOR_EACH_PAIR_ERR,      \
      Z_FOR_EACH_PAIR_9, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_8,         \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_7, Z_FOR_EACH_PAIR_ERR,       \
      Z_FOR_EACH_PAIR_6, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_5,         \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_4, Z_FOR_EACH_PAIR_ERR,       \
      Z_FOR_EACH_PAIR_3, Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_2,         \
      Z_FOR_EACH_PAIR_ERR, Z_FOR_EACH_PAIR_1, Z_FOR_EACH_PAIR_ERR,       \
      Z_FOR_EACH_PAIR_0)                                                 \
  (z_call, sep, fixed_arg0, fixed_arg1, __VA_ARGS__)

#define Z_FOR_EACH_PAIR_IDX_FIXED_ARG_EXEC(idx, x, y, fixed_arg0, fixed_arg1) \
  fixed_arg0(idx, x, y, fixed_arg1)

#define Z_FOR_EACH_PAIR_IDX_FIXED_ARG(F, sep, fixed_arg, ...)        \
  Z_FOR_EACH_PAIR_ENGINE(Z_FOR_EACH_PAIR_IDX_FIXED_ARG_EXEC, sep, F, \
                         fixed_arg, __VA_ARGS__)

#define Z_FOR_EACH_PAIR_FIXED_ARG_EXEC(idx, x, y, fixed_arg0, fixed_arg1) \
  fixed_arg0(x, y, fixed_arg1)

#define Z_FOR_EACH_PAIR_FIXED_ARG(F, sep, fixed_arg, ...)                   \
  Z_FOR_EACH_PAIR_ENGINE(Z_FOR_EACH_PAIR_FIXED_ARG_EXEC, sep, F, fixed_arg, \
                         __VA_ARGS__)

#define Z_FOR_EACH_PAIR_IDX_EXEC(idx, x, y, fixed_arg0, fixed_arg1) \
  fixed_arg0(idx, x, y)

#define Z_FOR_EACH_PAIR_IDX(F, sep, ...) \
  Z_FOR_EACH_PAIR_ENGINE(Z_FOR_EACH_PAIR_IDX_EXEC, sep, F, _, __VA_ARGS__)

#define Z_FOR_EACH_PAIR_EXEC(idx, x, y, fixed_arg0, fixed_arg1) fixed_arg0(x, y)

#define Z_FOR_EACH_PAIR(F, sep, ...) \
  Z_FOR_EACH_PAIR_ENGINE(Z_FOR_EACH_PAIR_EXEC, sep, F, _, __VA_ARGS__)

#define Z_BYPASS_PAIR(x, y) x, y

#endif  // NTURT_UTIL_LOOPS_H_
