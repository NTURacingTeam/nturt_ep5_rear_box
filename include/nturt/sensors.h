#ifndef NTURT_SENSORS_H_
#define NTURT_SENSORS_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

/* types----------------------------------------------------------------------*/
/// @brief Sensor data structure.
struct sensor_data {
  /// @brief Steering angle in degrees.
  float steer;

  /// @brief Accelerator pedal position sensor in [0, 1].
  float apps;

  /// @brief Brake pedal position sensor in [0, 1].
  float bse;

  /// @brief Accelerator pedal position sensor 1 raw data in degrees.
  float apps1_raw;

  /// @brief Accelerator pedal position sensor 2 raw data in degrees.
  float apps2_raw;

  /// @brief Brake pedal position sensor raw data in kilopascal.
  float bse_f_raw;

  /// @brief Brake pedal position sensor raw data in kilopascal.
  float bse_r_raw;

  /// @brief Accelerator engaged.
  bool accel_micro;

  /// @brief Brake engaged.
  bool break_micro;

  /// @brief Suspension sensor raw data in meters.
  float susp_dive_r;

  /// @brief Suspension sensor raw data in meters.
  float susp_roll_r;
};

/* exported variables --------------------------------------------------------*/
ZBUS_CHAN_DECLARE(sensor_data_chan);

#endif  // NTURT_SENSORS_H_
