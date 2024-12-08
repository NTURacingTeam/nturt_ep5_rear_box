#ifndef SENSORS_H_
#define SENSORS_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

/* types----------------------------------------------------------------------*/
struct pedal_data {
  /// @brief Pedal travel in [0, 1].
  float travel;

  /// @brief Pedal travel raw data, apps in degrees, bse in kilopascal.
  float raw[2];

  /// @brief Is pedal engaged.
  bool engaged;
};

/// @brief Sensor data structure.
struct sensor_data {
  struct {
    /// @brief Power voltage in volts.
    float v_24;

    /// @brief Power current in amperes.
    float i_24;

    /// @brief Power current in amperes.
    float i_5;
  } power;

  /// @brief Steering angle in degrees.
  float steer;

  struct pedal_data apps, bse;

  struct {
    /// @brief Suspension travel in meters.
    float dive;

    /// @brief Suspension travel in meters.
    float roll;
  } susp;
};

/* exported variables --------------------------------------------------------*/
ZBUS_CHAN_DECLARE(sensor_data_chan);

/* function declaration ------------------------------------------------------*/
bool sensors_apps_engaged();

bool sensors_bse_engaged();

void steer_set_zero_pos();
#endif  // SENSORS_H_
