#ifndef SENSORS_H_
#define SENSORS_H_

// glibc includes
#include <stdbool.h>

// zephyr includes
#include <zephyr/zbus/zbus.h>

/* types----------------------------------------------------------------------*/
/// @brief Sensor data structure.
struct sensor_data {
  /// @brief Steering angle in degrees.
  float steer;

  struct {
    /// @brief Pedal travel in [0, 1].
    float travel;

    /// @brief Pedal travel raw data, apps in degrees, bse in kilopascal.
    float raw[2];

    /// @brief Is pedal engaged.
    bool engaged;
  } apps, bse;

  struct {
    /// @brief Suspension travel in meters.
    float dive;

    /// @brief Suspension travel in meters.
    float roll;
  } susp;
};

/* exported variables --------------------------------------------------------*/
ZBUS_CHAN_DECLARE(sensor_data_chan);

#endif  // SENSORS_H_
