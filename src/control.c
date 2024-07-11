// zephyr includes
#include <zephyr/zbus/zbus.h>

// project includes
#include "nturt/sensors.h"

ZBUS_SUBSCRIBER_DEFINE(sensor_data_sub, 1);
ZBUS_CHAN_ADD_OBS(sensor_data_chan, sensor_data_sub, 0);
