# Health and current reading of one deceleration/landing-trigger sensor on a store
#
# MAVLink-M message STORE_SENSOR_STATUS (id 53034).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
float32 value # Current sensor reading in the sensor's native units. NaN if invalid.
float32 value_min # Lower bound of the valid operating range, native units. NaN if unspecified.
float32 value_max # Upper bound of the valid operating range, native units. NaN if unspecified.
uint8 store_id # Store instance identifier this sensor belongs to.
uint8 sensor_type # Type of sensor.
uint8 sensor_index # Index when multiple sensors of the same type exist on a store (0-based).
uint8 health # Health state of this sensor.
