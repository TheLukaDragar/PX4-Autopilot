# Per-store roll-up summary for release decisions
#
# MAVLink-M message STORE_STATUS (id 53033).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 sensor_fault_mask # Bitmask of MAVLINK_M_STORE_SENSOR_TYPE values currently NOT healthy (bit = 1&lt;&lt;sensor_type).
uint8 store_id # Store instance identifier.
uint8 store_count # Total number of stores on the carrier (lets the GCS know how many to expect). 0 if unknown.
uint8 readiness # Roll-up release-readiness state.
uint8 arming_status # ESAD arming state of this store (mirrored for convenience).
uint8 sensors_healthy # Count of sensors reporting OK.
uint8 sensors_total # Total count of sensors on this store.
