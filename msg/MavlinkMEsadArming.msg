# ESAD arming command for one or more ESAD instances
#
# MAVLink-M message ESAD_ARMING (id 53031).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint8 esad_id # Vehicle-local ESAD instance identifier to arm or disarm. 0 = all ESAD instances selected by store_id, 1-254 = specific ESAD instance, 255 = reserved.
uint32 arming_challenge_hash # Challenge hash matching the selected ESAD instance challenge. For esad_id = 0, the same hash must be active on every targeted ESAD.
uint8 arming_request # Requested arming state change.
uint8 store_id # Target store instance identifier. 0 = all stores / broadcast.
