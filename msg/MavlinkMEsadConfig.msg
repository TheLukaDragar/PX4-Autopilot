# Authenticated configuration command for an ESAD that accepts live configuration of a safe/arm par...
#
# MAVLink-M message ESAD_CONFIG (id 53032).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 arming_challenge_hash # Challenge hash matching the active ESAD_STATE hash for the target store.
uint8 esad_id # Vehicle-local ESAD instance identifier to configure. 0 = all ESAD instances selected by store_id, 1-254 = specific ESAD instance, 255 = reserved.
uint8 store_id # Target store instance identifier. 0 = all stores / broadcast.
uint8 trigger_distance_mode # Requested coarse trigger-distance standoff mode. Coarse selection only.
