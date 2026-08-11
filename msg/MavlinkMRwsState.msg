# Remote Weapon System (RWS) operational state including arming status and loaded weapon information
#
# MAVLink-M message RWS_STATE (id 53041).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
char[50] weapon_string # Human-readable weapon description or designation (null-terminated).
uint8 arming_state # Current arming state of the weapon system.
uint8 weapon_type # Weapon type identifier (implementation-defined).
