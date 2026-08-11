# Electronic Safe and Arm Device (ESAD) telemetry for one ESAD instance on a store
#
# MAVLink-M message ESAD_STATE (id 53030).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint8 esad_id # Vehicle-local ESAD instance identifier. Must be 1-254. 0 is invalid for ESAD_STATE, 255 is reserved. Systems that support broadcast arming set the same arming_challenge_hash on each target ESAD before accepting ESAD_ARMING.esad_id = 0.
uint32 arming_challenge_hash # Challenge hash for arming authentication of this ESAD instance.
uint32 fault_flags # Active fault flag bitmask.
float32 input_1 # Analog/digital input 1 (implementation-defined). For the standard safety inputs use the typed fields below instead.
float32 input_2 # Analog/digital input 2 (implementation-defined). For the standard safety inputs use the typed fields below instead.
uint8[8] sw_version_hash # Software version as 8-byte short Git hash.
uint8 arming_status # Current arming state.
uint8 munition_status # Current munition state.
uint8 ignition_status # Current ignition circuit state.
uint8 munition_type # Munition/store type identifier (implementation-defined).
uint8 store_id # Store instance identifier. Fixed physical bay assignment: a bay keeps its store_id permanently, and when a store is expended the bay may be reloaded under the same store_id. IDs never renumber. 0 = not applicable / single store. This is the canonical definition of store_id across the dialect.
uint8 pin_state # Physical safety pull-pin state (read-only status).
uint8 arm_delay_setting # Selected arming-delay switch position (90/60/30 s). The chosen setting.
int16 arm_delay_remaining_s # [s] Live arming-delay countdown: seconds remaining after the in-flight-event signal until arming is permitted. -1 = not counting / not applicable. Distinct from arm_delay_setting (the switch position).
uint8 trigger_distance_mode # Trigger-distance standoff mode currently in effect (coarse selection, not a distance value).
uint8[16] track_uid # Track UID (UUID) of the engagement this store is armed for, when applicable. Ties the armed state to a specific authorized track so a monitoring station sees "armed against track X," not merely "an ESAD is hot." All-zero if not associated with a track.
