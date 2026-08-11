# Generic acknowledgment for MAVLink-M messages
#
# MAVLink-M message MAVLINK_M_ACK (id 53004).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 ack_msgid # MAVLink message ID being acknowledged (e.g. 53001 for TARGET_CUE).
uint32 ack_instance # Instance identifier of the acknowledged item (e.g. cue_id / target_set_id / sequence). 0 if not applicable.
uint8 origin_sysid # System ID of the originator of the acknowledged message.
uint8 ack_sysid # System ID of the system issuing this acknowledgment.
uint8 result # Result of receiving/processing the acknowledged message.
char[50] reason # Optional human-readable reason, primarily for REJECTED/FAILED (null-terminated).
