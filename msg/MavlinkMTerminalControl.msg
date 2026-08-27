# Terminal-attack-control handshake (JP 3-09
#
# MAVLink-M message TERMINAL_CONTROL (id 53061).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint16 sequence # Engagement sequence this handshake applies to.
uint8 tac_type # Type of terminal attack control (1/2/3).
uint8 clearance # Clearance / abort verb (human authority).
uint8 aircraft_call # Attacking-aircraft status call.
uint8 abort_authority_present # 1 = a valid abort authority is established for this engagement; 0 = not.
