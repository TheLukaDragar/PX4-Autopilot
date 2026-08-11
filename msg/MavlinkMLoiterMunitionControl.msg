# Loitering-munition / one-way-attack UAS control and consent handshake (JFIRE 2023)
#
# MAVLink-M message LOITER_MUNITION_CONTROL (id 53036).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint8[16] track_uid # 128-bit track UID of the engaged object (see TRACK_IDENTITY).
uint8 consent_state # Man-in-the-loop consent-to-strike state (recorded human decision, not a trigger).
uint8 terminate # Abort / wave-off / self-destruct action.
uint8 tst_status # Time-sensitive-target prosecution status.
uint8 pid_status # Positive-identification status gate (from TRACK_IDENTITY set).
uint8 cde_level # Collateral-damage-estimate level gate (number only).
int32 loiter_lat # [degE7] Loiter/orbit station latitude, WGS84. INT32_MAX = not provided (0 is a valid coordinate).
int32 loiter_lon # [degE7] Loiter/orbit station longitude, WGS84. INT32_MAX = not provided (0 is a valid coordinate).
uint8 dmpi_kind # How the desired point of impact is referenced (no mensurated coordinate).
