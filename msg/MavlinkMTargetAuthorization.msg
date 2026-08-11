# Accountability attestation for a track
#
# MAVLink-M message TARGET_AUTHORIZATION (id 53013).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp this attestation was recorded (UNIX epoch, microseconds UTC).
uint8[16] track_uid # Track UID this authorization record applies to.
uint64 decision_usec # [us] Time the authority made the decision (UNIX epoch, microseconds UTC).
uint64 valid_until_usec # [us] Expiry of this attestation (UNIX epoch, microseconds UTC). 0 = unspecified.
uint32 auth_nonce # Freshness nonce bound into the authorization token by the C2 node. A verifier caches seen nonces within the validity window to reject replays. Anti-replay input only; not itself a secret.
float32 collateral_estimate # Collateral-damage estimate context [0.0-1.0 normalized], if assessed. NaN if not.
uint8[16] authority_id # Opaque identifier of the authorizing authority/operator (for audit).
uint8[8] authorization_token # Opaque authorization token (carried for record; not a release key).
uint8 pid_status # PID status asserted by the authority.
uint8 roe_state # ROE/clearance state asserted (attested, not commanded).
uint8 nostrike_status # No-strike / restricted-target context.
char[50] authority_role # Human-readable role of the authority (e.g. designation), null-terminated.
uint8 cde_level # Collateral-damage-estimation LEVEL number only (CJCSI 3160.01). Never the underlying computation or population data.
