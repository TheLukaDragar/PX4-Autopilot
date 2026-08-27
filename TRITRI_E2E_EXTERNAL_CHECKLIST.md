# TRITRI end-to-end — external flag-day checklist

PX4 (`TheLukaDragar/tritri-e2e` / merge to `main`) is the source of truth for wire + uORB.
Land these in lockstep with the FC flash.

## px4_msgs

- [ ] Regenerate / add `MavlinkMTritriTrack` and `MavlinkMTritriTarget` from PX4 `msg/`
- [ ] Point Jetson workspace at the new msgs
- [ ] Stop publishing/subscribing live COP on `MavlinkMTrackIdentity` / `MavlinkMTarget`

## seeker / speedo_c2

- [ ] Publish `/fmu/in/mavlink_m_tritri_track` (own `origin_sysid`)
- [ ] Publish `/fmu/in/mavlink_m_tritri_target_send` (include `track_uid`)
- [ ] Drop cut fields (package/CEP/id_basis/parent UID/…)

## QGC (custom)

- [ ] Ship `tritri` MAVLink dialect (53900/53901 names)
- [ ] COP UI / Inspector bind to msgid **53900** / **53901**
- [ ] Stop expecting live COP as 53000/53010 (expanded) — forward is as-is now
- [ ] Keep HANDOVER/FIRES/ACK/PPLI on existing military IDs

## Verify

- [ ] Interceptor `#2`: RX 53900@1 / 53901@5 from sysid 2
- [ ] `listener mavlink_m_tritri_*` fresh
- [ ] QGC shows 539xx from enemy; blue 53003 from interceptor on dummy C2 USB
- [ ] `ros2 topic hz` on new tritri out topics
