# Example: C-UAS interceptor with MAVLink-M

Scenario: a ground tracker finds a hostile multirotor, hands it to an interceptor
(**armed + gated**: handover loads the track only). C2/autonomy then sends `FIRES`
with a **predicted intercept point** to start the chase. Live kinematics stay on
`TARGET`. When the interceptor’s own camera/radar locks, it publishes an onboard
track (`TRACK_IDENTITY` + `TARGET`) — never by stuffing the target into
`PARTICIPANT_POSITION`.

## Policy (this example)


| Step        | Message                | Meaning                                                            |
| ----------- | ---------------------- | ------------------------------------------------------------------ |
| Assign      | `TARGET_HANDOVER`      | Load this track — **do not chase yet**                             |
| Engage      | `FIRES`                | Commit: fly to **estimated intercept** (lat/lon/alt + impact time) |
| Guide       | `TARGET` stream        | Keep updating prediction / pursuit after engage                    |
| Stop        | `ENGAGEMENT_DIRECTIVE` | Abort / check-fire / resume / retarget                             |
| Self on map | `PARTICIPANT_POSITION` | Friendly interceptor only                                          |


`FIRES` adapted for A2A: treat `lat/lon/alt` + `time_impact_usec` as the
**predicted meeting point**, tied to `track_uid`. Ignore artillery/fuze fields
unless you have a relevant payload. Replan continuously from `TARGET` (or
re-issue `FIRES` if C2 owns the prediction).

**Not used as primary engage:** `LOITER_MUNITION_CONTROL` (LM orbit/consent — poor fit for high-speed air intercept).

## Actors


| Actor               | Role                          | Publishes                                                                      |
| ------------------- | ----------------------------- | ------------------------------------------------------------------------------ |
| Ground tracker / C2 | Detect, track, assign, engage | `TRACK_IDENTITY`, `TARGET`, `TARGET_HANDOVER`, `FIRES`, `ENGAGEMENT_DIRECTIVE` |
| Interceptor drone   | Wait → midcourse → terminal   | `PARTICIPANT_POSITION` (self), ACK, onboard `TRACK_IDENTITY` + `TARGET`, BDA   |
| Operator / GCS      | COP, auth record, abort       | `TARGET_AUTHORIZATION` (record), may trigger `FIRES` / directive               |


## Map layers (important)


| Symbol on COP           | Message                                 | Meaning                                                |
| ----------------------- | --------------------------------------- | ------------------------------------------------------ |
| Blue / friendly         | `PARTICIPANT_POSITION`                  | Interceptor **itself** (“I am here”)                   |
| Red / hostile (ground)  | Ground `TRACK_IDENTITY` + `TARGET`      | External track for prediction / midcourse              |
| Red / hostile (onboard) | Interceptor `TRACK_IDENTITY` + `TARGET` | Seeker/radar detection                                 |
| Engage aim (logical)    | `FIRES` lat/lon/alt                     | Predicted intercept point at engage (may be refreshed) |


`PARTICIPANT_POSITION` never carries the target.

## Flow (overview)

```text
Detect          Track           Assign              Engage (FIRES)         Terminal              Assess
   |               |               |                      |                    |                     |
   | TRACK_IDENTITY|               |                      |                    |                     |
   | (ground) ---->|               |                      |                    |                     |
   |               | TARGET @10Hz  |                      |                    |                     |
   |               | (ground) ---->|--------------------->|------------------->|                     |
   |               |               | HANDOVER             |                    |                     |
   |               |               | (load track only)    |                    |                     |
   |               |               |--------------------->|                    |                     |
   |               |               | PARTICIPANT_POSITION |                    |                     |
   |               |               | (blue self)          |                    |                     |
   |               |               |                      | FIRES              |                     |
   |               |               |                      | predicted PIP + TOA|                     |
   |               |               |                      |------------------->|                     |
   |               |               |                      | chase + replan     |                     |
   |               |               |                      | from TARGET        |                     |
   |               |               |                      |                    | seeker: onboard     |
   |               |               |                      |                    | TRACK+TARGET        |
   |               |               |                      |                    |-------------------> | BDA
```

## Sequence diagram

```mermaid
sequenceDiagram
    autonumber
    participant T as Ground tracker / C2
    participant G as GCS / Operator
    participant I as Interceptor

    Note over T: Detect hostile multirotor

    T->>G: TRACK_IDENTITY (ground)<br/>track_uid_G, class=UAS_MULTIROTOR
    T->>G: TARGET (ground) @ ~10 Hz
    T->>I: TARGET (ground stream)

    G->>G: Operator confirms ID
    G->>T: TARGET_AUTHORIZATION (audit record only)

    T->>I: TARGET_HANDOVER (track_uid_G + kinematics)
    I->>T: MAVLINK_M_ACK (accepted)
    Note over I: TRACK_ASSIGNED — filter TARGET,<br/>do NOT chase yet (armed + gated)

    I->>G: PARTICIPANT_POSITION<br/>(friendly self only — not the target)

    T->>T: Predict intercept point (PIP)<br/>from TARGET + interceptor state
    T->>I: FIRES<br/>lat/lon/alt=PIP, time_impact, track_uid, sequence
    I->>T: MAVLINK_M_ACK (accepted)
    Note over I: MIDCOURSE — aim at PIP,<br/>keep replanning from TARGET + PX4 setpoints

    alt Seeker / onboard radar acquires target
        Note over I: Publish a real track — do NOT put target in PARTICIPANT_POSITION
        I->>G: TRACK_IDENTITY (onboard)<br/>track_uid_I, parent=track_uid_G
        I->>G: TARGET (onboard) @ higher rate
        I->>T: same onboard TRACK_IDENTITY + TARGET
        Note over T,I: Correlate track_uid_I ↔ track_uid_G
        Note over I: Prefer onboard TARGET for terminal;<br/>optional refreshed FIRES with new PIP
    else Abort before/during engage
        G->>I: ENGAGEMENT_DIRECTIVE (ABORT / CHECK_FIRE)
        I->>G: MAVLINK_M_ACK
        Note over I: Break off / RTB
    end

    I->>T: BATTLE_DAMAGE_ASSESSMENT<br/>(prefer ground track_uid_G for chain continuity)
```



## Plan: onboard detection → publish track

When the interceptor’s camera or radar detects the target:

1. **Keep** sending `PARTICIPANT_POSITION` for the interceptor body only.
2. **Create** an onboard `TRACK_IDENTITY`:
  - new `track_uid_I` (or reuse `track_uid_G` if ICD says interceptor inherits custody UID)
  - `origin_sysid` = interceptor
  - `origin_sensor` / `id_method` = EO or radar
  - optional: `parent_track_uid` = `track_uid_G`
3. **Stream** onboard `TARGET` to GCS/C2.
4. **Switch guidance** from ground `TARGET` to onboard `TARGET` (lower latency).
5. Optionally C2/companion **refreshes** `FIRES` with an updated PIP from the tighter track.

```text
Ground track:    track_uid_G   (assignment + BDA continuity)
Onboard track:   track_uid_I   with parent_track_uid = track_uid_G
COP display:     one hostile icon after association
Guidance:        ground TARGET until lock → onboard TARGET after lock
Engage gate:     FIRES (PIP + TOA + track_uid)
```

## Example payloads (simplified)

### 1. Ground track created — `TRACK_IDENTITY`

```text
track_uid            = 550e8400-e29b-41d4-a716-446655440000   # track_uid_G
origin_sysid         = 1          # ground tracker
target_class         = UAS_MULTIROTOR
target_force         = HOSTILE
environment          = AIR
id_confidence        = 0.82
first_detected_usec  = <t0>
```

### 2. Live ground track — `TARGET` (streamed)

```text
lat, lon, alt        = current estimated position
vx, vy, vz           = NED velocity
cov_*                = uncertainty
target_class         = UAS_MULTIROTOR
```

Used to predict PIP and to replan after `FIRES`.

### 3. Assign interceptor — `TARGET_HANDOVER`

```text
track_uid            = 550e8400-...   # track_uid_G
lat/lon/alt + vel    = snapshot at handover
valid_until_usec     = t0 + 30s
target_class         = UAS_MULTIROTOR
```

Meaning: “this track is yours” — **load only**, wait for `FIRES`.

### 4. Friendly self — `PARTICIPANT_POSITION`

FC-generated (estimator → MAVLink stream). Not published by the companion.

```text
time_usec            = UNIX epoch usec (RTC; stream skipped if clock unset)
lat, lon             = vehicle_global_position (degE7; require lat_lon_valid)
alt                  = MSL m (NaN if !alt_valid)
vx, vy, vz           = NED m/s from vehicle_local_position (NaN if invalid)
course               = course over ground deg = atan2(vy, vx)  # NOT body heading
callsign             = "speed0"
origin_sysid         = MAV_SYS_ID
external_track_number = "" (empty)
external_track_type  = NONE
stanag_identity      = FRIEND
ppli_type            = AIR
# NO target fields — this is not a detection message
```

### 5. Engage — `FIRES` (predicted intercept)

```text
time_usec            = now
time_impact_usec     = estimated intercept / TOA
lat, lon, alt        = predicted intercept point (PIP)
effector_id          = interceptor id
sequence             = 12                    # for ACK / abort / BDA correlation
cep_expected         = acceptable miss (m)   # optional uncertainty budget
track_uid            = 550e8400-...          # track_uid_G
store_id             = 0                     # vehicle itself
prf_code             = 0                     # unused unless SAL
requested_effect     = UNSPECIFIED / IMPACT  # as needed
munition_class       = UNKNOWN               # N/A for pure chase
fuze_mode            = UNKNOWN
hob_intent           = UNSPECIFIED
fuze_mofa_capable    = 0
```

**ICD note:** after ACK, companion chases using live `TARGET` (replan aim continuously).
`FIRES` coordinates are the **commit aim / initial PIP**, not a frozen-only guidance source
unless the endgame is short. C2 may re-send `FIRES` with the same `sequence` or a new one
when the PIP changes significantly.

### 6. Onboard lock — interceptor publishes track

`TRACK_IDENTITY` (from interceptor):

```text
track_uid            = 7c9e6679-7425-40de-944b-e07fc1f90ae7   # track_uid_I
parent_track_uid     = 550e8400-e29b-41d4-a716-446655440000   # track_uid_G
origin_sysid         = 42         # interceptor
origin_sensor        = EO / RADAR
target_class         = UAS_MULTIROTOR
id_confidence        = 0.91
```

`TARGET` (from interceptor, streamed):

```text
lat, lon, alt        = seeker-derived target position
vx, vy, vz           = seeker-derived velocity
cov_*                = usually tighter than ground track near terminal
```

### 7. Fly (companion)

1. On `HANDOVER`: store `track_uid_G`, filter `TARGET`, stay in wait/hold
2. On `FIRES`: enter midcourse — seed aim from PIP / `time_impact_usec`
3. Replan aim from live `TARGET` → PX4 offboard / external setpoints
4. On seeker lock: publish onboard track; prefer onboard `TARGET` for terminal
5. Keep `PARTICIPANT_POSITION` as self-report only

### 8. Engage or stop


| Intent                                 | Message                           |
| -------------------------------------- | --------------------------------- |
| Prosecute (commit chase)               | `FIRES` (PIP + TOA + `track_uid`) |
| Abort / check-fire / resume / retarget | `ENGAGEMENT_DIRECTIVE`            |
| Confirm receipt                        | `MAVLINK_M_ACK`                   |


### 9. Outcome — `BATTLE_DAMAGE_ASSESSMENT`

```text
track_uid            = 550e8400-...   # prefer ground UID for chain continuity
destruction_pct      = 100 (or 0 if miss)
reattack_recommended = 0 or 1
```

## What each message does in this scenario


| Message                    | Who sends         | What happens when it arrives                     |
| -------------------------- | ----------------- | ------------------------------------------------ |
| `TRACK_IDENTITY` (ground)  | Tracker           | Hostile appears on COP; names the object         |
| `TARGET` (ground)          | Tracker           | Kinematics for PIP + midcourse replan            |
| `TARGET_AUTHORIZATION`     | Operator/C2       | Audit trail only — not an auto-fire switch       |
| `TARGET_HANDOVER`          | C2                | Interceptor **loads** track; waits (gated)       |
| `PARTICIPANT_POSITION`     | Interceptor       | Blue force “I am here” only                      |
| `FIRES`                    | C2 / autonomy     | **Start chase** using predicted intercept point  |
| `TRACK_IDENTITY` (onboard) | Interceptor       | Seeker-originated track (parented to ground UID) |
| `TARGET` (onboard)         | Interceptor       | High-rate terminal track                         |
| `ENGAGEMENT_DIRECTIVE`     | Operator/C2       | Abort or retarget immediately                    |
| `BATTLE_DAMAGE_ASSESSMENT` | Interceptor or C2 | Close the loop on the engagement track           |


## Boundary reminder

```text
MAVLink-M  = track picture / assign / FIRES engage / abort / assess
PX4 MAVLink = how the interceptor actually flies (modes, setpoints, RTL)

PARTICIPANT_POSITION = friendly self
TRACK_IDENTITY+TARGET = any detected object (ground OR onboard seeker)
FIRES                 = engage gate + predicted intercept (A2A adaptation)
HANDOVER              = assign only (this ICD)
```

Private IDs (`53900–53999`) remain optional later for: seeker lock flag, TTG telemetry,
mission state (`SEARCH`/`TRACK_ASSIGNED`/`ENGAGE`/`RTB`) — not required for this `FIRES`-based engage path.

## PX4 FC path (SITL: `CONFIG_MAVLINK_DIALECT=military`)

MAVLink-M on the wire ↔ uORB on the FC ↔ ROS 2 via uXRCE-DDS. Naming:


| MAVLink-M message          | uORB topic                           | ROS 2 type                                |
| -------------------------- | ------------------------------------ | ----------------------------------------- |
| `TRACK_IDENTITY`           | `mavlink_m_track_identity`           | `px4_msgs/MavlinkMTrackIdentity`          |
| `TARGET`                   | `mavlink_m_target`                   | `px4_msgs/MavlinkMTarget`                 |
| `TARGET_HANDOVER`          | `mavlink_m_target_handover`          | `px4_msgs/MavlinkMTargetHandover`         |
| `FIRES`                    | `mavlink_m_fires`                    | `px4_msgs/MavlinkMFires`                  |
| `ENGAGEMENT_DIRECTIVE`     | `mavlink_m_engagement_directive`     | `px4_msgs/MavlinkMEngagementDirective`    |
| `MAVLINK_M_ACK`            | `mavlink_m_ack`                      | `px4_msgs/MavlinkMAck`                    |
| `BATTLE_DAMAGE_ASSESSMENT` | `mavlink_m_battle_damage_assessment` | `px4_msgs/MavlinkMBattleDamageAssessment` |


`PARTICIPANT_POSITION` is bidirectional blue-force / PPLI:

| Direction | Path |
| --------- | ---- |
| **TX own-ship** | FC estimator → MAVLink stream (callsign `speed0`, `FRIEND` / `AIR`) |
| **RX peers** | MAVLink → `mavlink_receiver` → uORB → `/fmu/out/mavlink_m_participant_position` → Jetson |

Own-ship TX field policy:


| Field              | Source / value                             |
| ------------------ | ------------------------------------------ |
| `time_usec`        | RTC UNIX usec; skip send if clock unset    |
| `lat` / `lon`      | `gpos` degE7; require `lat_lon_valid`      |
| `alt`              | `gpos.alt` MSL, or `NaN` if `!alt_valid`   |
| `vx` / `vy` / `vz` | `lpos` NED; `NaN` if velocity invalid      |
| `course`           | COG from `atan2(vy, vx)` (not yaw/heading) |
| `callsign`         | hardcoded `"speed0"`                       |
| `origin_sysid`     | `MAV_SYS_ID`                               |
| `external_track_*` | empty / `NONE` until a gateway assigns one |
| `stanag_identity`  | `FRIEND`                                   |
| `ppli_type`        | `AIR`                                      |

RX ignores messages whose `origin_sysid` equals local `MAV_SYS_ID`. Companion demuxes peers by `origin_sysid` / callsign (single uORB topic, last update per message). Give each aircraft a unique `MAV_SYS_ID`.

### Direction (wired on FC)

**Inbound** (MAVLink → `mavlink_receiver` → uORB → `/fmu/out/...`):

- `TRACK_IDENTITY`, `TARGET`, `TARGET_HANDOVER`, `FIRES`, `ENGAGEMENT_DIRECTIVE`
- `PARTICIPANT_POSITION` (peer PPLI only; own-ship filtered)
- `MAVLINK_M_ACK` (peer ACKs only; own `ack_sysid` filtered)
- `BATTLE_DAMAGE_ASSESSMENT` (peer / C2; own TX echo filtered by MAVLink sysid)

**Outbound** (companion `/fmu/in/...` → uORB → MAVLink stream):

- `TRACK_IDENTITY` (onboard seeker; stream sends only if `origin_sysid == MAV_SYS_ID`)
- `TARGET` (onboard seeker kinematics)
- `TARGET_HANDOVER` (custody transfer to another shooter)
- `MAVLINK_M_ACK` (stream sends only if `ack_sysid == MAV_SYS_ID`)
- `BATTLE_DAMAGE_ASSESSMENT`

**Outbound** (FC estimator → MAVLink stream):

- `PARTICIPANT_POSITION` (own-ship)

Seeker / ACK companion field policy:

| Message | Companion must set | Notes |
| ------- | ------------------ | ----- |
| `TRACK_IDENTITY` | `origin_sysid = MAV_SYS_ID` | Required or the FC stream will not TX |
| `MAVLINK_M_ACK` | `ack_sysid = MAV_SYS_ID` | Required or the FC stream will not TX |
| `TARGET` / `TARGET_HANDOVER` | (no origin field) | Enable those streams on the **C2 uplink only**, not a broadcast mesh — inbound messages share the same uORB topic and would otherwise be relayed |

Enable streams when needed, e.g.:

```text
mavlink stream -u 14540 -s PARTICIPANT_POSITION -r 5
mavlink stream -u 14540 -s TRACK_IDENTITY -r 10
mavlink stream -u 14540 -s TARGET -r 10
mavlink stream -u 14540 -s TARGET_HANDOVER -r 1
mavlink stream -u 14540 -s MAVLINK_M_ACK -r 10
mavlink stream -u 14540 -s BATTLE_DAMAGE_ASSESSMENT -r 1
```

ROS 2 companion topics:

| Direction | Topics |
| --------- | ------ |
| Publish (seeker / ACK / BDA / handoff) | `/fmu/in/mavlink_m_track_identity`, `/fmu/in/mavlink_m_target`, `/fmu/in/mavlink_m_target_handover`, `/fmu/in/mavlink_m_ack`, `/fmu/in/mavlink_m_battle_damage_assessment` |
| Subscribe (C2 / peers) | `/fmu/out/mavlink_m_*` for the same names plus `fires`, `engagement_directive`, `participant_position` |

All 26 MAVLink-M uORB defs exist as `msg/MavlinkM*.msg`; interceptor core is receiver/stream/DDS-wired as above.
