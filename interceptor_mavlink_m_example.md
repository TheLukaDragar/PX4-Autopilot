# C-UAS interceptor — MAVLink-M ICD

## Working field setup (13 Aug 2026)

Two MicoAir H743-v2. **Do not flash `demo_enemy` onto the interceptor.**


| Aircraft    | Git                       | `MAV_SYS_ID` | Role                                 |
| ----------- | ------------------------- | ------------ | ------------------------------------ |
| Interceptor | `main` + military dialect | **1**        | Hear HOSTILE, forward COP to QGC     |
| Enemy       | `demo_enemy`              | **2**        | Advertise self as HOSTILE (`leseni`) |


Two **separate** [LR24-F](https://micoair.com/radio_telemetry_lr24f/) pairs (different **ADDR**). Do not share one pair for both jobs.

| Pair | Who | UART | Job |
| ---- | --- | ---- | --- |
| **QGC** | Interceptor `MAV_0` ↔ QGC only | interceptor `ttyS0` @57600 | Fly the interceptor. Also **displays** forwarded enemy `TRACK_IDENTITY` **53000** / `TARGET` **53010**. Enemy radio is **not** on this pair. |
| **C2** | Enemy `MAV_1` ↔ interceptor `MAV_2` | enemy `ttyS3` (TELEM 2) @57600, interceptor `ttyS4` @57600 | COP air: 53000 / 53010 / 53002 / ACK |

C2 pair: **MODE=DUPLEX**, **RATE=HIGH** (8 KB/s), UART **57600**, same ADDR on that pair only. Half-duplex — keep the interceptor C2 instance lean.

```mermaid
flowchart LR
  QGC["QGC<br/>fly interceptor + COP view<br/>53000 TRACK_IDENTITY<br/>53010 TARGET  sysid 2"]

  subgraph qgcPair["Antenna pair 1 — interceptor ↔ QGC only"]
    RQ["LR24-F ADDR A  57600"]
  end

  subgraph icept["Interceptor FC  sysid 1  — main"]
    I0["MAV_0 Normal  ttyS0 57600  FORWARD=1"]
    I1["MAV_1 Onboard  ttyS3 115200"]
    I2["MAV_2 Custom  ttyS4 57600  FORWARD=0  HB only"]
  end

  subgraph c2Pair["Antenna pair 2 — C2 COP"]
    RC["LR24-F ADDR B  HIGH  57600"]
  end

  subgraph enemy["Enemy FC  sysid 2  — demo_enemy"]
    E0["MAV_0 Normal  ttyS0"]
    E1["MAV_1 Custom  ttyS3  FORWARD=0<br/>53000 + 53010 @ 5 Hz<br/>mavlink cop one-shots"]
    EU["USB Onboard  ttyACM0  FORWARD=1<br/>QGC here only — not C2"]
  end

  QGC --- RQ --- I0
  E1 -->|"53000 + 53010"| RC --> I2
  I2 -->|"forward COP"| I0
```

QGC never talks to the enemy radio. It sees TARGET because the interceptor **forwards** 53000/53010 from `MAV_2` onto `MAV_0` over pair 1. Stock QGC will not *name* military messages unless it has the dialect; MAVLink Inspector still shows msgid **53000** / **53010** from sysid **2**.



### Enemy params (`demo_enemy`)

```text
MAV_SYS_ID      2
MAV_0_MODE      0         Normal — local telem, not interceptor QGC
MAV_1_MODE      1         Custom — C2 on ttyS3
MAV_1_FORWARD   0         required — see "QGC on enemy USB" below
MAV_1_CONFIG    TELEM 2   /dev/ttyS3 @57600
```

Custom: TRACK_IDENTITY 5 Hz, TARGET 5 Hz, SYS_STATUS 0.5 Hz, HEARTBEAT 1 Hz. EVENT and STATUSTEXT suppressed. PPLI stubbed. `mavlink cop` queues one-shots for the mavlink thread (NSH must not `write()` the UART fd — EBADF).

**QGC on enemy USB:** USB always has `FORWARD` on. If `MAV_1_FORWARD=1`, QGC sees interceptor as vehicle 1 and `SET_MESSAGE_INTERVAL` opens a full GCS session on the LoRa hop. Interceptor C2 then shows `sysid 254`; enemy Custom UART `FIONSPACE` stays 0; HANDOVER never goes out. After `MAV_1_FORWARD=0` + reboot: C2 RX is interceptor HB only (~87 B/s), USB 20 kB/s stays on `ttyACM0`.

### Interceptor params (`main`) — no rebuild

```text
MAV_SYS_ID      1
MAV_0_MODE      0         Normal — QGC antenna ttyS0
MAV_0_FORWARD   1         QGC pair only
MAV_2_MODE      1         Custom — C2 antenna ttyS4
MAV_2_FORWARD   0         never forward QGC onto C2
```

On **main**, Custom streams nothing except HEARTBEAT. That keeps the C2 antenna at ~50 B/s TX so the enemy can talk. `MAV_2_FORWARD=0` stops QGC traffic from being blasted **onto** C2. COP still hops **to** QGC: C2 RX → `forward_message` → `MAV_0` (`FORWARD=1`). Do not set `MAV_2_FORWARD=1` or the QGC firehose returns to the C2 radio.

### Wire message IDs (`mavlink status` → `msgid`)

Military dialect. Common IDs stay in the low range; COP is 53000+.

| msgid | MAVLink name | On-wire | This demo |
| ----- | ------------ | ------- | --------- |
| 0 | HEARTBEAT | 21 B | both, 1 Hz |
| 1 | SYS_STATUS | 55 B | enemy Custom 0.5 Hz |
| 33 | GLOBAL_POSITION_INT | 40 B | not on Custom |
| 42 | MISSION_CURRENT | — | PX4 leak, ignore |
| 410 | EVENT | — | **off** on enemy Custom |
| 411 | CURRENT_EVENT_SEQUENCE | — | **off** on enemy Custom |
| **53000** | **TRACK_IDENTITY** | **153 B** | enemy → interceptor, 5 Hz |
| 53001 | TARGET_CUE | — | unused here |
| 53002 | TARGET_HANDOVER | ~219 B | `mavlink cop handover` (proven RX on Jetson) |
| 53003 | PARTICIPANT_POSITION | 122 B | interceptor only (blue PPLI). Enemy stubbed. |
| 53004 | MAVLINK_M_ACK | — | ACK (event) |
| **53010** | **TARGET** | **257 B** | enemy → interceptor, 5 Hz |
| 53011 | TARGET_SET_COORD | — | unused here |
| 53012 | TARGET_BOX_COORD | — | unused here |
| 53013 | TARGET_AUTHORIZATION | — | audit |
| 53020 | FIRES | — | start chase |
| 53021 | SPLASH_CORRECTION | — | unused here |
| 53022 | BATTLE_DAMAGE_ASSESSMENT | — | close engagement |
| 53023 | ENGAGEMENT_DIRECTIVE | — | `abort`/`checkfire`/`resume` (0/1/2) |

ROS does not use these numbers. Same payloads:

| msgid | uORB | ROS 2 |
| ----- | ---- | ----- |
| 53000 | `mavlink_m_track_identity` | `/px4_0/fmu/out/mavlink_m_track_identity` |
| 53010 | `mavlink_m_target` | `/px4_0/fmu/out/mavlink_m_target` |
| 53003 | `mavlink_m_participant_position` | `/px4_0/fmu/out/mavlink_m_participant_position` (interceptor self) |

### What broke it / what fixed it


| Failure | Fix |
| ------- | --- |
| Enemy Normal firehose on C2 | `MAV_1_MODE=1` Custom |
| Interceptor `#2` Normal + FORWARD, ~2 kB/s TX | `MAV_2_MODE=1`, `MAV_2_FORWARD=0` |
| EVENT 410 at ~61 Hz | Custom skips events (enemy build) |
| Full vehicle 1 on enemy USB / C2 `sysid 254` | `MAV_1_FORWARD=0` + reboot |
| `send_start` drops 219 B (`FIONSPACE=0` while TX is 1.7 kB/s) | queue one-shot; mavlink thread `write()`s |
| NSH `write(uart_fd)` errno 9 | NuttX fds are per-task — do not write from `mavlink cop` |
| Companion `valid_until` expired | send `valid_until_usec=0` (ROS UNIX µs ≠ FC hrt) |
| Companion `lat lon INT32_MAX` | expected without GPS; NACK `Failed`, not a radio miss |


### Check

```text
# interceptor nsh
listener mavlink_m_track_identity    # uid[15]=2, HOSTILE/FOE, origin_sysid=2
listener mavlink_m_target            # name=leseni, target_id=2
mavlink status                       # inst #2: msgid 53000 and 53010 ~5 Hz

# Jetson
ros2 topic hz /px4_0/fmu/out/mavlink_m_track_identity
ros2 topic hz /px4_0/fmu/out/mavlink_m_target
```

ROS `hz` can be ~2 Hz with holes (uXRCE + half-duplex). Trust interceptor `mavlink status` msgid rates for the air link. Unknown GPS: TARGET lat/lon=`INT32_MAX`, alt/vel=NaN.

### Full workflow test (enemy nsh)

`demo_enemy` only. One-shots are queued onto Custom; TRACK/TARGET pause while waiting for ACK.

Companion (`speedo_c2_example` on speed0-1): `own_sysid=1`, **`c2_sysid=200`**. ACK `origin_sysid` is 200, not 2. It NACKs HANDOVER if lat/lon are `INT32_MAX` or alt is NaN; `Accepted` then **arms the interceptor**. Enable interceptor C2 ACK TX (no reflash):

```text
mavlink stream -d /dev/ttyS4 -s MAVLINK_M_ACK -r 50
```

```text
# enemy (after MAV_1_FORWARD=0)
mavlink cop handover          # 53002 — proven: Jetson onHandover(), then geo NACK
mavlink cop fires             # 53020
mavlink cop checkfire         # 53023 CHECK_FIRE(1) — companion NACKs if no intercept
mavlink cop resume            # 53023 RESUME(2) — companion NACKs if not in check-fire
mavlink cop abort             # 53023 ABORT(0)
mavlink cop workflow          # handover ACK, then fires ACK

# interceptor
listener mavlink_m_target_handover
listener mavlink_m_fires
listener mavlink_m_engagement_directive

# Jetson log
#   TARGET_HANDOVER (target_set_id=0 valid_until=0 …)
#   rejected — lat lon INT32_MAX   ← bench, no GPS
```

Same `track_uid[15]=MAV_SYS_ID` (2) and `sequence` as FIRES so abort/resume match. Kinematics from estimator, or INT32_MAX/NaN if no GPS. Do not put a dummy PIP unless props are off — companion will arm.

`ENGAGEMENT_DIRECTIVE.directive` must match `speedo_c2_example` (`executor.hpp`). The old cop fallback sent ABORT=1, which is CHECK_FIRE.

| `mavlink cop` | `directive` | Companion |
| ------------- | ----------- | --------- |
| `abort` | **0** ABORT | always `Accepted` (`ROS2: ABORT`) |
| `checkfire` | **1** CHECK_FIRE | `Rejected` / `no active interception` unless FIRES/Kill |
| `resume` | **2** RESUME | `Rejected` / `not in check-fire` unless after check-fire |
| *(none yet)* | **3** RETARGET | needs valid lat/lon/alt |

Bench with no chase: `abort` ACKs; `resume` / `checkfire` NACK for the reason above — that still means 53023 landed.

---

Ground tracker finds a hostile → `TARGET_HANDOVER` loads the track (armed, gated) →
`FIRES` starts the chase with a predicted intercept point (PIP) → live kinematics on
`TARGET` → onboard seeker publishes its own `TRACK_IDENTITY` + `TARGET`.


| Role               | Message                                     |
| ------------------ | ------------------------------------------- |
| Hostile / track    | `TRACK_IDENTITY` + `TARGET`                 |
| Friendly self      | `PARTICIPANT_POSITION` only                 |
| Assign (no chase)  | `TARGET_HANDOVER`                           |
| Engage             | `FIRES` (PIP + TOA + `track_uid`)           |
| Guide after engage | `TARGET` stream                             |
| Abort / retarget   | `ENGAGEMENT_DIRECTIVE`                      |
| ACK / BDA          | `MAVLINK_M_ACK`, `BATTLE_DAMAGE_ASSESSMENT` |


## Actors


| Actor                           | Publishes                                                                      |
| ------------------------------- | ------------------------------------------------------------------------------ |
| Ground tracker / C2             | `TRACK_IDENTITY`, `TARGET`, `TARGET_HANDOVER`, `FIRES`, `ENGAGEMENT_DIRECTIVE` |
| Enemy surrogate (this FC build) | `TRACK_IDENTITY` + `TARGET` from estimator (no PPLI)                           |
| Interceptor                     | `PARTICIPANT_POSITION` (self), ACK, onboard `TRACK_IDENTITY` + `TARGET`, BDA   |
| Operator / GCS                  | `TARGET_AUTHORIZATION` (audit), may trigger `FIRES` / directive                |


## Sequence

```mermaid
sequenceDiagram
    autonumber
    participant T as Ground tracker / C2
    participant G as GCS / Operator
    participant I as Interceptor

    T->>G: TRACK_IDENTITY + TARGET (ground)
    T->>I: TARGET (ground stream)
    G->>T: TARGET_AUTHORIZATION (audit)

    T->>I: TARGET_HANDOVER
    I->>T: MAVLINK_M_ACK
    Note over I: Load track — do not chase yet

    I->>G: PARTICIPANT_POSITION (self)

    T->>I: FIRES (PIP + TOA + track_uid)
    I->>T: MAVLINK_M_ACK
    Note over I: Midcourse — replan from TARGET

    alt Seeker lock
        I->>G: TRACK_IDENTITY + TARGET (onboard)
        I->>T: TRACK_IDENTITY + TARGET (onboard)
        Note over I: Prefer onboard TARGET for terminal
    else Abort
        G->>I: ENGAGEMENT_DIRECTIVE
        I->>G: MAVLINK_M_ACK
    end

    I->>T: BATTLE_DAMAGE_ASSESSMENT
```



## Companion fly logic

1. `HANDOVER` → store `track_uid_G`, filter `TARGET`, hold
2. `FIRES` → midcourse from PIP / `time_impact_usec`
3. Replan from live `TARGET` → PX4 setpoints
4. Seeker lock → publish onboard `TRACK_IDENTITY` + `TARGET`; prefer onboard for terminal
5. Keep `PARTICIPANT_POSITION` as own-ship only

```text
Ground track:   track_uid_G
Onboard track:  track_uid_I  (parent_track_uid = track_uid_G)
Guidance:       ground TARGET until lock → onboard TARGET after
```

## Example fields

**Ground `TRACK_IDENTITY`**

```text
track_uid / origin_sysid / target_class=UAS_MULTIROTOR / target_force=HOSTILE
```

**Ground `TARGET`** — lat/lon/alt, NED vel, cov (PIP + midcourse replan)

`**TARGET_HANDOVER**` — `track_uid_G` + kinematics snapshot; load only, wait for `FIRES`

**Enemy surrogate FC (this build, hardcoded)** — no companion needed:

```text
PPLI:           disabled
TRACK_IDENTITY: track_uid[15]=MAV_SYS_ID, HOSTILE, FOE, UAS_MULTIROTOR, AIR
TARGET:         own lat/lon/alt + NED vel, target_id=MAV_SYS_ID, name=leseni
                lat/lon=INT32_MAX if unknown; alt/vel/cov/CEP=NaN if unknown
```

Do not flash this tree to the interceptor without reverting those streams.

`**PARTICIPANT_POSITION**` (interceptor / blue FC only — disabled on this enemy build)

```text
lat/lon/alt, vx/vy/vz, course=COG, callsign=speed0, origin_sysid=MAV_SYS_ID
stanag_identity=FRIEND, ppli_type=AIR
lat/lon = INT32_MAX if unknown (never fake 0,0); alt/vel/course = NaN if unknown
```

`**FIRES**`

```text
lat/lon/alt = PIP, time_impact_usec = TOA, track_uid, sequence, effector_id
```

After ACK, chase from live `TARGET`; C2 may re-send `FIRES` when PIP changes.

**Onboard lock**

```text
TRACK_IDENTITY: track_uid_I, parent=track_uid_G, origin_sysid=interceptor
TARGET:         seeker lat/lon/alt + vel
```

`**BATTLE_DAMAGE_ASSESSMENT**` — prefer `track_uid_G` for chain continuity

## Message cheat sheet


| Message                               | Effect                     |
| ------------------------------------- | -------------------------- |
| `TRACK_IDENTITY` / `TARGET` (ground)  | Hostile on COP; kinematics |
| `TARGET_AUTHORIZATION`                | Audit only                 |
| `TARGET_HANDOVER`                     | Load track; wait           |
| `PARTICIPANT_POSITION`                | Blue self                  |
| `FIRES`                               | Start chase (PIP)          |
| `TRACK_IDENTITY` / `TARGET` (onboard) | Seeker track               |
| `ENGAGEMENT_DIRECTIVE`                | Abort(0) / check-fire(1) / resume(2) / retarget(3) |
| `BATTLE_DAMAGE_ASSESSMENT`            | Close engagement           |


`LOITER_MUNITION_CONTROL` is not used for this high-speed intercept path.

## PX4 wiring (`CONFIG_MAVLINK_DIALECT=military`)


| msgid | MAVLink-M | uORB | ROS 2 |
| ----- | --------- | ---- | ----- |
| 53000 | `TRACK_IDENTITY` | `mavlink_m_track_identity` | `MavlinkMTrackIdentity` |
| 53010 | `TARGET` | `mavlink_m_target` | `MavlinkMTarget` |
| 53002 | `TARGET_HANDOVER` | `mavlink_m_target_handover` | `MavlinkMTargetHandover` |
| 53020 | `FIRES` | `mavlink_m_fires` | `MavlinkMFires` |
| 53023 | `ENGAGEMENT_DIRECTIVE` | `mavlink_m_engagement_directive` | `MavlinkMEngagementDirective` |
| 53004 | `MAVLINK_M_ACK` | `mavlink_m_ack` | `MavlinkMAck` |
| 53022 | `BATTLE_DAMAGE_ASSESSMENT` | `mavlink_m_battle_damage_assessment` | `MavlinkMBattleDamageAssessment` |
| 53003 | `PARTICIPANT_POSITION` | `mavlink_m_participant_position` | `MavlinkMParticipantPosition` |


### Paths


| Direction                  | Messages                                                                      |
| -------------------------- | ----------------------------------------------------------------------------- |
| In `/fmu/out/`             | peer `TRACK`/`TARGET`/`HANDOVER`/`FIRES`/`DIRECTIVE`, peer PPLI, peer ACK/BDA |
| Out `/fmu/in/` → stream    | seeker `TRACK`, `TARGET`/`HANDOVER` via `*_send`, ACK, BDA (interceptor)      |
| Out (FC, this enemy build) | own-ship `TRACK_IDENTITY` + `TARGET` (PPLI off)                               |
| Out (FC, interceptor)      | own-ship `PARTICIPANT_POSITION`                                               |


Companion must set `origin_sysid` / `ack_sysid` = `vehicle_status.system_id` (`MAV_SYS_ID`).

**TARGET / TARGET_HANDOVER in vs out:** wire payloads have no `origin_sysid`, so peer RX and companion TX use separate uORB topics:


| ROS                                      | uORB                             | Role                        |
| ---------------------------------------- | -------------------------------- | --------------------------- |
| `/fmu/out/mavlink_m_target`              | `mavlink_m_target`               | Network RX only             |
| `/fmu/in/mavlink_m_target_send`          | `mavlink_m_target_send`          | Companion → TARGET stream   |
| `/fmu/out/mavlink_m_target_handover`     | `mavlink_m_target_handover`      | Network RX only             |
| `/fmu/in/mavlink_m_target_handover_send` | `mavlink_m_target_handover_send` | Companion → HANDOVER stream |


`TRACK_IDENTITY` can share one uORB: the stream already TX-filters on payload `origin_sysid == MAV_SYS_ID`.

Default rates (`MAVLINK_MODE_NORMAL` / `ONBOARD`, military dialect only):

```text
PARTICIPANT_POSITION       10 Hz
TRACK_IDENTITY             20 Hz
TARGET                     20 Hz
TARGET_HANDOVER            unlimited (on event)
MAVLINK_M_ACK              unlimited (on event)
BATTLE_DAMAGE_ASSESSMENT   unlimited (on event)
```

Override later with `mavlink stream` or `MAV_CMD_SET_MESSAGE_INTERVAL`.

| Publish | `/fmu/in/mavlink_m_{track_identity,target,target_handover,ack,battle_damage_assessment}` |
| Subscribe | `/fmu/out/mavlink_m_*` (+ `fires`, `engagement_directive`, `participant_position`) |
