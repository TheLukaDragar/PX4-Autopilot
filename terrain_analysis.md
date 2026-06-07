# PX4 Terrain Estimation - Complete Analysis

## The Chicken-and-Egg Deadlock Problem

### Problem Summary
When you set `EKF2_HGT_REF = 2` (Range sensor as height reference) with optical flow enabled, the system enters a deadlock state that prevents optical flow from starting, leading to `local_position_invalid` failsafe and forced switch to Attitude mode.

---

## The Deadlock Logic Chain

### Initial State
- `EKF2_HGT_REF = 2` (Range sensor)
- `EKF2_RNG_CTRL = 2` (Range fusion enabled)
- `EKF2_OF_CTRL = 1` (Optical flow enabled)
- Optical flow is **NOT** currently running (`_control_status.flags.opt_flow = false`)

### Step-by-Step Breakdown

#### Step 1: Terrain Flag is Disabled
**Location:** `optical_flow_control.cpp:162, 192, 218`

```cpp
_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE);
```

**Result when `EKF2_HGT_REF = 2`:**
```
opt_flow_terrain = opt_flow && !(RANGE == RANGE)
                 = opt_flow && !true
                 = opt_flow && false
                 = false  // DISABLED
```

**Why?** The comment at line 161 explains:
```cpp
// If the height is relative to the ground, terrain height cannot be observed.
```

#### Step 2: Starting Conditions Check Fails
**Location:** `optical_flow_control.cpp:153-159`

```cpp
const bool starting_conditions_passing = continuing_conditions_passing
    && is_quality_good
    && is_magnitude_good
    && is_tilt_good
    && (_flow_counter > 10)
    && (isTerrainEstimateValid() || isHorizontalAidingActive())  // LINE 158 - THE PROBLEM
    && isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)2e6);
```

**Evaluation:**
- `isTerrainEstimateValid()` = **FALSE** (terrain disabled because opt_flow_terrain = false)
- `isHorizontalAidingActive()` = **FALSE** (optical flow not running, no GPS, no other sources)

**Result:**
```
starting_conditions_passing = ... && (false || false) && ...
                            = false  ❌
```

#### Step 3: Optical Flow Cannot Start
**Location:** `optical_flow_control.cpp:189-220`

```cpp
} else {  // Optical flow NOT running
    if (starting_conditions_passing) {  // FALSE - never enters this block!
        _control_status.flags.opt_flow_terrain = (_height_sensor_ref != HeightSensor::RANGE);

        if (isHorizontalAidingActive()) {
            // Start optical flow
        } else {
            if (isTerrainEstimateValid() || (_height_sensor_ref == HeightSensor::RANGE)) {
                // LINE 206-209: Would start optical flow here!
                ECL_INFO("starting optical flow, resetting");
                resetFlowFusion(flow_sample);
                _control_status.flags.opt_flow = true;
            }
        }
    }
    // Never reaches line 206 because starting_conditions_passing = false!
}
```

**The Paradox:**
- Line 206 has a condition `(_height_sensor_ref == HeightSensor::RANGE)` that would allow starting
- BUT it's inside the `if (starting_conditions_passing)` block
- Line 158 ensures `starting_conditions_passing = false`
- **Line 206 is unreachable!** ❌

#### Step 4: No Horizontal Aiding → Failsafe
**Location:** `ekf_helper.cpp:880-885`

```cpp
if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.ekf2_noaid_tout)) {
    // deadreckon time exceeded (default 5 seconds)
    if (!_horizontal_deadreckon_time_exceeded) {
        ECL_WARN("horizontal dead reckon time exceeded");
        _horizontal_deadreckon_time_exceeded = true;
    }
}
```

**Result:** After 5 seconds (`EKF2_NOAID_TOUT`):
- `_horizontal_deadreckon_time_exceeded = true`
- `isLocalHorizontalPositionValid()` returns `false`
- `local_position_invalid` failsafe triggers
- System switches to **Attitude mode**

---

## The Core Logic Bug

### The Design Flaw

**Line 158** requires:
```cpp
(isTerrainEstimateValid() || isHorizontalAidingActive())
```

**Line 206** tries to handle range-as-height-ref case:
```cpp
if (isTerrainEstimateValid() || (_height_sensor_ref == HeightSensor::RANGE)) {
    // Start optical flow
}
```

**The Bug:**
Line 206's special case for `_height_sensor_ref == HeightSensor::RANGE` is **unreachable** because:
1. It's inside `if (starting_conditions_passing)` block
2. Line 158 makes `starting_conditions_passing = false` when range is height ref
3. Code never reaches line 206

### What Should Happen

**Line 158 should be:**
```cpp
&& (isTerrainEstimateValid()
    || isHorizontalAidingActive()
    || (_height_sensor_ref == HeightSensor::RANGE))  // Add this condition!
```

**OR** the check at line 206 should be evaluated BEFORE checking `starting_conditions_passing`.

---

## Why Terrain Cannot Be Observed with Range-as-Height-Reference

### Physical/Mathematical Reason

When using range sensor as the height reference:

```
Coordinate System:
- Your altitude Z = Range sensor reading (e.g., 2 meters)
- Ground level = 0 (by definition, since range measures distance TO ground)
- Terrain height = ??? (UNDEFINED in this reference frame)
```

**The fundamental issue:**
- "Terrain height" means: height of ground relative to some absolute reference (like sea level)
- When range sensor IS your height reference, the ground is your zero point
- You can't estimate "terrain height" when your coordinate system is ALREADY relative to terrain
- It's like asking: "How high is the ground above the ground?" → Nonsensical!

### Optical Flow Dependency

Optical flow calculates velocity from visual motion:

```
Velocity = Angular_velocity_in_image × Height_above_ground × Camera_focal_length
```

**Two scenarios:**

1. **With Terrain Estimation (height ref = Baro/GPS):**
   - Absolute altitude from baro/GPS (e.g., 100m MSL)
   - Terrain height from range (e.g., 98m MSL)
   - Height above ground = 100 - 98 = 2m ✓
   - Optical flow can calculate velocity ✓

2. **Without Terrain (height ref = Range):**
   - Height = Range reading = 2m (relative to ground)
   - No terrain estimate exists (ground IS the reference)
   - Optical flow uses height = 2m directly ✓
   - **Should work, but code prevents it!**

---

## What Counts as Horizontal Aiding

**Location:** `estimator_interface.cpp:602-646`

### Horizontal Position Sources
```cpp
int getNumberOfActiveHorizontalPositionAidingSources() const {
    return int(_control_status.flags.gnss_pos)      // GPS position
         + int(_control_status.flags.ev_pos)        // External vision position
         + int(_control_status.flags.aux_gpos);     // Auxiliary GPS position
}
```

### Horizontal Velocity Sources
```cpp
int getNumberOfActiveHorizontalVelocityAidingSources() const {
    return int(_control_status.flags.gnss_vel)      // GPS velocity
         + int(_control_status.flags.opt_flow)      // OPTICAL FLOW
         + int(_control_status.flags.ev_vel)        // External vision velocity
         + int(_control_status.flags.wind_dead_reckoning);
}
```

### Combined Horizontal Aiding
```cpp
int getNumberOfActiveHorizontalAidingSources() const {
    return getNumberOfActiveHorizontalPositionAidingSources()
         + getNumberOfActiveHorizontalVelocityAidingSources();
}

bool isHorizontalAidingActive() const {
    return getNumberOfActiveHorizontalAidingSources() > 0;
}
```

**Key Point:** Range sensor is **NOT** included in any horizontal aiding sources!

---

## Local Position Validity Check

### How local_position_invalid is Set

**Location:** `estimatorCheck.cpp:733-735`

```cpp
failsafe_flags.local_position_invalid =
    !checkPosVelValidity(now, xy_valid, lpos.eph, lpos_eph_threshold, lpos.timestamp,
                         _last_lpos_fail_time_us, !failsafe_flags.local_position_invalid);
```

Where `xy_valid` comes from:

**Location:** `EKF2.cpp:1585`
```cpp
lpos.xy_valid = _ekf.isLocalHorizontalPositionValid();
```

**Location:** `ekf.h:232-235`
```cpp
bool isLocalHorizontalPositionValid() const {
    return !_horizontal_deadreckon_time_exceeded;
}
```

**Location:** `ekf_helper.cpp:880-885`
```cpp
if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.ekf2_noaid_tout)) {
    // deadreckon time exceeded
    if (!_horizontal_deadreckon_time_exceeded) {
        ECL_WARN("horizontal dead reckon time exceeded");
        _horizontal_deadreckon_time_exceeded = true;
    }
}
```

### The Timeline

```
T=0s:    Arm and takeoff
         - optical_flow = false (can't start due to deadlock)
         - isHorizontalAidingActive() = false
         - _time_last_horizontal_aiding = 0

T=0-5s:  No horizontal aiding updates
         - _horizontal_deadreckon_time_exceeded = false (still OK)
         - xy_valid = true (still OK)
         - local_position_invalid = false (still OK)

T=5s:    EKF2_NOAID_TOUT expires (default 5000000 µs = 5 seconds)
         - _horizontal_deadreckon_time_exceeded = true  ❌
         - xy_valid = false  ❌
         - local_position_invalid = true  ❌

T=5s+:   Failsafe triggers
         - Position control modes require valid local_position
         - Failsafe action = FallbackStab (Attitude mode)
         - System switches to Attitude mode
```

---

## Failsafe Trigger Mechanism

**Location:** `framework.cpp:699-716`

```cpp
bool FailsafeBase::modeCanRun(const failsafe_flags_s &status_flags, uint8_t mode) {
    uint32_t mode_mask = 1u << mode;
    return
        (!status_flags.angular_velocity_invalid || ((status_flags.mode_req_angular_velocity & mode_mask) == 0)) &&
        (!status_flags.attitude_invalid || ((status_flags.mode_req_attitude & mode_mask) == 0)) &&
        (!status_flags.local_position_invalid || ((status_flags.mode_req_local_position & mode_mask) == 0)) &&  // THIS!
        (!status_flags.local_position_invalid_relaxed || ((status_flags.mode_req_local_position_relaxed & mode_mask) == 0)) &&
        (!status_flags.global_position_invalid || ((status_flags.mode_req_global_position & mode_mask) == 0)) &&
        // ... more checks
}
```

**Mode Requirements:**
- **Position Control:** Requires `local_position` valid
- **Altitude Control:** Requires `local_position` valid
- **Stabilized/Attitude:** NO position requirement ✓

**Failsafe Action:**
```cpp
case Action::FallbackStab:
    if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_STAB)) {
        selected_action = Action::FallbackStab;  // Attitude mode
        break;
    }
```

---

## Terrain Concepts in PX4

### 1. Main Height Reference (EKF2_HGT_REF)

**What it is:**
- The primary sensor used for vertical position (Z-axis) in the main EKF2 state estimator
- Defines the reference frame for altitude

**Options:**
- `0` = Barometric pressure (altitude above mean sea level)
- `1` = GPS altitude (altitude above ellipsoid/sea level)
- `2` = Range sensor (altitude above ground)
- `3` = External vision altitude

**Coordinate Frame:**
```
EKF2_HGT_REF = 0 (Baro):
- Z=0 is at takeoff location's barometric altitude
- Positive Z is up
- Measures altitude above sea level (approximately)

EKF2_HGT_REF = 2 (Range):
- Z=0 is at ground level (where range sensor points)
- Positive Z is up
- Measures height above ground directly
```

### 2. Terrain Estimate (Separate from Height Reference)

**What it is:**
- A **separate single-state estimator** that estimates the height of the ground relative to the local NED frame origin
- Only used when height reference is NOT range sensor

**Location in code:** `terrain_control.cpp`, `terrain_estimator/`

**How it works:**
```
If EKF2_HGT_REF = Baro/GPS:
    Absolute altitude (Z) = 100m MSL (from baro/GPS)
    Range sensor reads = 2m (distance to ground)
    Terrain height = 100 - 2 = 98m MSL

    Terrain estimator maintains this 98m value
    Used by optical flow, terrain following, etc.
```

**When terrain is valid:**
```cpp
bool isTerrainEstimateValid() const {
    return (valid_rng_terrain || valid_opt_flow_terrain || valid_ev_terrain)
           && (now - _time_last_terrain_fuse) < timeout;
}
```

Sources that can provide terrain:
- Range sensor → terrain (when height ref ≠ range)
- Optical flow → terrain estimation
- External vision → terrain

### 3. Optical Flow Terrain Flag

**What it is:**
- `_control_status.flags.opt_flow_terrain`
- Indicates whether optical flow is being used to estimate terrain

**The Critical Rule:**
```cpp
opt_flow_terrain = opt_flow && !(height_sensor_ref == RANGE)
```

**Logic:**
- If using range as height ref → **terrain cannot be observed** → `opt_flow_terrain = false`
- If using baro/GPS as height ref → terrain can be estimated → `opt_flow_terrain = true` (if opt_flow active)

### 4. Range Sensor Terrain Flag

**What it is:**
- `_control_status.flags.rng_terrain`
- Indicates whether range sensor is being used to estimate terrain

**When it's active:**
```cpp
// From range_height_control.cpp
if (starting_conditions_passing) {
    if (_control_status.flags.opt_flow_terrain) {
        if (!aid_src.innovation_rejected) {
            _control_status.flags.rng_terrain = true;
            fuseHaglRng(aid_src, false, true);  // fuse to terrain only
        }
    } else {
        _control_status.flags.rng_terrain = true;
    }
}
```

### 5. Terrain Following Mode (MPC_ALT_MODE)

**What it is:**
- A **multicopter flight mode** that maintains constant height above ground
- Separate from EKF2 height reference selection

**How it works:**
```
MPC_ALT_MODE = 1 (Terrain Following):
    1. EKF2 provides absolute altitude (from baro/GPS)
    2. Terrain estimator provides ground height
    3. Controller maintains: altitude_setpoint = terrain_height + desired_AGL
    4. Adjusts altitude to follow terrain
```

**Parameters:**
- `MPC_ALT_MODE = 0` → Normal altitude control (above takeoff point)
- `MPC_ALT_MODE = 1` → Terrain following
- `MPC_ALT_MODE = 2` → Terrain hold

**Key difference from EKF2_HGT_REF:**
- `EKF2_HGT_REF` = What sensor provides height to the estimator
- `MPC_ALT_MODE` = How the controller uses that height (terrain-relative or not)

---

## Complete Terrain Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSOR INPUTS                             │
├─────────────────────────────────────────────────────────────────┤
│  Barometer  │  GPS Alt  │  Range Sensor  │  Optical Flow       │
└──────┬───────────┬─────────────┬─────────────────┬──────────────┘
       │           │             │                  │
       │           │             │                  │
       ▼           ▼             ▼                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                   EKF2 HEIGHT REFERENCE                          │
│                     (EKF2_HGT_REF)                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Selected based on EKF2_HGT_REF parameter:                      │
│  - Baro (0): Uses barometer for absolute altitude               │
│  - GPS (1): Uses GPS altitude                                   │
│  - Range (2): Uses range sensor → Height = distance to ground   │
│  - Vision (3): Uses external vision                             │
│                                                                  │
│  Output: _state.pos(2) = Vehicle altitude in NED frame         │
│                                                                  │
└──────┬───────────────────────────────────────────────────────────┘
       │
       ├─────── If HGT_REF = Range ────────┐
       │                                   │
       ▼                                   ▼
┌─────────────────────┐          ┌──────────────────────┐
│  TERRAIN ESTIMATOR  │          │   NO TERRAIN         │
│                     │          │                      │
│  Only active if:    │          │  Terrain undefined   │
│  HGT_REF ≠ Range    │          │  opt_flow_terrain=0  │
│                     │          │  rng_terrain=0       │
│  Estimates:         │          │                      │
│  - Ground height    │          │  Why?                │
│    relative to      │          │  "Height is already  │
│    NED origin       │          │   relative to        │
│                     │          │   ground"            │
│  Inputs:            │          │                      │
│  - Range sensor     │          └──────────────────────┘
│  - Optical flow     │
│  - Ext vision       │
│                     │
│  Output:            │
│  - _state.terrain   │
│                     │
└──────┬──────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    OPTICAL FLOW FUSION                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Starting conditions (line 158):                                │
│    ✓ Quality good                                               │
│    ✓ Tilt good                                                  │
│    ✓ (isTerrainEstimateValid() || isHorizontalAidingActive())  │
│                                                                  │
│  If HGT_REF = Range:                                            │
│    ✗ isTerrainEstimateValid() = FALSE                           │
│    ✗ isHorizontalAidingActive() = FALSE (flow not started yet)  │
│    → DEADLOCK! Cannot start!                                    │
│                                                                  │
│  If HGT_REF = Baro/GPS:                                         │
│    ✓ isTerrainEstimateValid() = TRUE (from range sensor)        │
│    → Can start! Provides horizontal velocity aiding             │
│                                                                  │
└──────┬───────────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│              HORIZONTAL AIDING STATUS                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  isHorizontalAidingActive() checks:                             │
│    - GPS position/velocity                                       │
│    - Optical flow (if running)                                   │
│    - External vision                                             │
│    - NOT range sensor!                                           │
│                                                                  │
│  If no horizontal aiding for > 5 seconds:                       │
│    → horizontal_deadreckon_time_exceeded = TRUE                 │
│    → xy_valid = FALSE                                            │
│    → local_position_invalid = TRUE                              │
│    → FAILSAFE → Attitude mode                                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Solutions and Workarounds

### Solution 1: Use Terrain Following Mode (RECOMMENDED)

```bash
# Keep baro/GPS as main height reference
EKF2_HGT_REF = 0          # Barometric pressure

# Enable range for terrain estimation
EKF2_RNG_CTRL = 1         # Conditional range fusion
EKF2_RNG_A_HMAX = 10.0    # Use range below 10m

# Enable optical flow
EKF2_OF_CTRL = 1

# Enable terrain following MODE
MPC_ALT_MODE = 1          # Terrain following
```

**Why this works:**
- ✓ Terrain estimator active (height ref ≠ range)
- ✓ Optical flow can start (terrain valid)
- ✓ Horizontal aiding active (optical flow running)
- ✓ Maintains height above ground (terrain following mode)

### Solution 2: Fix the Code (Proper fix for the bug)

**File:** `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp`

**Change line 158 from:**
```cpp
&& (isTerrainEstimateValid() || isHorizontalAidingActive())
```

**To:**
```cpp
&& (isTerrainEstimateValid() || isHorizontalAidingActive() || (_height_sensor_ref == HeightSensor::RANGE))
```

This allows optical flow to start when range is the height reference, which was the original intent (as evidenced by line 206).

### Solution 3: Bootstrap with GPS Then Switch

```bash
# Start with GPS
param set EKF2_HGT_REF 1     # GPS altitude

# Enable optical flow
param set EKF2_OF_CTRL 1

# After optical flow starts, switch to range (via script/mission)
param set EKF2_HGT_REF 2     # Range sensor
```

**Why this works:**
- Optical flow starts with terrain estimate (from GPS+range)
- Once running, `isHorizontalAidingActive() = true`
- Can maintain fusion even after switching to range height ref

### Solution 4: Use GPS for Horizontal Aiding

```bash
EKF2_HGT_REF = 2          # Range sensor height
EKF2_RNG_CTRL = 2         # Range fusion enabled
EKF2_GPS_CTRL = 3         # GPS for horizontal position only (not height)
```

**Why this works:**
- GPS provides horizontal position aiding
- `isHorizontalAidingActive() = true`
- No deadlock

---

## Key Takeaways

1. **Range sensor only provides VERTICAL information** - never horizontal aiding

2. **Terrain estimation is separate from height reference** - terrain estimator only works when height ref ≠ range

3. **Optical flow has a chicken-and-egg problem with range-as-height-ref:**
   - Needs terrain OR horizontal aiding to start
   - Terrain disabled when range = height ref
   - Optical flow IS the horizontal aiding (can't start if it's not running)

4. **The code has a logic bug:** Line 206 tries to handle range-as-height-ref case but is unreachable due to line 158's precondition check

5. **Use `MPC_ALT_MODE` for terrain following,** not `EKF2_HGT_REF = 2` (range) - this is the proper/supported way

6. **Local position invalid failsafe** triggers when no horizontal aiding for >5 seconds (`EKF2_NOAID_TOUT`)

---

## Diagnostic Commands

```bash
# Check current configuration
param show EKF2_HGT_REF
param show EKF2_RNG_CTRL
param show EKF2_OF_CTRL
param show MPC_ALT_MODE

# Check if optical flow is fusing (in flight)
listener estimator_status_flags
# Look for: cs_opt_flow flag

# Check horizontal aiding sources
listener estimator_status
# Look for: control_mode_flags

# Check terrain estimate
listener estimator_status
# Look for: terrain_altitude, terrain_valid

# Monitor failsafe state
listener failsafe_flags
# Look for: local_position_invalid
```
