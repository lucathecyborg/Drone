# Drone PID Tuning Guide

## Before You Fly

### Equipment Needed
- Computer with serial monitor (115200 baud)
- Propellers installed for flight
- Battery
- Safe open space (preferably outdoors)
- Helper (strongly recommended)

### Pre-Flight Checks
1. **Motor Direction Verification** (CRITICAL - props OFF)
   ```
   From above (bird's eye view):
   - topL should spin CLOCKWISE
   - topR should spin COUNTER-CLOCKWISE  
   - bottomL should spin COUNTER-CLOCKWISE
   - bottomR should spin CLOCKWISE
   
   If wrong: Swap two ESC signal wires on that motor
   ```

2. **Joystick Verification** (in serial monitor)
   ```
   When joysticks centered: Should see ~512 for all values
   When moving left stick right: joystickL.x should go 512 → 1023
   When moving left stick forward: joystickL.y should go 512 → 0
   When moving right stick right: joystickR.x should go 512 → 1023
   ```

3. **Throttle Verification**
   ```
   When pot1 at minimum: base_motor_speed should be ~40
   When pot1 at maximum: base_motor_speed should be ~180
   ```

---

## First Flight Protocol

### Session 1: Low Throttle Test (base_motor_speed = 40-80)

**Goal:** Test basic stability and control authority

**Setup:**
1. Hold drone firmly in hand
2. Increase throttle slowly to base_speed = 50
3. Apply gentle pitch/roll/yaw inputs

**Expected Behavior:**
- **Pitch response:** Nose should tilt correctly
- **Roll response:** Wings should bank smoothly
- **Yaw response:** Drone should rotate around vertical axis
- **Motors:** Should not hit speed limits (max 180)

**Acceptable Behavior:**
- Small oscillations (1-2Hz) that damp out
- 1-2 second settling time
- Smooth, predictable response

**ABORT if:**
- Uncontrolled oscillations (>2Hz, growing)
- Inverted response (right stick left = rolls right)
- Motors maxing out frequently (see "180" in motor output)
- Yawing wrong direction (opposite of stick)

---

### Session 2: Mid-Throttle Test (base_motor_speed = 100-150)

**Goal:** Test stability at sustained flight power

**Procedure:**
1. Same as Session 1, but at higher throttle
2. Hold steady for 10+ seconds
3. Watch for oscillation magnitude (should stay same or decrease)

**Key Observations:**
```
Base Speed | Roll Response | Pitch Response | Motor Headroom
---------- | ------------- | -------------- | ---------------
100        | Quick/stable  | Quick/stable   | ±40 available
140        | Same/faster   | Same/faster    | ±20 available
180        | Sluggish      | Sluggish       | 0 available (saturated)
```

**If oscillating more at higher throttle:**
- Issue: Integral term too high
- Fix: Reduce ki (0.02 → 0.01 for roll/pitch)

---

### Session 3: Free Flight (Outdoor, Open Area)

**Minimum Confidence Level:** You've completed Sessions 1-2 with no ABORT conditions

**Preflight Check:**
- [ ] Props tightened
- [ ] Battery secured
- [ ] Remote calibrated
- [ ] IMU calibration done (during startup)
- [ ] Another person present

**Takeoff Procedure:**
1. Throttle up slowly to base_speed = 60-80
2. Drone should lift off smoothly
3. Hold altitude, test pitch/roll gently
4. Land gently (reduce throttle gradually)

**Do NOT attempt:**
- Aggressive maneuvers on first flight
- Yaw rotation (get pitch/roll stable first)
- High altitude (stay within 3 meters initially)

---

## Tuning Process

### Understanding Your Drone's Behavior

#### Symptom: Oscillating (jerky, shaky response)
**Possible Causes:**
1. **Proportional gain too high (kp)**
   - Overly aggressive response to error
   - Overshoots target angle repeatedly
2. **Derivative gain too low (kd)**
   - Not damping oscillations enough
3. **Integral windup**
   - Accumulated error causing hunting

**Fix:** 
```cpp
// Current: kp = 0.8
// Try: kp = 0.6 (reduce proportional aggression)
// Then: kd = 0.5 (increase damping)

// Location: Line ~70
PID rollPID = {0.6, 0.02, 0.5, 0, 0, 20.0f, 0};  // Reduced kp, increased kd
```

#### Symptom: Sluggish/slow response
**Possible Causes:**
1. **Proportional gain too low (kp)**
   - Weak response to error
2. **Derivative gain too high (kd)**
   - Over-dampening, suppressing response
3. **Motor minimum speed too high**
   - base_motor_speed = 40 might be limiting range

**Fix:**
```cpp
// Current: kp = 0.8
// Try: kp = 1.2 (increase proportional response)
// Then: kd = 0.3 (reduce excessive dampening)

PID rollPID = {1.2, 0.02, 0.3, 0, 0, 20.0f, 0};  // Increased kp, reduced kd
```

#### Symptom: One direction sluggish (e.g., right roll only)
**Possible Causes:**
1. **Motor imbalance**
   - One ESC responding slower than others
2. **Physical imbalance**
   - Uneven motor thrust
3. **Sensor offset**
   - MPU6050 not calibrated correctly

**Fix:**
1. Re-run calibration (keep drone flat during startup)
2. Verify all motors have same idle thrust
3. Check for bent frame/props

---

## PID Tuning Checklist

### Step 1: Stabilize Pitch & Roll
**Objective:** Level flight without oscillation or drift

**Variables to Adjust:** `kp`, `kd` for rollPID and pitchPID

**Procedure:**
1. Increase `kp` until slight oscillation appears
2. Back off `kp` by 10%
3. Increase `kd` to dampen remaining oscillation
4. Test in flight; iterate

**Target Behavior:**
- Corrects to level in <1 second
- Settles smoothly (no ringing)
- No drift when joystick centered

### Step 2: Add Yaw Stability (If Needed)
**Objective:** Stable yaw response without spin-up/spin-down lag

**Variables to Adjust:** `kp`, `kd` for yawPID

**Note:** Yaw uses rate control (gyro), not angle
- Kp = 1.5 (aggressive)
- Kd = 0.05 (light damping)

**Procedure:**
Similar to pitch/roll, but expect faster response (rate control)

### Step 3: Add Integral If Needed
**Objective:** Eliminate steady-state angle offset

**Only do if:**
- Pitch/roll stable but drone always tilts 5-10° in one direction
- Not a sensor calibration issue

**Adjustment:**
- Increase `ki` from 0.02 to 0.03 (small step)
- Test; if oscillating, revert

**Warning:** Too much integral causes "hunting" (overshooting back and forth)

---

## Serial Monitor Output Interpretation

### Example Output
```
Base: 75 | R:-2.3 P:1.5 YawRate:0.0 | tR:0.0 tP:0.0 tYR:0.0 | Motors: 75,75,75,75
Base: 75 | R:-1.8 P:1.2 YawRate:0.1 | tR:2.5 tP:5.2 tYR:0.0 | Motors: 72,78,78,72
Base: 75 | R:0.2 P:0.1 YawRate:0.0 | tR:0.0 tP:0.0 tYR:0.0 | Motors: 75,75,75,75
```

### Field Breakdown
| Field | Meaning | Normal Range | Bad Signs |
|-------|---------|--------------|-----------|
| **Base** | Throttle speed | 40-180 | N/A |
| **R** | Current roll angle | ±30° | >±45° = oscillating |
| **P** | Current pitch angle | ±30° | >±45° = oscillating |
| **YawRate** | Rotation speed | ±180°/s | High variance = unstable |
| **tR** | Target roll angle | ±30° | Should match stick |
| **tP** | Target pitch angle | ±30° | Should match stick |
| **tYR** | Target yaw rate | ±150°/s | Should match stick |
| **Motors** | Individual speeds | ~same | >20 difference = imbalanced |

### What to Look For

**Stable Flight:**
```
All motors approximately same value (±5)
R/P angles near zero or slowly responding to input
Response to joystick is smooth and predictable
```

**Motor Saturation Warning:**
```
Motors frequently at 180 = losing control authority
If base=150 and motors hit 180, headroom too small
Reduce kp or ki to lower PID corrections
```

**Oscillation Detector:**
```
R or P bouncing ±5° rapidly = oscillating
Example: R:-3 → R:+4 → R:-2 (jittery) = too much kp
Reduce kp by 20%, increase kd by 20%
```

---

## Advanced Tuning (Manual PID Tuning)

### Ziegler-Nichols Method (Simplified)

**Step 1: Eliminate I and D**
```cpp
PID rollPID = {0.8, 0.0, 0.0, 0, 0, 20.0f, 0};  // ki=0, kd=0
```

**Step 2: Increase Kp until oscillation**
- Start: kp = 0.8
- Increase: 0.8 → 1.0 → 1.2 → 1.5 → 2.0
- Stop when: Consistent oscillation at ~2Hz

**Step 3: Back off Kp**
```cpp
// If oscillation starts at kp=1.5:
// Then: kp = 1.5 * 0.6 = 0.9 (Ziegler-Nichols rule)
```

**Step 4: Add Derivative**
```cpp
// Start kd at: kd = 0.9 * 0.125 = ~0.11
// Then increase until oscillation stops
PID rollPID = {0.9, 0.0, 0.15, 0, 0, 20.0f, 0};
```

**Step 5: Add Integral (if needed)**
```cpp
// Only if DC offset exists
PID rollPID = {0.9, 0.02, 0.15, 0, 0, 20.0f, 0};
```

---

## Quick Adjustments Reference

### Too Oscillatory
```cpp
// BEFORE:
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 20.0f, 0};

// AFTER (reduce P, increase D):
PID rollPID = {0.6, 0.02, 0.6, 0, 0, 20.0f, 0};
```

### Too Sluggish
```cpp
// BEFORE:
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 20.0f, 0};

// AFTER (increase P, reduce D):
PID rollPID = {1.2, 0.02, 0.2, 0, 0, 20.0f, 0};
```

### Steady Drift (always tilted)
```cpp
// BEFORE:
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 20.0f, 0};

// AFTER (increase integral):
PID rollPID = {0.8, 0.04, 0.4, 0, 0, 20.0f, 0};  // ki doubled
```

### Windup Behavior (bouncing back)
```cpp
// BEFORE:
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 20.0f, 0};

// AFTER (reduce integral limit):
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 10.0f, 0};  // Limit halved
```

---

## When to Stop Tuning

**Good Enough Indicators:**
- ✅ Responds quickly to control input
- ✅ Settles to target angle in <500ms
- ✅ No oscillations visible
- ✅ Smooth, predictable flying
- ✅ Motors not saturating
- ✅ Can fly in wind without input correction

**Don't Over-Tune:**
- Chasing microsecond-level perfection
- More aggressive gains ≠ better
- Conservative gains = stable flight

---

## Emergency Recovery

### If Drone Becomes Unstable in Flight

1. **IMMEDIATELY:** Reduce throttle to minimum (base_speed → 40)
   - Forces disarmMotors() to reset PIDs
   - Stops uncontrolled behavior

2. **Land immediately**
   - Do not try to recover in flight
   - Get drone on ground safely

3. **Diagnose:**
   - Check gains (might be too aggressive)
   - Check motor directions (might be reversed)
   - Check sensor calibration

4. **Adjust ONE variable:**
   - Reduce kp by 20%
   - Increase kd by 20%
   - Try again

---

## Suggested Starting Values (Conservative)

```cpp
// For initial flight testing:
PID rollPID = {0.6, 0.01, 0.3, 0, 0, 15.0f, 0};
PID pitchPID = {0.6, 0.01, 0.3, 0, 0, 15.0f, 0};
PID yawPID = {1.0, 0.005, 0.03, 0, 0, 10.0f, 0};

// Once stable, increase kp in 10% increments
// If oscillation appears, reduce kp 20%, increase kd 20%
```

---

## Success Criteria

After tuning, your drone should:
- [ ] Hover level without drift
- [ ] Respond quickly (but not jerky) to stick input
- [ ] Return to level when stick centered
- [ ] Fly smoothly in calm conditions
- [ ] Tolerate small wind gusts
- [ ] Have no oscillations
- [ ] Not saturate motors under normal control

**YOU'RE DONE TUNING!** ✅
