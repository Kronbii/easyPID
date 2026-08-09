# PID Tuning Guide for easyPID

A practical guide to tuning PID controllers for optimal performance.

## Table of Contents
1. [Understanding PID Terms](#understanding-pid-terms)
2. [Manual Tuning Procedures](#manual-tuning-procedures)
3. [Anti-Windup](#anti-windup)
4. [Derivative Filtering](#derivative-filtering)
5. [Automatic Tuning](#automatic-tuning)
6. [Common Pitfalls](#common-pitfalls)
7. [Troubleshooting](#troubleshooting)

---

## Understanding PID Terms

### The PID Equation
```
output = Kp*error + Ki*∫(error*dt) + Kd*d(error)/dt
```

Where:
- `error = setpoint - measurement`
  (in `REVERSE` mode the control terms use the negated value; `getError()` still
  reports `setpoint - measurement`, while `getPterm()`/`getDterm()` are
  direction-adjusted)
- `Kp` = Proportional gain
- `Ki` = Integral gain
- `Kd` = Derivative gain

### Proportional Term (P)

**What it does:** Responds proportionally to current error

**Effect of increasing Kp:**
- ✅ Faster response
- ✅ Reduces steady-state error
- ❌ Can cause oscillations if too high
- ❌ May overshoot

**Visual behavior:**
```
Low Kp:  Slow, sluggish response
High Kp: Fast but oscillatory
```

**When to use:**
- Always! Start here
- Dominant term for most systems

**Typical range:** 0.1 to 10.0 (system-dependent)

### Integral Term (I)

**What it does:** Accumulates error over time, eliminates steady-state error

**Effect of increasing Ki:**
- ✅ Eliminates persistent offset
- ✅ Reaches setpoint exactly
- ❌ Causes overshoot
- ❌ Can cause instability
- ❌ Integral windup if saturated

**Visual behavior:**
```
No Ki:   Settles near setpoint (with offset)
With Ki: Reaches setpoint exactly (but slower)
```

**When to use:**
- When system has steady-state error
- When disturbances cause drift
- Not needed if P alone reaches setpoint

**Typical range:** 0.01 to 2.0 (system-dependent)

### Derivative Term (D)

**What it does:** Responds to rate of change, provides damping

**Effect of increasing Kd:**
- ✅ Reduces overshoot
- ✅ Dampens oscillations
- ✅ Anticipates future error
- ❌ Amplifies noise
- ❌ Can make system sluggish if too high

**Visual behavior:**
```
No Kd:   May overshoot, oscillate
With Kd: Smoother approach, less overshoot
```

**When to use:**
- When system overshoots
- When reducing oscillations
- May need derivative filtering if noisy

**Typical range:** 0.01 to 1.0 (system-dependent)

---

## Manual Tuning Procedures

### Method 1: Ziegler-Nichols Open-Loop

**Best for:** Systems where you can apply step input

**Steps:**
1. Apply step input to system (without PID)
2. Measure response curve
3. Find:
   - L (delay time) = time before response starts
   - T (time constant) = time to reach 63.2% of final value
   - K (gain) = (change in output) / (change in input)
4. Calculate:
   ```
   Kp = 1.2 * T / (K * L)
   Ki = Kp / (2 * L)
   Kd = 0.5 * Kp * L
   ```
   (equivalently Ti = 2L and Td = 0.5L. The rule assumes L > 0 and works best
   when T/L is roughly in the range 1–10.)

### Method 2: Manual Trial-and-Error (Recommended for Beginners)

**Best for:** Most systems, especially when starting from scratch

**Steps:**

1. **Set all gains to zero**
   ```cpp
   pid.setTunings(0, 0, 0);
   ```

2. **Tune Kp first (P-only control)**
   - Start with a small value (e.g., 0.1)
   - Gradually increase until system responds quickly
   - Stop when oscillations begin
   - Reduce Kp to 50-75% of oscillation point
   ```cpp
   pid.setTunings(2.0, 0, 0);  // Example after tuning P
   ```

3. **Add Ki if needed (PI control)**
   - Only if system has steady-state error
   - Start with Ki = Kp / 10
   - Increase slowly until error eliminated
   - Watch for overshoot
   ```cpp
   pid.setTunings(2.0, 0.2, 0);  // Added integral
   ```

4. **Add Kd if needed (full PID)**
   - Only if system overshoots or oscillates
   - Start with Kd = Kp / 10
   - Increase slowly until oscillations dampen
   - May need derivative filtering if noisy
   ```cpp
   pid.setTunings(2.0, 0.2, 0.2);  // Added derivative
   pid.setDerivativeFilter(FILTER_EMA, 0.7);  // Filter noise
   ```

### Method 3: Relay Autotuning (Easiest)

**Best for:** Quick starting point, unknown systems

**Steps:**
1. Use the autotuner
   ```cpp
   #include <PIDTuner.h>
   PIDTuner tuner(pid);
   tuner.start(setpoint, relayAmplitude);
   ```
2. Let it run until complete
3. Try different tuning rules:
   - Ziegler-Nichols: Aggressive, fast
   - Tyreus-Luyben: Conservative, less overshoot
   - No-Overshoot: Very conservative
4. Fine-tune manually if needed

### Fine-Tuning Tips

**Too much overshoot:**
- Decrease Kp
- Decrease Ki
- Increase Kd

**Too slow response:**
- Increase Kp
- Increase Ki (if no steady-state error concern)

**Oscillations:**
- Decrease Kp
- Decrease Ki
- Increase Kd
- Enable derivative filtering

**Steady-state error:**
- Increase Ki
- Check for output saturation (increase limits if needed)

---

## Anti-Windup

### What is Integral Windup?

When output saturates (hits min/max limits), the integral term keeps accumulating. This causes:
- Excessive overshoot when error changes sign
- Delayed response when setpoint changes
- Poor performance during saturation

**Symptoms:**
- Large overshoot after long saturation
- System "sticks" at one extreme
- Slow recovery from disturbances

### Anti-Windup Modes

#### 1. NONE (testing only — NOT the default)
```cpp
pid.setAntiWindup(ANTIWINDUP_NONE);
```
- No protection
- Use only for initial testing or when output never saturates

#### 2. CLAMP (Recommended for most applications)

**This is the default.** A controller you never call `setAntiWindup()` on is already using CLAMP.
```cpp
pid.setAntiWindup(ANTIWINDUP_CLAMP);
```
- Stops integral accumulation when output saturates
- Simple and effective
- **Use this for most applications**

#### 3. BACKCALC (Advanced)
```cpp
pid.setAntiWindup(ANTIWINDUP_BACKCALC);
```
- Back-calculation method
- Reduces integral based on saturation amount
- Use for aggressive systems or critical applications

### Setting Integral Limits

You can also limit the integral term directly:
```cpp
pid.setIntegralLimits(-50, 50);  // Bounds the accumulator, not the Ki-scaled term
```

These bound the raw error-time accumulator, so the resulting contribution to
the output is `Ki * limit` — with `Ki = 2.0` the limits above allow ±100 of
output, not ±50. Ignored if `min >= max`.

**When to use:**
- When integral term grows too large
- To prevent excessive overshoot
- For multi-mode control

---

## Derivative Filtering

### Why Filter the Derivative?

The derivative term amplifies high-frequency noise, causing:
- Erratic control output
- Unnecessary actuator wear
- Potential instability

**Symptoms of noise:**
- Output "jumps" or "jitters"
- Unstable control even with low gains
- Audible noise from motors/actuators

### Filter Modes

#### NONE (No filtering)
```cpp
pid.setDerivativeFilter(FILTER_NONE);
```
- Raw derivative
- Maximum responsiveness
- Use only with clean, noise-free sensors

#### EMA (Exponential Moving Average)
```cpp
pid.setDerivativeFilter(FILTER_EMA, 0.7);  // alpha = 0.7
```
- 1st-order low-pass filter
- Simple and effective
- **Recommended starting point**

**Alpha parameter (clamped to 0.0 – 0.999):**
- Higher alpha (0.8-0.95) = more filtering, slower response
- Lower alpha (0.3-0.5) = less filtering, faster response
- Typical: 0.7
- Exactly 1.0 is not permitted: it would freeze the filtered derivative and
  disable the D term entirely, so the value is clamped to 0.999

**When to use:**
- When derivative term causes jitter
- With noisy sensors (encoders, cheap sensors)
- Always test without filtering first, add if needed

**Example:**
```cpp
// Try without filtering first
pid.setDerivativeFilter(FILTER_NONE);

// If noisy, add filtering
pid.setDerivativeFilter(FILTER_EMA, 0.7);

// If still noisy, increase alpha
pid.setDerivativeFilter(FILTER_EMA, 0.85);
```

---

## Automatic Tuning

### Using the Autotuner

```cpp
#include <easyPID.h>
#include <PIDTuner.h>

PIDController pid(1.0, 0.0, 0.0, 0, 255);
PIDTuner tuner(pid);

void setup() {
  pid.begin();
  
  // Start autotuning
  float setpoint = 100.0;
  float relayAmplitude = 20.0;  // 10-20% of output range
  float noiseBand = 2.0;         // Ignore small oscillations
  
  tuner.start(setpoint, relayAmplitude, noiseBand);
}

void loop() {
  float measurement = readSensor();
  
  if (!tuner.isComplete()) {
    float output = tuner.update(measurement);
    applyOutput(output);
  } else {
    // Apply the tuning exactly once, then run normally.
    // Calling reset() every iteration would wipe the integral and the previous
    // error on every sample, degrading the controller to proportional-only
    // with a permanent steady-state offset.
    static bool applied = false;
    if (!applied) {
      float kp, ki, kd;
      tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);
      pid.setTunings(kp, ki, kd);
      pid.reset();
      applied = true;
    }

    // Normal PID control
    float output = pid.update(setpoint, measurement);
    applyOutput(output);
  }
}
```

### Choosing Relay Amplitude

**Too small:** Weak oscillations, poor measurement
**Too large:** Excessive stress on system, unsafe

**Guidelines:**
- Start with 10-20% of output range
- Ensure oscillations are visible
- Keep within safe operating limits

### Choosing Tuning Rule

After autotuning completes, you can apply different rules:

```cpp
tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);  // Aggressive
tuner.getTunings(kp, ki, kd, TUNING_TYREUS_LUYBEN);    // Moderate
tuner.getTunings(kp, ki, kd, TUNING_NO_OVERSHOOT);     // Conservative
```

**Which to use:**
- **Ziegler-Nichols:** Fast response, may overshoot
- **Tyreus-Luyben:** Good compromise, less overshoot
- **Pessen:** Very fast, some overshoot
- **No-Overshoot:** Conservative, minimal overshoot

**Recommendation:** Start with Tyreus-Luyben, adjust if needed

---

## Common Pitfalls

### 1. Wrong Sample Time
**Problem:** Inconsistent timing breaks integral and derivative calculations

**Solution:**
```cpp
// Keep sample time consistent
const unsigned long SAMPLE_TIME = 100;  // 100ms

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= SAMPLE_TIME) {
    lastTime = millis();
    float output = pid.update(setpoint, measurement);
  }
}
```

### 2. Wrong Control Direction
**Problem:** Controller pushes away from setpoint

**Solution:**
```cpp
// For heating, motor speed increase:
pid.setDirection(DIRECT);

// For cooling, reverse systems:
pid.setDirection(REVERSE);
```

### 3. Output Limits Too Narrow
**Problem:** Insufficient control authority

**Solution:**
```cpp
// Ensure limits match your actuator range
pid.setOutputLimits(0, 255);  // For analogWrite
pid.setOutputLimits(-90, 90);  // For servo angles
```

### 4. Not Calling begin()
**Problem:** Timing not initialized, unpredictable behavior

**Solution:**
```cpp
void setup() {
  pid.begin();  // Always call this!
}
```

### 5. Forgetting to Reset After Tuning
**Problem:** Old integral/derivative state causes strange behavior

**Solution:**
```cpp
pid.setTunings(new_kp, new_ki, new_kd);
pid.reset();  // Clear state
```

---

## Troubleshooting

### System oscillates continuously
- **Cause:** Kp too high, or Ki too high
- **Fix:** Reduce Kp by 50%, or reduce Ki

### System never reaches setpoint
- **Cause:** Ki = 0, or output limits too narrow
- **Fix:** Add integral term, or increase output limits

### Large overshoot
- **Cause:** Kp or Ki too high, Kd too low
- **Fix:** Reduce Kp/Ki, or increase Kd

### Slow response
- **Cause:** Gains too low
- **Fix:** Increase Kp gradually

### Erratic/jittery output
- **Cause:** Noisy derivative, or Kd too high
- **Fix:** Enable derivative filtering, reduce Kd

### Response changes over time
- **Cause:** Integral windup
- **Fix:** Enable anti-windup (CLAMP mode)

### Autotuner never completes
- **Cause:** Relay amplitude too small, or system non-responsive
- **Fix:** Increase relay amplitude, check system connection

---

## Advanced Tips

### Gain Scheduling
For systems with varying dynamics:
```cpp
if (measurement < 50) {
  pid.setTunings(kp_low, ki_low, kd_low);
} else {
  pid.setTunings(kp_high, ki_high, kd_high);
}
```

### Setpoint Ramping
For smooth transitions:
```cpp
float rampedSetpoint = currentSetpoint + 
                       constrain(targetSetpoint - currentSetpoint, -1, 1);
float output = pid.update(rampedSetpoint, measurement);
```

### Monitoring
Use introspection for debugging:
```cpp
Serial.print("Error: "); Serial.print(pid.getError());
Serial.print(" P: "); Serial.print(pid.getPterm());
Serial.print(" I: "); Serial.print(pid.getIterm());
Serial.print(" D: "); Serial.println(pid.getDterm());
```

---

## Summary Checklist

- [ ] Start with P-only control (Ki=0, Kd=0)
- [ ] Tune Kp until fast response (before oscillations)
- [ ] Add Ki only if steady-state error exists
- [ ] Add Kd only if overshoot/oscillations occur
- [ ] Enable anti-windup (ANTIWINDUP_CLAMP)
- [ ] Add derivative filtering if output is noisy
- [ ] Use consistent sample time
- [ ] Check control direction matches system
- [ ] Call begin() in setup()
- [ ] Reset state after changing gains

**Happy tuning!** 🎛️
