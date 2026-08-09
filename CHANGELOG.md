# Changelog

All notable changes to the easyPID library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.14] - 2026-08-09

### Fixed
- `setOutputLimits()` and `setIntegralLimits()` now ignore `min >= max`.
  Inverted limits made every output compare as saturated, which permanently
  inhibited integration and left the controller unable to reach setpoint.
- The derivative-filter `alpha` is clamped strictly below 1.0. At exactly 1.0
  the EMA reduces to `filtered = 1*filtered + 0*raw`, freezing the filtered
  derivative at its initial value and killing the D term for the rest of the
  run. The old clamp permitted that value.

### Removed
- The dead integral-limit seeding in `setOutputLimits()`. It assigned
  `integralMin_`/`integralMax_` while `integralLimitsSet_` was false, but those
  members are only read when it is true, so the assignment never had any effect.
  The comment claiming integral limits "default to output limits" was
  misleading: until `setIntegralLimits()` is called there is no integral limit.

### Documentation
- `setIntegralLimits()` bounds the raw error-time accumulator, not the
  Ki-scaled I term. The contribution to the output is `Ki * limit`. This was
  never stated and the parameter names implied otherwise.

---

## [1.0.13] - 2026-08-09

### Fixed
- **`update()` fabricated a timestep when no time had passed.** `dt` was
  computed from `millis()`, and when it came out as zero the controller
  substituted a whole `sampleTime_` (100 ms by default) and integrated as if
  that period had elapsed. The documented usage calls `update()` unconditionally
  from `loop()`, which on any reasonably fast board runs many times per
  millisecond, so the integral accrued at up to 100x the true rate. Measured:
  ten calls with the clock frozen integrated a full second. Such calls now
  return the previous output unchanged, and `lastTime_` is not advanced, so
  sub-millisecond time carries into the next call instead of being discarded.
- The manual-`dt` overload likewise returns the previous output for a
  non-positive `dtMs` rather than inventing a timestep. Previously
  `setSampleTime(0)` made the divide-by-zero guard restore `dt = 0` and the
  derivative became inf, then NaN.
- `setSampleTime()` now ignores zero.

### Changed
- `setSampleTime()` is documented as advisory. It never gated the update rate,
  and now that no timestep is ever fabricated it plays no part in the control
  math at all. The previous note calling it "mainly for documentation" was
  wrong in the opposite direction: it *was* driving the math, on exactly the
  path that was broken.

---

## [1.0.12] - 2026-08-09

### Fixed
- **`ANTIWINDUP_CLAMP` could pin the output at a limit indefinitely.** The
  rollback fired on saturation alone, with no test of the error direction, so it
  cancelled accumulation that would have *relieved* saturation just as readily as
  accumulation that worsened it. Once the I term alone exceeded the output
  limit, the integrator froze and the controller stayed on the rail regardless
  of the error. Reproduced: integral wound to `I = 300` against a ceiling of 50,
  then a sustained error of `-100` for 300 samples left the output at exactly
  `50.000` the whole time with the integral unmoved. Now only accumulation that
  drives further into saturation is inhibited; the same scenario unwinds to
  `I = 100` and the output leaves the rail.
- The rollback restores the pre-accumulation value instead of subtracting
  `error * dt`, making it an exact inverse when `setIntegralLimits()` truncated
  the accumulation. Previously it removed more than had been added.

### Note
This changes the numeric response of saturating controllers using the default
anti-windup mode. A loop tuned around the frozen-integral behaviour will react
differently — better, but differently.

---

## [1.0.11] - 2026-08-09

### Fixed
- `getIterm()` reported the integral term as it was *before* anti-windup ran,
  so during saturation it kept climbing even though the integrator was being
  held. That made it look as though anti-windup was not working, which is
  precisely the situation the getter exists to diagnose. It now reflects the
  post-correction integrator state.

---

## [1.0.10] - 2026-08-09

### Fixed
- `getError()` is now always `setpoint - measurement`, as its documentation
  always claimed. `REVERSE` mode negated the error in place, so introspection
  reported the internal sign-corrected control error instead: a REVERSE
  controller at setpoint 100 with measurement 40 reported `-60` rather than
  `+60`. The reported error and the error driving the terms are now separate
  values.

### Note for REVERSE users
`DIRECT` controllers (the default) are unaffected. If you have a `REVERSE`
controller and were compensating for the old sign when logging or plotting
`getError()`, remove that compensation. Control behaviour itself is unchanged.

---

## [1.0.9] - 2026-08-09

### Fixed
- `update(setpoint, measurement, dtMs)` did not refresh the internal timing
  reference, so a sketch that used the manual-`dt` overload and later called the
  automatic overload had all the intervening wall-clock counted as a single
  sample period. Measured: ten manual 100 ms updates spread over 10 s of real
  time, then one automatic update, integrated 10.1 s in one step instead of
  0.1 s.
- `reset()` now also restarts the timing reference. It is typically called after
  a pause or a large setpoint change, and without this the next automatic update
  integrated the entire idle period in one step.

### Removed
- Private member `autoTiming_`, which was assigned in the constructor and never
  read. No public API change.

---

## [1.0.8] - 2026-08-09

### Fixed
- `ANTIWINDUP_BACKCALC` divided by `ki` without guarding against zero. A PD
  controller (`ki = 0`) configured with back-calculation computed `1.0f / 0.0f`,
  wrote inf into the integral, and every subsequent output was inf or NaN --
  permanently, since inf never recovers. Back-calculation is now skipped when
  there is no meaningful integral gain to correct.

---

## [1.0.7] - 2026-08-09

### Fixed
- No more derivative kick on the first update after `begin()` or `reset()`.
  With `previousError_` still at its initial `0.0`, the first sample computed
  `(error - 0) / dt`, producing a derivative proportional to the entire error
  at exactly the moment the error is normally largest. With `Kd = 10`, a step
  to an error of 100 at `dt = 0.1 s` produced a D term of 10000. The derivative
  now starts at zero and develops from the second sample onward.

---

## [1.0.6] - 2026-08-09

### Fixed
- `getProgress()` returned `0.0` after tuning finished instead of `1.0`, because
  it early-returned for any state other than `TUNER_RELAY_STEP`. A sketch
  displaying progress showed it collapse back to zero on success.
- `getState()` no longer reports `TUNER_COMPLETE` when the run produced no
  usable result. It now falls back to `TUNER_IDLE`, so `getState()` and
  `isComplete()` can no longer disagree.
- `TUNER_ANALYZING` was declared but never entered. It is now set while results
  are computed. It is transient within a single `update()` call, and the header
  documents it as such rather than implying it is externally observable.
- `PIDTuner` held a `PIDController&` that was never used. `start()` now calls
  `reset()` on it: the relay run drives the plant directly, so any integral the
  controller had accumulated is stale by the time tuning completes.

---

## [1.0.5] - 2026-08-09

### Fixed
- The "process not responding" timeout is now keyed to the last relay edge
  rather than the last completed cycle. A slow process switches the relay twice
  per period, so the old reference silently capped the tunable limit-cycle
  period at `MAX_WAIT_TIME_MS` (60 s) and aborted tuning on exactly the
  lag-dominant thermal processes autotuning is most useful for. Verified: a
  plant with `Pu = 69.6 s` now tunes successfully instead of timing out.

### Added
- An absolute 15-minute deadline per tuning run, as a backstop for pathological
  cases where the relay keeps switching but no consistent limit cycle emerges.

---

## [1.0.4] - 2026-08-09

### Fixed
- The ultimate-gain estimate now accounts for the noise band acting as relay
  hysteresis: `Ku = 4d / (pi * sqrt(a^2 - h^2))`. The previous ideal-relay form
  `4d / (pi * a)` ignored the hysteresis and biased `Ku` low, increasingly so as
  `noiseBand` grew relative to the oscillation.
- Tuning results are now rejected when the measured swing is not larger than
  the noise band, instead of reporting a confident value derived from noise.

---

## [1.0.3] - 2026-08-09

### Fixed
- **The autotuner could never complete.** `cyclesDetected_` was incremented
  inside `if (cyclesDetected_ > 0)`, but `start()` initialises it to `0`, so the
  counter was pinned at zero forever. `isComplete()` never returned `true`,
  `getProgress()` never rose above `0.0`, and because the timeout reference was
  refreshed on every relay switch the tuner never timed out either — it simply
  relayed indefinitely. Confirmed by simulation: 400 s of relay operation with
  `Ku = Pu = 0`.
- **Oscillation amplitude was measured incorrectly.** `peakHigh_` was reset to
  the measurement at each switching instant and then never tracked upward, so
  `peakHigh_ - peakLow_` recorded roughly `2 x noiseBand` instead of the real
  limit-cycle swing. Extremes are now accumulated continuously across each
  cycle, which is required because the process peak lags the relay switch.
- Amplitude is now the half peak-to-peak swing, matching the `a` term in the
  describing-function relation `Ku = 4d / (pi * a)`. The previous code passed
  the full peak-to-peak span, understating `Ku` by a factor of two.

### Removed
- Private members `peakHigh_`, `peakLow_`, `peakHighTime_`, `peakLowTime_`,
  `lookingForPeak_`, `peakType_` and `lastPeakTime_`, superseded by the
  cycle-window measurement. No public API change.

---

## [1.0.2] - 2026-08-09

### Fixed
- Relay edge detection no longer uses a function-local `static` inside
  `PIDTuner::detectPeak()`. That variable was shared by every `PIDTuner`
  instance in the sketch and was initialised only once for the lifetime of the
  program, so a second tuner (or a second `start()` on the same tuner) saw
  corrupted edge state. It is now a per-instance member.
- Initialise every `PIDTuner` member in the constructor, so calling `update()`
  before `start()` can no longer read indeterminate values.

---

## [1.0.1] - 2026-08-09

### Fixed
- Stop redefining Arduino's `PI` macro in `PIDTuner.cpp`, which emitted a
  "PI redefined" warning on every compilation (reproduced on AVR and ESP32).
  The tuner now uses a private float constant instead.

---

## [1.0.0] - 2024-01-15

### Added
- **Core PIDController class** with full PID control functionality
  - Proportional, Integral, and Derivative control terms
  - dt-aware integral accumulation and derivative calculation
  - Support for automatic timing (millis-based) and manual dt input
  - Dual update patterns: `update(setpoint, measurement)` and `update(setpoint, measurement, dt)`
  - Alternative pattern: `setSetpoint()`, `setMeasurement()`, `compute()`
  
- **Anti-windup protection** with multiple modes
  - NONE: No anti-windup (for testing)
  - CLAMP: Prevent integral accumulation during saturation
  - BACKCALC: Back-calculation method for advanced applications
  
- **Derivative filtering** to reduce noise sensitivity
  - NONE: Raw derivative (no filtering)
  - EMA: Exponential Moving Average (1st-order low-pass filter)
  - Configurable filter coefficient (alpha parameter)
  
- **Full state introspection** for debugging and monitoring
  - `getError()`: Current control error
  - `getPterm()`: Proportional term contribution
  - `getIterm()`: Integral term contribution
  - `getDterm()`: Derivative term contribution
  - `getOutput()`: Last computed output value
  
- **Runtime configuration** methods
  - `setTunings()`: Update PID gains on-the-fly
  - `setOutputLimits()`: Configure output saturation bounds
  - `setIntegralLimits()`: Set separate integral term limits
  - `setSampleTime()`: Document expected sample period
  - `setDirection()`: DIRECT or REVERSE control action
  - `reset()`: Clear all state (integral, derivative, errors)
  
- **Optional PIDTuner add-on module** for automatic tuning
  - Relay/limit-cycle autotuning method
  - Oscillation detection and parameter extraction
  - Ultimate gain (Ku) and period (Pu) calculation
  - Four tuning rule options:
    - Ziegler-Nichols (classic, aggressive)
    - Tyreus-Luyben (less overshoot)
    - Pessen Integral Rule (fast response)
    - No Overshoot (conservative)
  - Progress monitoring and timeout protection
  - Safe oscillation amplitude and noise band configuration
  
- **Three comprehensive examples**
  - BasicPID: Single controller with simulated first-order process
  - MultiLoopPID: Two independent controllers demonstrating multi-instance capability
  - AutoTunePID: Complete autotuning workflow with multiple tuning rules
  
- **Complete documentation**
  - README.md with quick start, API reference, and feature comparison
  - docs/tuning_guide.md with practical tuning procedures and troubleshooting
  - Inline code documentation with Doxygen-style comments
  
- **Arduino library metadata**
  - library.properties for Arduino Library Manager compatibility
  - keywords.txt for Arduino IDE syntax highlighting
  - MIT License (LICENSE file)
  - Arduino Library Specification 1.5 compliant structure

### Technical Details
- **Multi-instance safe**: No global mutable state, create unlimited controllers
- **Memory efficient**: Uses float (not double) for AVR compatibility
- **Proven algorithm**: Based on field-tested light-tracking robot implementation
- **Hardware agnostic**: No dependencies on specific sensors or actuators
- **AVR optimized**: Tested on Arduino Uno (ATmega328P)
- **Portable**: Compatible with most Arduino architectures (AVR, ARM, ESP8266, ESP32)

### Dependencies
- Arduino Core library (included with Arduino IDE)
- No external dependencies required

### Tested Platforms
- Arduino Uno (ATmega328P)
- Arduino Nano
- Arduino Mega 2560
- Compatible with most Arduino boards

### Known Limitations
- Sample time consistency is user's responsibility (library doesn't enforce it in auto-timing mode)
- Autotuner requires stable process (no major disturbances during tuning)
- Derivative filtering limited to 1st-order (EMA) in v1.0.0

### Migration Notes
This is the initial release. No migration necessary.

---

## Future Roadmap (Not in v1.0.0)

Potential features for future versions:
- 2nd-order derivative filtering
- Derivative-on-measurement (to avoid derivative kick)
- Setpoint weighting/ramping
- Bumpless transfer support
- Additional autotuning algorithms
- EEPROM parameter persistence

---

## Version Format

Version numbers follow [Semantic Versioning](https://semver.org/):
- **MAJOR**: Incompatible API changes
- **MINOR**: New functionality (backward-compatible)
- **PATCH**: Bug fixes (backward-compatible)

Example: v1.2.3
- 1 = Major version
- 2 = Minor version  
- 3 = Patch version
