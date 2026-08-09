# Changelog

All notable changes to the easyPID library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.19] - 2026-08-09

### Fixed
- The `@file` tag in `src/easyPID.cpp` said `PIDController.cpp`, a name that
  does not exist in the repository, so Doxygen filed the translation unit under
  a phantom file and cross-references from `easyPID.h` resolved to nothing.
- Including `easyPID.h` after a library that defines `DIRECT`/`REVERSE` as
  macros (PID_v1 does) made the preprocessor rewrite the `ControlDirection`
  enumerator list, and the compiler blamed easyPID for another library's macros
  with an unreadable parse error. A `#error` now names the actual cause and the
  two ways out. The enumerators are unchanged; renaming them would break every
  existing call site, so that is a 2.0 consideration.

### Documentation
- Removed the "based on tracker.h lines N-M" citations throughout. That file is
  not in the repository, so none of the eight references could be checked. The
  substantive "original vs. enhanced" explanations are kept.
- The constructor block now lists the defaults it actually applies, which were
  documented nowhere: `ANTIWINDUP_CLAMP`, `FILTER_NONE`, alpha 0.8, `DIRECT`,
  100 ms sample time, and integral limits inactive.
- `getPterm()`/`getDterm()` note that they are direction-adjusted and therefore
  carry the opposite sign to `getError()` under `REVERSE`. The terms sum to the
  pre-clamp output; the error is the plain physical error. Both are useful, and
  the difference now has a stated contract instead of being an accident.

---

## [1.0.18] - 2026-08-09

### Fixed
- **AutoTunePID could never induce a limit cycle.** The tuner swings its output
  symmetrically about zero, so half of every relay period was a negative drive
  into a plant that only accepts 0-255. The measurement peaked at 22 against a
  setpoint of 100, never crossed it, the relay never switched, and tuning
  aborted on the timeout with no result. The sketch now centres the relay on an
  `OUTPUT_BIAS` operating point, which is what a unipolar actuator requires.
- The setpoint (100) was also above the plant's ceiling of 75, so even a
  working relay could not have reached it. Now 37, with the time constant
  raised to 1.0 s so the limit cycle spans ~18 samples instead of ~2. At the
  old values the measured period was near the sampling limit and the resulting
  gains were meaningless.
- The sketch spun forever if tuning failed, because it only ever tested
  `isComplete()` and never noticed the tuner had returned to `TUNER_IDLE`. It
  now detects failure, explains the likely causes, and holds the output at zero.
- The progress line printed on every loop iteration whose percentage happened
  to be a multiple of ten, repeating the same value hundreds of times. It now
  prints only on change.
- Braced the `switch` case bodies that declare variables, and moved the tuning
  rule names to flash via `F()`.

### Verified
Simulated end to end against the sketch's own plant model: tuning completes in
9.2 s with `Ku = 8.95`, `Pu = 1.82 s` (18 samples per cycle), and all four rule
sets then drive the loop to setpoint with the expected overshoot ordering --
No-Overshoot 0.7%, Tyreus-Luyben 0%, Ziegler-Nichols 3.8%, Pessen 6.5%.

---

## [1.0.17] - 2026-08-09

### Fixed
- **BasicPID and MultiLoopPID chased setpoints their simulated plants could not
  reach.** Each plant settles at `PROCESS_GAIN * 100` at full output, so
  BasicPID asked for 100 from a plant with a ceiling of 80, and MultiLoopPID's
  second loop asked for 120 from a ceiling of 70. Both pinned the output at 255
  forever. The sketches advertised as demonstrations of PID control were
  demonstrating integral windup. Gains raised to 1.5, 1.2 and 1.6 respectively;
  all three loops now settle exactly on setpoint. Verified by simulating the
  sketches' own plant math.
- Labels and values printed on separate lines throughout both sketches, from
  `Serial.println(F("Setpoint: "))` followed by `Serial.print(value)`.
- MultiLoopPID injected its simulated noise into the plant *state*, making it a
  random walk the integrator had to chase, and drew it from `random(-10, 10)`,
  which is asymmetric (-1.0 to +0.9, mean -0.05). Noise is now zero-mean and
  applied to the value handed to the controller, which is what sensor noise
  actually is.
- MultiLoopPID's periodic P/I/D block interleaved non-CSV lines into the CSV
  stream, corrupting it for Serial Plotter. It is now behind `VERBOSE_TERMS`,
  off by default.
- `randomSeed()` is now called, so the noise is not identical on every run.

### Changed
- MultiLoopPID uses the explicit-`dt` overload, both to demonstrate it and
  because the loop is already gated to a fixed period, so the controller and
  the plant simulation now agree exactly.

---

## [1.0.16] - 2026-08-09

### Fixed
- `PIDTuner::start()` validates its parameters and returns `false` for a
  non-positive `relayAmplitude` (which drives nothing, so no limit cycle can
  form) or a negative `noiseBand` (which inverts the switching thresholds and
  makes the relay chatter every sample). Both previously started a run that
  could only fail.
- A tuning run that times out now computes results from the cycles it did
  collect instead of discarding them. `calculateResults()` already enforced
  `MIN_CYCLES_FOR_TUNING`, but nothing ever called it on the timeout path, so
  that guard was unreachable and a run that gathered 4 of 5 cycles threw all of
  them away.
- `getTunings()` and the internal `applyTuningRule()` are `const`; neither
  mutates the tuner.

---

## [1.0.15] - 2026-08-09

### Fixed
- `setDirection()` now negates the carried integral and derivative state when
  the direction actually changes. Those values were accumulated under the
  opposite sign convention, so after a switch the integrator fought the new
  direction until it bled off, and the sign flip in `previousError_` produced
  one large spurious derivative sample. Setting the direction it already has is
  now a no-op rather than a state disturbance.

---

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
