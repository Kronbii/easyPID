# easyPID — agent guide

Instructions for AI agents and new contributors working in this repository.
This is an **Arduino IDE library**, which constrains layout, metadata and
release process far more than an ordinary C++ project. Read the constraints
before editing.

---

## 1. What this is

A hardware-agnostic PID controller for Arduino, plus an opt-in relay autotuner.

| Path | Role |
|---|---|
| `src/easyPID.h`, `src/easyPID.cpp` | `PIDController` — the whole library for most users |
| `src/PIDTuner.h`, `src/PIDTuner.cpp` | `PIDTuner` — opt-in relay/limit-cycle autotuner |
| `examples/*/` | Three sketches, surfaced in **File → Examples → easyPID** |
| `extras/test/` | Host-side tests. Ignored by the IDE; see §5 |
| `library.properties` | Library Manager metadata. Holds the canonical version |
| `keywords.txt` | IDE syntax highlighting. **Tab-separated, single tabs** |
| `.github/workflows/` | `arduino-lint.yml` (metadata/structure), `build.yml` (compiles + host tests) |

Everything under `src/` is compiled into **every** sketch, including
`PIDTuner.cpp` when only `easyPID.h` is included. The linker discards it if
unreferenced. "Optional module" is an API statement, not a build one — don't
re-introduce claims to the contrary.

---

## 2. Arduino constraints that bite

- **`src/` layout** — flat, no nested source dirs, no build system. The IDE
  compiles every `.cpp` it finds.
- **`extras/` is ignored** by the IDE and the compiler. It is the only
  spec-sanctioned home for tests, tools and notes. It *is* still shipped in the
  Library Manager archive, so keep it small.
- **`keywords.txt` uses single TAB separators.** Spaces silently break
  highlighting. Verify with `cat -A keywords.txt` — you want `^I`, not spaces.
- **Example folder name must match its `.ino`** (`examples/BasicPID/BasicPID.ino`).
- **`library.properties` has no line-continuation syntax.** `paragraph` must be
  one physical line, and the file needs a trailing newline.
- **AVR is the floor.** An Uno has 2 KB of RAM. Use `float`, never `double`;
  wrap sketch string literals in `F()` to keep them out of RAM.
- **`millis()` rollover** — always subtract into an *unsigned* type
  (`now - last`), never compare absolute timestamps.
- **Global-scope enum names leak.** `DIRECT`/`REVERSE` are unprefixed for
  backward compatibility and collide with `PID_v1`'s macros; `easyPID.h` carries
  an `#error` guard that names the real cause. Every *other* enumerator here is
  prefixed (`ANTIWINDUP_`, `FILTER_`, `TUNING_`, `TUNER_`). Keep it that way.
  Renaming `DIRECT`/`REVERSE` is a **2.0** change.

---

## 3. The public API surface

Changing any of this is **not** a patch:

- `PIDController`: `begin`, both `update` overloads, `setSetpoint`,
  `setMeasurement`, `compute`, `setTunings`, `setOutputLimits`,
  `setIntegralLimits`, `setAntiWindup`, `setDerivativeFilter`, `setSampleTime`,
  `setDirection`, `reset`, `getError`, `getPterm`, `getIterm`, `getDterm`,
  `getOutput`
- `PIDTuner`: `start`, `update`, `isComplete`, `getTunings`, `applyTunings`,
  `getUltimateGain`, `getUltimatePeriod`, `cancel`, `getState`, `getProgress`
- Enums: `AntiWindupMode`, `DerivativeFilterMode`, `ControlDirection`,
  `TuningRule`, `TunerState`

Rules of thumb:

- **Append enumerators, never insert.** Sketches compare them as ints.
  `TUNER_FAILED` was appended after `TUNER_COMPLETE` for exactly this reason.
- **A defaulted parameter is additive** (source-compatible) → MINOR.
- **Private members are free to change.** Several were removed during 1.0.x
  with no version consequence beyond the patch bump.

Two conventions worth knowing before you "fix" them:

- `getError()` returns `setpoint - measurement` **in both directions**.
  `getPterm()`/`getIterm()`/`getDterm()` are direction-adjusted, so they sum to
  the pre-clamp output and carry the opposite sign under `REVERSE`. This
  asymmetry is deliberate and documented.
- `setSampleTime()` is **advisory**. It does not gate the update rate and takes
  no part in the control math. `dt` comes from measured elapsed time, or from
  the `dtMs` you pass — **in milliseconds**, not seconds.

---

## 4. Versioning and release

**This library is published.** It is registered in the Arduino Library Manager
and appears in the IDE for every user. Confirm what the world currently sees
with:

```bash
arduino-cli lib update-index && arduino-cli lib search easyPID
```

A released version is not a draft you can amend — it is what users install.

### Policy

1. **One commit per defect.** Each commit bumps the version in
   `library.properties` *and* adds its own `CHANGELOG.md` section. The history
   stays bisectable and each bump has exactly one reason.
2. **PATCH** for bug fixes and documentation. **MINOR** for additive API.
   MAJOR for anything that breaks a sketch.
3. **Tag only the final version of a sweep.** Library Manager indexes every
   tag, so tagging each intermediate patch would show users a dozen releases in
   one day. Intermediate versions live in history, unreleased.
4. Tags are **bare semver, no `v` prefix** — the first tag is `1.0.0`. A
   `v`-prefixed URL in `CHANGELOG.md` would 404.
5. Only add a `CHANGELOG.md` link definition for versions actually tagged.

Commit message: `fix(scope): what changed (X.Y.Z)`, body explaining what was
wrong, how it manifested, and the evidence. Scopes in use: `pid`, `tuner`,
`examples`, `docs`, `test`.

### Publishing — the order matters

> **Library Manager indexes git *tags*, not GitHub releases.** The moment a tag
> matching `library.properties` lands on the remote, Arduino's crawler will pick
> it up and ship it to every IDE user. **Never push a tag before the change is
> merged and reviewed.** There is no unpublish.

```
1. branch            git checkout -b fix/<topic>
2. commit            one per defect, each bumping the version + CHANGELOG
3. verify            all three checks in §5 must pass
4. push BRANCH ONLY  git push -u origin fix/<topic>        # never --tags
5. open PR           gh pr create --base main
6. draft release     gh release create <version> --draft --target main \
                       --notes-file <notes>
                     # a draft does NOT create the tag
7. MERGE             maintainer merges the PR
8. publish release   creates the tag on main -> this is the publish step
9. Library Manager   picks up the new tag automatically, usually within an hour.
                     No resubmission: the library is already registered
10. confirm          arduino-cli lib update-index
                     arduino-cli lib search easyPID     # new version listed?
```

Prefer a **fast-forward merge**, or re-tag afterwards. If the PR is merged with
a merge commit and the tag was cut from the branch tip, the tag points at a
commit that is not on `main`.

`.github/workflows/arduino-lint.yml` runs with `library-manager: update`, which
is the correct mode for an **already-registered** library. `submit` is only for
first-time registration — do not switch it back.

### Testing a change on hardware before publishing

The library is not installed in the sketchbook by default. Symlink the working
tree so the IDE compiles what you are editing:

```bash
ln -s "$PWD" "$(arduino-cli config get directories.user)/libraries/easyPID"
```

Everything in this repo is verified by simulation and by compiler. Flash the
autotune example on a real board before publishing anything that touches the
control math.

---

## 5. Verification — run all of it before claiming done

```bash
# 1. Host tests (fast, no hardware, catches control-behaviour regressions)
make -C extras/test test

# 2. Metadata and structure, exactly as CI runs it
arduino-lint --compliance strict --library-manager update

# 3. Real compilation, all examples, warnings visible
for B in arduino:avr:uno esp32:esp32:esp32; do
  for E in BasicPID MultiLoopPID AutoTunePID; do
    arduino-cli compile -b "$B" --library . --warnings all "examples/$E"
  done
done
```

`arduino-lint` is not in most package managers; install a prebuilt binary:

```bash
mkdir -p bin && BINDIR="$PWD/bin" \
  sh <(curl -fsSL https://raw.githubusercontent.com/arduino/arduino-lint/main/etc/install.sh)
```

> **`arduino-lint` passing means nothing about correctness.** It never invokes a
> compiler. It was green for the entire 1.0.0 release, during which the
> autotuner could not complete, two examples chased unreachable setpoints, and
> every build emitted a `PI redefined` warning. Steps 1 and 3 are the ones that
> find real defects.

### The host test harness

`extras/test/mock/Arduino.h` replaces the real header and supplies a **virtual
clock**: `millis()` reads a counter tests advance with `mockAdvance(ms)`.
Without it, nothing timing-dependent is testable — anti-windup, the derivative,
and the tuner's period measurement all hinge on elapsed time.

Plants are **first-order-plus-dead-time**. The dead time is essential: a relay
loop needs phase lag to oscillate, so an autotuner test against a pure lag
either won't oscillate or will oscillate at the sampling limit and prove
nothing.

When you fix a bug, add a case that fails before and passes after.

---

## 6. Hard-won gotchas

Things that already went wrong here, so they don't again:

- **Test the *failure* you claim to have found.** A first pass "cleared" the
  `CLAMP` anti-windup logic because the test let the P term dominate, so the
  output hit the opposite rail and never exercised the lock-in. The real bug —
  the output pinning at a limit forever — needed `|I| > |P|` to reproduce.
  Construct the precise condition; a passing test proves nothing until you have
  seen it fail for the right reason.
- **Verify example constants numerically.** Each simulated plant tops out at
  `PROCESS_GAIN * 100`. Two sketches asked for setpoints above their ceiling and
  demonstrated windup instead of control. `extras/test/test_examples.cpp` now
  guards this — its constants are *mirrored* from the `.ino` files, so update
  both together.
- **Relay autotuning needs a well-sampled limit cycle.** A period of two or
  three samples is at the Nyquist limit and every derived gain is noise. Aim for
  ≥ 10 samples per cycle; the example gets ~18.
- **Relay autotuning on unipolar hardware needs an output bias.** A relay
  centred on zero spends half of each period commanding a negative drive the
  hardware clips to zero, and no limit cycle forms.
- **Never fabricate a timestep.** Returning the previous output when no time has
  elapsed is correct; substituting a nominal sample period silently multiplies
  the integral rate.
- **Check arithmetic against the describing function, not intuition.** `Ku`
  uses the *half* peak-to-peak amplitude and corrects for the noise band acting
  as hysteresis: `Ku = 4d / (π·√(a² − h²))`.

---

## 7. References

**This repository**

| | |
|---|---|
| Repository | https://github.com/Kronbii/easyPID |
| Library Manager entry | `arduino-cli lib search easyPID` (registered; `1.0.0` indexed as of the 1.1.0 sweep) |
| Releases | https://github.com/Kronbii/easyPID/releases |
| The 1.0.1 → 1.1.0 audit sweep | [PR #2](https://github.com/Kronbii/easyPID/pull/2) — 24 commits, one per defect; the model for how a sweep is structured |
| Changelog | [CHANGELOG.md](CHANGELOG.md) — every entry states what was wrong and how it showed |
| Tuning guide | [docs/tuning_guide.md](docs/tuning_guide.md) |
| Host tests | [extras/test/README.md](extras/test/README.md) |

**Arduino specifications and tooling**

| | |
|---|---|
| Library specification (layout, `library.properties`, `keywords.txt`) | https://arduino.github.io/arduino-cli/latest/library-specification/ |
| Library Manager submission & update rules | https://github.com/arduino/library-registry/blob/main/FAQ.md |
| `arduino-lint` rule reference | https://arduino.github.io/arduino-lint/latest/rules/library/ |
| `arduino-cli` commands | https://arduino.github.io/arduino-cli/latest/commands/arduino-cli/ |
| `arduino/compile-sketches` action (used in `build.yml`) | https://github.com/arduino/compile-sketches |
| `arduino/arduino-lint-action` | https://github.com/arduino/arduino-lint-action |
| Valid `category` values | https://arduino.github.io/arduino-cli/latest/library-specification/#library-categories |

**Control theory behind the autotuner**

| | |
|---|---|
| Relay (limit-cycle) autotuning | Åström & Hägglund, *Automatic Tuning of PID Controllers* — the describing-function basis for `Ku = 4d / (π·a)` |
| Hysteresis correction | With relay hysteresis `h`, the critical point moves off the real axis; projecting back gives `Ku = 4d / (π·√(a² − h²))`, which is what `calculateResults()` implements |
| `a` is the **half** peak-to-peak swing | not the full span — getting this wrong understates `Ku` by 2x |
| Ziegler-Nichols open-loop (FOPDT) | `Kp = 1.2·T/(K·L)`, `Ti = 2L`, `Td = 0.5L` |
| Conditional integration (anti-windup) | Inhibit only accumulation driving *further* into saturation; inhibiting both directions freezes the integrator |

---

## 8. Sanity checks on a finished change

- [ ] `make -C extras/test test` — both suites pass
- [ ] `arduino-lint --compliance strict --library-manager update` — clean
- [ ] All three examples compile warning-free on AVR **and** ESP32
- [ ] `library.properties` version bumped, one reason per bump
- [ ] `CHANGELOG.md` entry added, saying what was wrong and how it showed
- [ ] `keywords.txt` updated if the public API changed (single tabs)
- [ ] Public signatures and enumerator values unchanged, or the bump reflects it
- [ ] A test exists that fails without the fix
- [ ] **No tag pushed** unless the change is merged and you intend to publish
      it to every Library Manager user
