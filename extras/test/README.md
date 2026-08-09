# Host-side tests

These run the library on your development machine, not on a board. The Arduino
library specification ignores everything under `extras/`, so none of this is
compiled into a sketch or seen by the IDE.

```bash
cd extras/test
make            # builds and runs both suites
make clean
```

Requires only a C++11 compiler. No Arduino toolchain, no hardware.

## How it works

`mock/Arduino.h` stands in for the real header and provides a **virtual clock**:
`millis()` reads a counter the test advances explicitly with `mockAdvance(ms)`.
That makes every timing-dependent behaviour reproducible — anti-windup, the
derivative, the autotuner's period measurement — none of which can be tested
reliably against wall-clock time.

The plants are first-order-plus-dead-time models. The dead time matters: a relay
loop needs phase lag to oscillate, so an autotuner test against a pure lag would
either not oscillate or oscillate at the sampling limit and prove nothing.

## The two suites

**`test_easypid.cpp`** — library behaviour. Each test corresponds to a defect
that shipped in 1.0.0, so a regression fails loudly rather than silently
degrading control quality:

- the autotuner reaches `TUNER_COMPLETE` and reports a plausible `Ku`/`Pu`
- a slower plant yields a longer `Pu` (proves the measurement tracks dynamics
  rather than returning an artefact, and that tuner state is per-instance)
- a plant with a period longer than the stall timeout still tunes
- a plant that never oscillates times out instead of relaying forever
- no derivative kick on the first sample after `begin()`/`reset()`
- `BACKCALC` with `Ki = 0` does not produce inf/NaN
- `CLAMP` bounds the integral **and** lets it unwind — it must not pin the
  output at a rail under a sustained opposing error
- `getError()` is `setpoint - measurement` in both directions
- mixing the manual-`dt` and automatic overloads keeps `dt` sane
- a call with zero elapsed time integrates nothing
- `P + I + D` reconstructs the output while unsaturated

**`test_examples.cpp`** — the constants in the example sketches. It checks each
setpoint sits below its plant's ceiling (`GAIN * 100`), that each loop settles,
and that AutoTunePID's relay actually induces a well-sampled limit cycle.

Two of the three sketches once chased setpoints their plants could never reach,
and the autotune example could not oscillate at all. Neither was visible to
inspection or to `arduino-lint`.

> The example constants are **mirrored** here, not parsed from the `.ino` files.
> If you change a sketch, change the matching constant in `test_examples.cpp`.
> The duplication is deliberate: it makes drift fail the build.

## Adding a test

Fixing a bug? Add a case that fails before the fix and passes after, and say in
a comment what the wrong behaviour was. Every test here was written that way,
and several caught mistakes that reasoning alone had cleared as fine.
