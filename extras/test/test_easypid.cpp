// Host-side behavioural tests for easyPID.
// Compiled against the mock Arduino.h with a virtual clock.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "Arduino.h"
#include "easyPID.h"
#include "PIDTuner.h"

unsigned long g_mockMillis = 0;

static int g_pass = 0;
static int g_fail = 0;
static std::vector<std::string> g_failures;

static void check(bool cond, const std::string& name, const std::string& detail = "") {
    if (cond) {
        g_pass++;
        printf("  PASS  %s\n", name.c_str());
    } else {
        g_fail++;
        g_failures.push_back(name + (detail.empty() ? "" : "  [" + detail + "]"));
        printf("  FAIL  %s   %s\n", name.c_str(), detail.c_str());
    }
}

// ---------------------------------------------------------------------------
// A first-order-plus-dead-time plant. Dead time is what makes a relay loop
// actually oscillate with a finite period, so the autotuner has something real
// to measure.
// ---------------------------------------------------------------------------
class Plant {
public:
    Plant(float gain, float tau, int deadSteps)
        : gain_(gain), tau_(tau), y_(0.0f), delay_(deadSteps > 0 ? deadSteps : 1, 0.0f), idx_(0) {}

    float step(float u, float dt) {
        float delayed = delay_[idx_];
        delay_[idx_] = u;
        idx_ = (idx_ + 1) % delay_.size();
        float alpha = dt / (tau_ + dt);
        y_ = y_ + (gain_ * delayed - y_) * alpha;
        return y_;
    }
    float value() const { return y_; }
    void reset() {
        y_ = 0.0f;
        for (size_t i = 0; i < delay_.size(); i++) delay_[i] = 0.0f;
        idx_ = 0;
    }

private:
    float gain_, tau_, y_;
    std::vector<float> delay_;
    size_t idx_;
};

// ---------------------------------------------------------------------------
static void test_autotuner_completes() {
    printf("\n[1] Autotuner reaches TUNER_COMPLETE on an oscillating plant\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.begin();
    PIDTuner tuner(pid);

    Plant plant(2.0f, 0.5f, 6);          // gain 2, tau 0.5s, 0.6s dead time
    const float setpoint = 50.0f;
    const unsigned long dtMs = 100;
    const float dt = dtMs / 1000.0f;

    bool started = tuner.start(setpoint, 40.0f, 0.5f);
    check(started, "tuner.start() returns true");

    // Bias the plant near the setpoint so the relay swings around it.
    for (int i = 0; i < 200; i++) plant.step(setpoint / 2.0f, dt);

    int steps = 0;
    const int maxSteps = 4000;           // 400 simulated seconds
    while (!tuner.isComplete() && steps < maxSteps) {
        float u = tuner.update(plant.value());
        plant.step(u + setpoint / 2.0f, dt);
        mockAdvance(dtMs);
        steps++;
    }

    check(tuner.isComplete(), "tuner completes within 400 simulated seconds",
          "state=" + std::to_string((int)tuner.getState()) + " steps=" + std::to_string(steps));
    check(tuner.getState() == TUNER_COMPLETE, "final state is TUNER_COMPLETE");

    float ku = tuner.getUltimateGain();
    float pu = tuner.getUltimatePeriod();
    printf("        Ku=%.4f  Pu=%.4f s  (after %d steps)\n", ku, pu, steps);
    check(std::isfinite(ku) && ku > 0.0f, "Ku is finite and positive", "Ku=" + std::to_string(ku));
    check(std::isfinite(pu) && pu > 0.0f, "Pu is finite and positive", "Pu=" + std::to_string(pu));
    // With 0.6 s dead time the limit cycle period should be a few seconds, not
    // milliseconds and not minutes.
    check(pu > 0.5f && pu < 30.0f, "Pu is physically plausible", "Pu=" + std::to_string(pu));

    float kp, ki, kd;
    bool got = tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);
    check(got, "getTunings() succeeds after completion");
    check(std::isfinite(kp) && std::isfinite(ki) && std::isfinite(kd), "Z-N gains are finite");
    check(kp > 0.0f && ki > 0.0f && kd > 0.0f, "Z-N gains are positive");
    printf("        Z-N: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
}

// ---------------------------------------------------------------------------
static void test_tuner_progress_advances() {
    printf("\n[2] getProgress() actually advances (cycle counter increments)\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.begin();
    PIDTuner tuner(pid);
    Plant plant(2.0f, 0.5f, 6);
    const float setpoint = 50.0f;
    const float dt = 0.1f;

    tuner.start(setpoint, 40.0f, 0.5f);
    for (int i = 0; i < 200; i++) plant.step(setpoint / 2.0f, dt);

    float maxProgress = 0.0f;
    for (int i = 0; i < 2000 && !tuner.isComplete(); i++) {
        float u = tuner.update(plant.value());
        plant.step(u + setpoint / 2.0f, dt);
        mockAdvance(100);
        if (tuner.getProgress() > maxProgress) maxProgress = tuner.getProgress();
    }
    printf("        max progress observed: %.3f\n", maxProgress);
    check(maxProgress > 0.0f, "progress rises above 0", "max=" + std::to_string(maxProgress));
}

// ---------------------------------------------------------------------------
static void test_two_tuners_independent() {
    printf("\n[3] Two PIDTuner instances do not share state\n");
    mockSetMillis(1000);

    PIDController pidA(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    PIDController pidB(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pidA.begin();
    pidB.begin();
    PIDTuner tA(pidA), tB(pidB);

    // Deliberately different dynamics -> different Ku/Pu if truly independent.
    Plant pA(2.0f, 0.5f, 6);
    Plant pB(2.0f, 1.5f, 18);
    const float sp = 50.0f;
    const float dt = 0.1f;

    tA.start(sp, 40.0f, 0.5f);
    tB.start(sp, 40.0f, 0.5f);
    for (int i = 0; i < 400; i++) { pA.step(sp / 2.0f, dt); pB.step(sp / 2.0f, dt); }

    for (int i = 0; i < 6000 && !(tA.isComplete() && tB.isComplete()); i++) {
        if (!tA.isComplete()) pA.step(tA.update(pA.value()) + sp / 2.0f, dt);
        if (!tB.isComplete()) pB.step(tB.update(pB.value()) + sp / 2.0f, dt);
        mockAdvance(100);
    }

    check(tA.isComplete(), "tuner A completes");
    check(tB.isComplete(), "tuner B completes");
    printf("        A: Ku=%.4f Pu=%.4f    B: Ku=%.4f Pu=%.4f\n",
           tA.getUltimateGain(), tA.getUltimatePeriod(),
           tB.getUltimateGain(), tB.getUltimatePeriod());
    // The slower plant must show a longer limit-cycle period.
    check(tB.getUltimatePeriod() > tA.getUltimatePeriod(),
          "slower plant yields longer Pu (state is per-instance)",
          "PuA=" + std::to_string(tA.getUltimatePeriod()) + " PuB=" + std::to_string(tB.getUltimatePeriod()));
}

// ---------------------------------------------------------------------------
static void test_tuner_timeout() {
    printf("\n[4] Autotuner gives up on a plant that never oscillates\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.begin();
    PIDTuner tuner(pid);

    tuner.start(50.0f, 40.0f, 0.5f);
    // Measurement pinned far below setpoint: relay never switches.
    for (int i = 0; i < 5000; i++) {
        tuner.update(0.0f);
        mockAdvance(100);
        if (tuner.getState() == TUNER_IDLE) break;
    }
    check(tuner.getState() == TUNER_IDLE, "tuner times out back to TUNER_IDLE",
          "state=" + std::to_string((int)tuner.getState()));
    check(!tuner.isComplete(), "isComplete() stays false after timeout");
}

// ---------------------------------------------------------------------------
// A slow thermal-style process can easily have a limit-cycle period longer
// than the 60 s stall timeout. Keying the timeout to the last relay EDGE
// rather than the last completed CYCLE is what makes such a process tunable,
// because the relay switches twice per period.
// ---------------------------------------------------------------------------
static void test_slow_process_not_killed_by_timeout() {
    printf("\n[4b] Slow process with period > 60 s still tunes\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.begin();
    PIDTuner tuner(pid);

    // tau = 12 s, dead time = 27 s  ->  limit-cycle period of roughly 90 s,
    // comfortably above the 60 s stall timeout but under 2x it.
    Plant plant(2.0f, 12.0f, 270);
    const float sp = 50.0f;
    const float dt = 0.1f;

    tuner.start(sp, 40.0f, 0.5f);
    for (int i = 0; i < 3000; i++) plant.step(sp / 2.0f, dt);

    int steps = 0;
    while (!tuner.isComplete() && tuner.getState() == TUNER_RELAY_STEP && steps < 100000) {
        plant.step(tuner.update(plant.value()) + sp / 2.0f, dt);
        mockAdvance(100);
        steps++;
    }
    printf("        state=%d  Pu=%.2f s  after %d steps (%.0f s simulated)\n",
           (int)tuner.getState(), tuner.getUltimatePeriod(), steps, steps * 0.1);
    check(tuner.isComplete(), "slow process completes rather than timing out",
          "state=" + std::to_string((int)tuner.getState()));
    check(tuner.getUltimatePeriod() > 60.0f,
          "measured Pu really is longer than the stall timeout",
          "Pu=" + std::to_string(tuner.getUltimatePeriod()));
}

// ---------------------------------------------------------------------------
static void test_no_derivative_kick_on_first_update() {
    printf("\n[5] No derivative spike on the first update after begin()/reset()\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 10.0f, -1e6f, 1e6f);
    pid.begin();
    mockAdvance(100);
    pid.update(100.0f, 0.0f);            // step from 0 error to error=100
    printf("        first-update D term = %.4f\n", pid.getDterm());
    check(std::fabs(pid.getDterm()) < 1e-6f, "D term is zero on first update",
          "D=" + std::to_string(pid.getDterm()));

    // And after an explicit reset the same must hold.
    for (int i = 0; i < 10; i++) { mockAdvance(100); pid.update(100.0f, 50.0f); }
    pid.reset();
    mockAdvance(100);
    pid.update(100.0f, 0.0f);
    check(std::fabs(pid.getDterm()) < 1e-6f, "D term is zero on first update after reset()",
          "D=" + std::to_string(pid.getDterm()));
}

// ---------------------------------------------------------------------------
static void test_backcalc_zero_ki() {
    printf("\n[6] BACKCALC anti-windup with ki = 0 does not produce NaN/Inf\n");
    mockSetMillis(1000);

    PIDController pid(10.0f, 0.0f, 0.0f, 0.0f, 255.0f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_BACKCALC);

    bool allFinite = true;
    for (int i = 0; i < 50; i++) {
        mockAdvance(100);
        float out = pid.update(1000.0f, 0.0f);   // hard saturation
        if (!std::isfinite(out) || !std::isfinite(pid.getIterm())) allFinite = false;
    }
    check(allFinite, "output and I term stay finite with ki == 0");
    check(pid.getOutput() >= 0.0f && pid.getOutput() <= 255.0f, "output stays within limits");
}

// ---------------------------------------------------------------------------
static void test_antiwindup_clamp() {
    printf("\n[7] CLAMP anti-windup bounds the integral and still unwinds\n");
    mockSetMillis(1000);

    PIDController noAw(1.0f, 5.0f, 0.0f, 0.0f, 100.0f);
    PIDController clamped(1.0f, 5.0f, 0.0f, 0.0f, 100.0f);
    noAw.begin();
    clamped.begin();
    noAw.setAntiWindup(ANTIWINDUP_NONE);
    clamped.setAntiWindup(ANTIWINDUP_CLAMP);

    // Drive both hard into saturation for a long time.
    for (int i = 0; i < 200; i++) {
        mockAdvance(100);
        noAw.update(500.0f, 0.0f);
        clamped.update(500.0f, 0.0f);
    }
    printf("        I term  none=%.2f  clamp=%.2f\n", noAw.getIterm(), clamped.getIterm());
    check(std::fabs(clamped.getIterm()) < std::fabs(noAw.getIterm()),
          "CLAMP holds the integral below the unprotected case");

    // Now reverse the error; the clamped controller must come off the rail
    // quickly rather than staying saturated for hundreds of cycles.
    int cyclesToLeaveRail = -1;
    for (int i = 0; i < 500; i++) {
        mockAdvance(100);
        float out = clamped.update(0.0f, 500.0f);
        if (out <= 0.0f + 1e-6f) { cyclesToLeaveRail = i; break; }
    }
    printf("        cycles to leave the rail after error reversal: %d\n", cyclesToLeaveRail);
    check(cyclesToLeaveRail >= 0 && cyclesToLeaveRail < 20,
          "integral unwinds promptly once the error reverses",
          "cycles=" + std::to_string(cyclesToLeaveRail));
}

// ---------------------------------------------------------------------------
static void test_error_sign_and_direction() {
    printf("\n[8] getError() reports setpoint - measurement in both directions\n");
    mockSetMillis(1000);

    PIDController d(1.0f, 0.0f, 0.0f, -255.0f, 255.0f);
    d.begin();
    mockAdvance(100);
    d.update(100.0f, 40.0f);
    check(std::fabs(d.getError() - 60.0f) < 1e-3f, "DIRECT: getError() == +60",
          "err=" + std::to_string(d.getError()));

    PIDController r(1.0f, 0.0f, 0.0f, -255.0f, 255.0f);
    r.begin();
    r.setDirection(REVERSE);
    mockAdvance(100);
    float out = r.update(100.0f, 40.0f);
    check(std::fabs(r.getError() - 60.0f) < 1e-3f,
          "REVERSE: getError() is still setpoint - measurement (+60)",
          "err=" + std::to_string(r.getError()));
    check(out < 0.0f, "REVERSE: control action is inverted", "out=" + std::to_string(out));
}

// ---------------------------------------------------------------------------
static void test_manual_dt_then_auto() {
    printf("\n[9] Mixing manual-dt and auto-timing updates keeps dt sane\n");
    mockSetMillis(1000);

    PIDController pid(0.0f, 1.0f, 0.0f, -1e9f, 1e9f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_NONE);

    // 10 manual updates while the virtual clock runs far ahead.
    for (int i = 0; i < 10; i++) {
        mockAdvance(1000);
        pid.update(1.0f, 0.0f, 100.0f);       // claim 100 ms each
    }
    float afterManual = pid.getIterm();

    // One auto update: must integrate roughly one sample, not the 10 s of
    // wall-clock that elapsed during the manual phase.
    mockAdvance(100);
    pid.update(1.0f, 0.0f);
    float delta = pid.getIterm() - afterManual;
    printf("        I after manual=%.3f, delta on first auto update=%.3f\n", afterManual, delta);
    check(delta < 0.5f, "auto update after manual updates integrates ~0.1 s, not ~10 s",
          "delta=" + std::to_string(delta));
}

// ---------------------------------------------------------------------------
static void test_closed_loop_converges() {
    printf("\n[10] Closed loop reaches setpoint (end-to-end sanity)\n");
    mockSetMillis(1000);

    PIDController pid(2.0f, 1.0f, 0.05f, 0.0f, 255.0f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_CLAMP);

    Plant plant(0.4f, 0.3f, 1);
    const float sp = 100.0f;
    const float dt = 0.1f;
    for (int i = 0; i < 2000; i++) {
        mockAdvance(100);
        float u = pid.update(sp, plant.value());
        plant.step(u, dt);
    }
    printf("        final measurement = %.3f (setpoint %.1f)\n", plant.value(), sp);
    check(std::fabs(plant.value() - sp) < 2.0f, "settles within 2 units of setpoint",
          "y=" + std::to_string(plant.value()));
    check(std::isfinite(pid.getIterm()), "integral stays finite");
}

// ---------------------------------------------------------------------------
static void test_introspection_matches_output() {
    printf("\n[11] P+I+D introspection reconstructs the pre-clamp output\n");
    mockSetMillis(1000);

    PIDController pid(2.0f, 3.0f, 0.5f, 0.0f, 255.0f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_CLAMP);

    Plant plant(0.5f, 0.4f, 2);
    bool consistent = true;
    float worst = 0.0f;
    for (int i = 0; i < 400; i++) {
        mockAdvance(100);
        float out = pid.update(100.0f, plant.value());
        plant.step(out, 0.1f);
        float sum = pid.getPterm() + pid.getIterm() + pid.getDterm();
        // The sum may exceed the clamp, but where the output is NOT saturated
        // the two must agree.
        if (out > 0.001f && out < 254.999f) {
            float d = std::fabs(sum - out);
            if (d > worst) worst = d;
            if (d > 1e-3f) consistent = false;
        }
    }
    printf("        worst |P+I+D - output| while unsaturated: %.6f\n", worst);
    check(consistent, "P+I+D equals output when unsaturated",
          "worst=" + std::to_string(worst));
}

// ---------------------------------------------------------------------------
static void test_iterm_reflects_antiwindup() {
    printf("\n[12] getIterm() reflects the anti-windup correction\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 5.0f, 0.0f, 0.0f, 100.0f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_CLAMP);

    // Saturate hard, then watch the reported I term across successive cycles.
    for (int i = 0; i < 20; i++) { mockAdvance(100); pid.update(500.0f, 0.0f); }
    float a = pid.getIterm();
    for (int i = 0; i < 20; i++) { mockAdvance(100); pid.update(500.0f, 0.0f); }
    float b = pid.getIterm();

    printf("        I term after 20 saturated cycles = %.3f, after 40 = %.3f\n", a, b);
    check(std::fabs(b - a) < 1e-3f,
          "reported I term stops climbing while CLAMP holds the integrator",
          "a=" + std::to_string(a) + " b=" + std::to_string(b));
}

// ---------------------------------------------------------------------------
// Conditional integration must only inhibit accumulation that drives FURTHER
// into saturation. If it also reverts accumulation that would relieve
// saturation, the integrator freezes and the output can stay pinned forever.
// ---------------------------------------------------------------------------
static void test_clamp_does_not_lock_output() {
    printf("\n[13] CLAMP does not lock the output at a rail\n");
    mockSetMillis(1000);

    // Wide ceiling during wind-up so the integrator can actually accumulate.
    PIDController pid(1.0f, 5.0f, 0.0f, 0.0f, 1.0e6f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_CLAMP);

    // Wind the integrator up until the I term DOMINATES the P term. That is
    // the condition for lock-in: with |I| > |P| the raw output stays above the
    // ceiling even when the error has reversed, so a non-directional rollback
    // reverts the very accumulation that would have freed it.
    for (int i = 0; i < 10; i++) { mockAdvance(100); pid.update(60.0f, 0.0f); }
    printf("        after wind-up: I=%.2f out=%.2f\n", pid.getIterm(), pid.getOutput());

    // Now shrink the output ceiling so the accumulated integral alone
    // saturates the controller, and apply a sustained NEGATIVE error that
    // should unwind it.
    pid.setOutputLimits(0.0f, 50.0f);

    float firstOut = 0.0f, lastOut = 0.0f;
    for (int i = 0; i < 300; i++) {
        mockAdvance(100);
        lastOut = pid.update(0.0f, 100.0f);   // error = -100, sustained
        if (i == 0) firstOut = lastOut;
    }
    printf("        after 300 samples of error=-100: out=%.3f  I=%.3f\n",
           lastOut, pid.getIterm());
    check(lastOut < 50.0f - 1e-3f,
          "output comes off the ceiling under a sustained opposing error",
          "first=" + std::to_string(firstOut) + " last=" + std::to_string(lastOut));
    check(pid.getIterm() < 290.0f,
          "integrator actually unwinds rather than freezing",
          "I=" + std::to_string(pid.getIterm()));
}

// ---------------------------------------------------------------------------
// When millis() has not advanced, no time has passed. Substituting a whole
// sampleTime_ credits integration that never happened -- and README shows
// update() called unconditionally from loop(), which on a fast board runs
// many times per millisecond.
// ---------------------------------------------------------------------------
static void test_no_time_no_integration() {
    printf("\n[14] A call with zero elapsed time must not integrate\n");
    mockSetMillis(1000);

    PIDController pid(0.0f, 1.0f, 0.0f, -1e6f, 1e6f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_NONE);

    // Ten calls, clock frozen: exactly what a tight loop() does between ticks.
    for (int i = 0; i < 10; i++) pid.update(1.0f, 0.0f);
    printf("        I term after 10 calls with the clock frozen: %.6f\n", pid.getIterm());
    check(std::fabs(pid.getIterm()) < 1e-6f,
          "no integration accrues while the clock is frozen",
          "I=" + std::to_string(pid.getIterm()));

    // A real 100 ms tick must still integrate normally.
    mockAdvance(100);
    pid.update(1.0f, 0.0f);
    printf("        I term after one real 100 ms tick: %.6f\n", pid.getIterm());
    check(std::fabs(pid.getIterm() - 0.1f) < 1e-4f,
          "a genuine 100 ms tick integrates 0.1",
          "I=" + std::to_string(pid.getIterm()));
}

// ---------------------------------------------------------------------------
// The zero-elapsed early return must not swallow the first-sample flag, or the
// derivative kick returns the moment a fast loop is used.
// ---------------------------------------------------------------------------
static void test_first_sample_flag_survives_zero_dt() {
    printf("\n[15] Zero-elapsed call does not consume the first-sample flag\n");
    mockSetMillis(1000);

    PIDController pid(1.0f, 0.0f, 10.0f, -1e6f, 1e6f);
    pid.begin();

    pid.update(100.0f, 0.0f);          // zero elapsed: should be a no-op
    mockAdvance(100);
    pid.update(100.0f, 0.0f);          // this is the real first sample
    printf("        D term on the first real sample = %.4f\n", pid.getDterm());
    check(std::fabs(pid.getDterm()) < 1e-6f,
          "first real sample still has no derivative kick",
          "D=" + std::to_string(pid.getDterm()));
}

// ---------------------------------------------------------------------------
int main() {
    printf("=========== easyPID host test suite ===========\n");
    test_autotuner_completes();
    test_tuner_progress_advances();
    test_two_tuners_independent();
    test_tuner_timeout();
    test_slow_process_not_killed_by_timeout();
    test_no_derivative_kick_on_first_update();
    test_backcalc_zero_ki();
    test_antiwindup_clamp();
    test_error_sign_and_direction();
    test_manual_dt_then_auto();
    test_closed_loop_converges();
    test_introspection_matches_output();
    test_iterm_reflects_antiwindup();
    test_clamp_does_not_lock_output();
    test_no_time_no_integration();
    test_first_sample_flag_survives_zero_dt();

    printf("\n===============================================\n");
    printf("PASS: %d   FAIL: %d\n", g_pass, g_fail);
    if (!g_failures.empty()) {
        printf("\nFailures:\n");
        for (size_t i = 0; i < g_failures.size(); i++) printf("  - %s\n", g_failures[i].c_str());
    }
    return g_fail == 0 ? 0 : 1;
}
