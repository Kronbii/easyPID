// Regression guard for the constants baked into the example sketches.
//
// The plant model below mirrors the one every example uses. The constants are
// duplicated from the .ino files rather than parsed out of them, so if you
// change a sketch, change the matching value here too -- that is the point:
// the test fails loudly rather than letting an example drift into a state
// where it cannot reach its own setpoint.
//
// This exists because two of the three sketches once chased setpoints above
// their plant's ceiling, and the autotune sketch could not induce a limit
// cycle at all. Both were invisible to inspection and to arduino-lint.
#include <cmath>
#include <cstdio>
#include <string>

#include "Arduino.h"
#include "easyPID.h"
#include "PIDTuner.h"

unsigned long g_mockMillis = 0;

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const std::string& name, const std::string& detail = "") {
    if (cond) {
        g_pass++;
        printf("  PASS  %s\n", name.c_str());
    } else {
        g_fail++;
        printf("  FAIL  %s   %s\n", name.c_str(), detail.c_str());
    }
}

// measurement += (output * GAIN / 255 * 100 - measurement) * (dt / (TAU + dt))
static float plantStep(float y, float u, float gain, float tau, float dt) {
    float alpha = dt / (tau + dt);
    return y + (u * gain / 255.0f * 100.0f - y) * alpha;
}

// The plant can never exceed GAIN * 100, reached only at full output.
static float plantCeiling(float gain) { return gain * 100.0f; }

static void checkReachable(const char* name, float setpoint, float gain) {
    float ceiling = plantCeiling(gain);
    check(setpoint < ceiling,
          std::string(name) + ": setpoint is below the plant ceiling",
          "setpoint=" + std::to_string(setpoint) + " ceiling=" + std::to_string(ceiling));
}

static void checkSettles(const char* name, float sp, float gain, float tau,
                         float kp, float ki, float kd) {
    mockSetMillis(1000);
    PIDController pid(kp, ki, kd, 0.0f, 255.0f);
    pid.begin();
    pid.setAntiWindup(ANTIWINDUP_CLAMP);
    float y = 0.0f;
    for (int i = 0; i < 3000; i++) {
        mockAdvance(100);
        y = plantStep(y, pid.update(sp, y), gain, tau, 0.1f);
    }
    check(std::fabs(y - sp) < 1.0f,
          std::string(name) + ": closed loop settles on setpoint",
          "final=" + std::to_string(y) + " setpoint=" + std::to_string(sp));
}

int main() {
    printf("=========== easyPID example-constant tests ===========\n");

    // ---- BasicPID.ino ---------------------------------------------------
    printf("\n[BasicPID]\n");
    const float B_SP = 100.0f, B_GAIN = 1.5f, B_TAU = 0.1f;
    checkReachable("BasicPID", B_SP, B_GAIN);
    checkSettles("BasicPID", B_SP, B_GAIN, B_TAU, 2.0f, 0.5f, 0.1f);

    // ---- MultiLoopPID.ino -----------------------------------------------
    printf("\n[MultiLoopPID]\n");
    const float M1_SP = 80.0f,  M1_GAIN = 1.2f, M1_TAU = 0.08f;
    const float M2_SP = 120.0f, M2_GAIN = 1.6f, M2_TAU = 0.25f;
    checkReachable("MultiLoop p1", M1_SP, M1_GAIN);
    checkReachable("MultiLoop p2", M2_SP, M2_GAIN);
    checkSettles("MultiLoop p1", M1_SP, M1_GAIN, M1_TAU, 1.5f, 0.8f, 0.05f);
    checkSettles("MultiLoop p2", M2_SP, M2_GAIN, M2_TAU, 3.0f, 0.3f, 0.2f);

    // ---- AutoTunePID.ino ------------------------------------------------
    printf("\n[AutoTunePID]\n");
    const float A_SP = 37.0f, A_GAIN = 0.75f, A_TAU = 1.0f;
    const float A_RELAY = 50.0f, A_BAND = 5.0f, A_BIAS = 127.0f;
    const float A_DT = 0.1f;
    checkReachable("AutoTunePID", A_SP, A_GAIN);

    mockSetMillis(1000);
    PIDController pid(1.0f, 0.0f, 0.0f, 0.0f, 255.0f);
    pid.begin();
    PIDTuner tuner(pid);
    check(tuner.start(A_SP, A_RELAY, A_BAND), "AutoTunePID: tuner.start() accepted");

    float y = 0.0f;
    int steps = 0;
    while (tuner.getState() == TUNER_RELAY_STEP && steps < 100000) {
        float u = tuner.update(y) + A_BIAS;
        if (u < 0.0f) u = 0.0f;
        if (u > 255.0f) u = 255.0f;
        y = plantStep(y, u, A_GAIN, A_TAU, A_DT);
        mockAdvance(100);
        steps++;
    }
    check(tuner.isComplete(), "AutoTunePID: the relay induces a measurable limit cycle",
          "state=" + std::to_string((int)tuner.getState()));

    float pu = tuner.getUltimatePeriod();
    float samplesPerCycle = pu / A_DT;
    printf("        Ku=%.4f  Pu=%.3f s  (%.1f samples/cycle, tuned in %.1f s)\n",
           tuner.getUltimateGain(), pu, samplesPerCycle, steps * 0.1);

    // A period of only a few samples sits at the sampling limit, where the
    // measurement -- and therefore every derived gain -- is meaningless.
    check(samplesPerCycle >= 10.0f,
          "AutoTunePID: the limit cycle is well sampled (>= 10 samples/cycle)",
          "samples=" + std::to_string(samplesPerCycle));

    // Every rule should produce a loop that reaches setpoint.
    const char* names[] = {"Ziegler-Nichols", "Tyreus-Luyben", "Pessen", "No-Overshoot"};
    TuningRule rules[] = {TUNING_ZIEGLER_NICHOLS, TUNING_TYREUS_LUYBEN,
                          TUNING_PESSEN, TUNING_NO_OVERSHOOT};
    float overshoot[4] = {0, 0, 0, 0};
    for (int r = 0; r < 4; r++) {
        float kp, ki, kd;
        if (!tuner.getTunings(kp, ki, kd, rules[r])) {
            check(false, std::string(names[r]) + ": getTunings() succeeded");
            continue;
        }
        mockSetMillis(1000);
        PIDController run(kp, ki, kd, 0.0f, 255.0f);
        run.begin();
        run.setAntiWindup(ANTIWINDUP_CLAMP);
        float yy = 0.0f, peak = 0.0f;
        for (int i = 0; i < 3000; i++) {
            mockAdvance(100);
            yy = plantStep(yy, run.update(A_SP, yy), A_GAIN, A_TAU, A_DT);
            if (yy > peak) peak = yy;
        }
        overshoot[r] = (peak - A_SP) / A_SP * 100.0f;
        printf("        %-16s Kp=%7.3f Ki=%7.3f Kd=%6.3f -> final=%6.2f overshoot=%5.1f%%\n",
               names[r], kp, ki, kd, yy, overshoot[r]);
        check(std::fabs(yy - A_SP) < 1.0f,
              std::string("AutoTunePID: ") + names[r] + " reaches setpoint",
              "final=" + std::to_string(yy));
    }

    // Sanity check on the rule set itself: the aggressive rules must overshoot
    // more than the conservative ones, or the tuning constants are transposed.
    check(overshoot[2] >= overshoot[0], "Pessen overshoots at least as much as Ziegler-Nichols");
    check(overshoot[0] >= overshoot[3], "Ziegler-Nichols overshoots at least as much as No-Overshoot");

    printf("\n======================================================\n");
    printf("PASS: %d   FAIL: %d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
