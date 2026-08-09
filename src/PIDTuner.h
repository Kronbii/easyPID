/**
 * @file PIDTuner.h
 * @brief Optional PID Autotuner Module (Relay/Limit-Cycle Method)
 * @author Rami Kronbi
 * @date 2024
 * 
 * This is an OPTIONAL add-on module for automatic PID tuning.
 * Users must explicitly include <PIDTuner.h> to use autotuning.
 * 
 * The autotuner uses relay feedback (bang-bang control) to induce
 * sustained oscillations, then applies classical tuning rules to
 * calculate PID parameters.
 * 
 * Supported tuning rules:
 * - Ziegler-Nichols (classic, aggressive)
 * - Tyreus-Luyben (less overshoot, better for lag-dominant processes)
 * - Pessen Integral Rule (faster response, some overshoot)
 * - No Overshoot (conservative tuning)
 * 
 * @warning This module induces oscillations in your system. Use with caution
 *          and ensure your system can safely handle oscillations around the setpoint.
 */

#pragma once

#include <Arduino.h>
#include "easyPID.h"

/**
 * @enum TuningRule
 * @brief Available tuning rule methods
 */
enum TuningRule {
    TUNING_ZIEGLER_NICHOLS,   ///< Classic Z-N: aggressive, may overshoot
    TUNING_TYREUS_LUYBEN,     ///< T-L: less overshoot, slower response
    TUNING_PESSEN,            ///< Pessen: fast response, moderate overshoot
    TUNING_NO_OVERSHOOT       ///< Conservative: minimal overshoot, slower
};

/**
 * @enum TunerState
 * @brief Autotuner state machine states
 */
enum TunerState {
    TUNER_IDLE,           ///< Not running, and not yet attempted
    TUNER_RELAY_STEP,     ///< Applying relay feedback
    TUNER_ANALYZING,      ///< Computing tuning parameters (transient, within one update() call)
    TUNER_COMPLETE,       ///< Tuning complete, results ready
    TUNER_FAILED          ///< A run was attempted and produced no usable result
    // TUNER_FAILED is appended rather than inserted so the numeric values of
    // the existing enumerators are unchanged for code that compares them.
};

/**
 * @class PIDTuner
 * @brief Automatic PID tuner using relay feedback method
 * 
 * This class implements the relay/limit-cycle autotuning method
 * popularized by Åström and Hägglund. It works by:
 * 
 * 1. Applying relay (bang-bang) control around setpoint
 * 2. Measuring resulting oscillation period (Pu) and amplitude (a)
 * 3. Calculating ultimate gain: Ku = 4*d / (pi * sqrt(a^2 - h^2))
 *    where d = relay amplitude, a = half peak-to-peak oscillation amplitude,
 *    and h = the noise band acting as relay hysteresis
 * 4. Applying tuning rules to get Kp, Ki, Kd
 *
 * Usage:
 * @code
 * PIDController pid(1.0, 0.0, 0.0, 0, 255);
 * PIDTuner tuner(pid);
 *
 * // In setup. The fourth argument centres the relay on an operating point,
 * // which a unipolar actuator (0..255) requires; omit it for a bipolar one.
 * tuner.start(setpoint, relayAmplitude, noiseBand, 127.0);
 *
 * // In loop:
 * if (tuner.getState() == TUNER_RELAY_STEP) {
 *   applyOutput(tuner.update(measurement));
 * } else if (tuner.isComplete()) {
 *   tuner.applyTunings(TUNING_ZIEGLER_NICHOLS);   // straight onto pid
 *   applyOutput(pid.update(setpoint, measurement));
 * } else if (tuner.getState() == TUNER_FAILED) {
 *   // no usable limit cycle was measured
 * }
 * @endcode
 */
class PIDTuner {
public:
    /**
     * @brief Construct autotuner for a PID controller
     * @param pid Reference to PIDController to be tuned
     */
    PIDTuner(PIDController& pid);

    /**
     * @brief Start autotuning process
     * @param setpoint Target value for tuning
     * @param relayAmplitude Amplitude of relay output (higher = more aggressive)
     * @param noiseBand Noise band around setpoint (ignore small oscillations)
     * @param outputBias Operating point the relay swings around. The tuner
     *        output ranges over [outputBias - relayAmplitude,
     *        outputBias + relayAmplitude]. Defaults to 0, giving the symmetric
     *        +/-relayAmplitude swing of earlier versions, which only suits a
     *        bipolar actuator. For a unipolar one -- a heater, a PWM pin,
     *        anything on 0..255 -- set a bias that holds the process near the
     *        setpoint. Without it half of every relay period is a negative
     *        drive the hardware clips to zero, the measurement never crosses
     *        the setpoint, and tuning times out with no result.
     * @return true if started successfully; false if a run is already in
     *         progress, or if relayAmplitude <= 0 or noiseBand < 0
     * @note Typical relayAmplitude: 10-20% of full output range
     * @note Typical noiseBand: 1-5% of setpoint value
     * @note Calls reset() on the attached PIDController. The relay run drives
     *       the plant directly, so any integral the controller had accumulated
     *       is stale once tuning completes.
     */
    bool start(float setpoint, float relayAmplitude, float noiseBand = 0.5f,
               float outputBias = 0.0f);

    /**
     * @brief Update autotuner (call in loop during tuning)
     * @param measurement Current process variable
     * @return Relay output to apply to the system while tuning, in the range
     *         [outputBias - relayAmplitude, outputBias + relayAmplitude].
     *         Returns 0 when the tuner is not in TUNER_RELAY_STEP, so check
     *         getState() before treating the value as a drive level.
     */
    float update(float measurement);

    /**
     * @brief Check if tuning is complete
     * @return true if tuning finished successfully
     */
    bool isComplete() const;

    /**
     * @brief Get computed tuning parameters
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     * @param kd Output: derivative gain
     * @param rule Tuning rule to apply (default: Ziegler-Nichols)
     * @return true if parameters available
     */
    bool getTunings(float& kp, float& ki, float& kd, TuningRule rule = TUNING_ZIEGLER_NICHOLS) const;

    /**
     * @brief Apply the computed tunings directly to the attached controller
     * @param rule Tuning rule to apply (default: Ziegler-Nichols)
     * @return true if gains were available and applied, false otherwise
     * @note Also calls reset() on the controller, since gains changed under it
     *       and the carried integral was accumulated with the old ones.
     */
    bool applyTunings(TuningRule rule = TUNING_ZIEGLER_NICHOLS);

    /**
     * @brief Get ultimate gain (Ku) found during tuning
     * @return Ultimate gain value (0 if not available)
     */
    float getUltimateGain() const;

    /**
     * @brief Get ultimate period (Pu) found during tuning
     * @return Ultimate period in seconds (0 if not available)
     */
    float getUltimatePeriod() const;

    /**
     * @brief Cancel ongoing tuning
     */
    void cancel();

    /**
     * @brief Get current tuner state
     * @return Current state
     */
    TunerState getState() const;

    /**
     * @brief Get progress indication (0.0 to 1.0)
     * @return Fraction of the required cycles collected while tuning, 1.0 once
     *         tuning has completed, 0.0 when idle or failed
     */
    float getProgress() const;

private:
    PIDController& pid_;
    TunerState state_;
    
    // Tuning parameters
    float setpoint_;
    float relayAmplitude_;
    float noiseBand_;
    
    // Relay state
    bool relayHigh_;
    bool relayHighPrev_;   ///< Relay state at the previous update, for edge detection
    float outputHigh_;
    float outputLow_;
    float outputBias_;     ///< Operating point the relay swings around
    
    // Limit-cycle measurement: running extremes of the cycle in progress.
    // The process peak lags the relay switch, so the extremes have to be
    // accumulated continuously rather than sampled at the switching instant.
    float cycleMax_;
    float cycleMin_;
    bool haveCycleStart_;           ///< True once the first rising edge has been seen
    unsigned long cycleStartTime_;  ///< millis() at the last rising edge
    unsigned long tuningStartTime_;     ///< millis() at start(), for the absolute deadline
    unsigned long lastTransitionTime_;  ///< millis() at the last relay edge, either direction

    // Cycle detection
    int cyclesDetected_;
    int cyclesNeeded_;
    float periodSum_;
    float amplitudeSum_;
    
    // Results
    float ultimateGain_;      // Ku
    float ultimatePeriod_;    // Pu in seconds
    bool resultsValid_;
    
    // Helpers
    void trackLimitCycle(float measurement, unsigned long now);
    void calculateResults();
    void applyTuningRule(float& kp, float& ki, float& kd, TuningRule rule) const;
};
