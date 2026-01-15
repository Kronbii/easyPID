/**
 * @file PIDTuner.h
 * @brief Optional PID Autotuner Module (Relay/Limit-Cycle Method)
 * @author Rami Kronbi
 * @date 2024
 * @version 1.0.0
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
#include "PIDController.h"

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
    TUNER_IDLE,           ///< Not running
    TUNER_RELAY_STEP,     ///< Applying relay feedback
    TUNER_ANALYZING,      ///< Computing tuning parameters
    TUNER_COMPLETE        ///< Tuning complete, results ready
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
 * 3. Calculating ultimate gain: Ku = 4*d / (π*a)
 *    where d = relay amplitude
 * 4. Applying tuning rules to get Kp, Ki, Kd
 * 
 * Usage:
 * @code
 * PIDController pid(1.0, 0.0, 0.0, 0, 255);
 * PIDTuner tuner(pid);
 * 
 * // In setup:
 * tuner.start(setpoint, relayAmplitude);
 * 
 * // In loop:
 * float output = tuner.update(measurement);
 * applyOutput(output);
 * 
 * if (tuner.isComplete()) {
 *   float kp, ki, kd;
 *   tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);
 *   pid.setTunings(kp, ki, kd);
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
     * @return true if started successfully
     * @note Typical relayAmplitude: 10-20% of full output range
     * @note Typical noiseBand: 1-5% of setpoint value
     */
    bool start(float setpoint, float relayAmplitude, float noiseBand = 0.5f);

    /**
     * @brief Update autotuner (call in loop during tuning)
     * @param measurement Current process variable
     * @return Control output to apply to system
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
    bool getTunings(float& kp, float& ki, float& kd, TuningRule rule = TUNING_ZIEGLER_NICHOLS);

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
     * @return Progress value
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
    float outputHigh_;
    float outputLow_;
    
    // Oscillation detection
    float peakHigh_;
    float peakLow_;
    unsigned long peakHighTime_;
    unsigned long peakLowTime_;
    bool lookingForPeak_;
    int peakType_; // 1 = high, -1 = low
    
    // Cycle detection
    int cyclesDetected_;
    int cyclesNeeded_;
    unsigned long lastPeakTime_;
    float periodSum_;
    float amplitudeSum_;
    
    // Results
    float ultimateGain_;      // Ku
    float ultimatePeriod_;    // Pu in seconds
    bool resultsValid_;
    
    // Helpers
    void detectPeak(float measurement, unsigned long now);
    void calculateResults();
    void applyTuningRule(float& kp, float& ki, float& kd, TuningRule rule);
};
