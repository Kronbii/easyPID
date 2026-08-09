/**
 * @file easyPID.h
 * @brief General-purpose PID Controller for Arduino
 * @author Rami Kronbi
 * @date 2024
 * @version 1.0.0
 * 
 * A feature-rich PID controller library with:
 * - Multi-instance support (no global state)
 * - Flexible timing (automatic or manual dt input)
 * - Anti-windup modes (NONE, CLAMP, BACKCALC)
 * - Derivative filtering (NONE, EMA)
 * - Full state introspection for debugging
 * - Runtime tuning changes
 */

#pragma once

#include <Arduino.h>

/**
 * @enum AntiWindupMode
 * @brief Anti-windup strategies to prevent integral term saturation
 */
enum AntiWindupMode {
    ANTIWINDUP_NONE,      ///< No anti-windup (integral can grow unbounded)
    ANTIWINDUP_CLAMP,     ///< Clamp integral when output saturates
    ANTIWINDUP_BACKCALC   ///< Back-calculation method (reduces integral based on saturation)
};

/**
 * @enum DerivativeFilterMode
 * @brief Derivative filtering options to reduce noise sensitivity
 */
enum DerivativeFilterMode {
    FILTER_NONE,    ///< No filtering (raw derivative)
    FILTER_EMA      ///< Exponential Moving Average (1st-order low-pass)
};

/**
 * @enum ControlDirection
 * @brief Control action direction
 */
enum ControlDirection {
    DIRECT,   ///< Increase output when error is positive (heating, etc.)
    REVERSE   ///< Decrease output when error is positive (cooling, etc.)
};

/**
 * @class PIDController
 * @brief Hardware-agnostic PID controller with advanced features
 * 
 * This class implements a discrete-time PID controller based on proven
 * control algorithms. It supports multiple independent instances, flexible
 * timing modes, and advanced features like anti-windup and derivative filtering.
 * 
 * Core PID equation (direct form):
 *   output = Kp*error + Ki*integral(error*dt) + Kd*d(error)/dt
 * 
 * @note All internal calculations use float for AVR compatibility
 * @note Based on proven implementation from light-tracking system
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param outMin Minimum output limit
     * @param outMax Maximum output limit
     */
    PIDController(float kp, float ki, float kd, float outMin, float outMax);

    /**
     * @brief Initialize controller state and timing
     * @note Call this in setup() before first update
     */
    void begin();

    /**
     * @brief Update PID with automatic timing (uses millis internally)
     * @param setpoint Desired target value
     * @param measurement Current process variable
     * @return Calculated control output (clamped to limits)
     * @note If no whole millisecond has elapsed since the last update this
     *       returns the previous output without recomputing, so it is safe to
     *       call from a fast loop(). Introspection getters likewise keep their
     *       previous values on such a call.
     */
    float update(float setpoint, float measurement);

    /**
     * @brief Update PID with manual time delta
     * @param setpoint Desired target value
     * @param measurement Current process variable
     * @param dtMs Time delta in milliseconds since last update
     * @return Calculated control output (clamped to limits)
     * @note Safe to interleave with the automatic-timing overload: this call
     *       also refreshes the internal timing reference
     */
    float update(float setpoint, float measurement, float dtMs);

    /**
     * @brief Set setpoint for alternative usage pattern
     * @param setpoint Desired target value
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Set measurement for alternative usage pattern
     * @param measurement Current process variable
     */
    void setMeasurement(float measurement);

    /**
     * @brief Compute output using stored setpoint/measurement
     * @return Calculated control output
     * @note Use after setSetpoint() and setMeasurement()
     */
    float compute();

    /**
     * @brief Update PID tuning parameters at runtime
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setTunings(float kp, float ki, float kd);

    /**
     * @brief Set output saturation limits
     * @param min Minimum output value
     * @param max Maximum output value
     * @note Ignored if min >= max, which would otherwise mark every output as
     *       saturated and permanently inhibit integration
     */
    void setOutputLimits(float min, float max);

    /**
     * @brief Set integral limits to prevent excessive windup
     * @param min Minimum accumulator value
     * @param max Maximum accumulator value
     * @note These bound the raw error-time accumulator, NOT the Ki-scaled I
     *       term. The resulting contribution to the output is Ki * limit.
     * @note Ignored if min >= max. Until this is called, no integral limit is
     *       applied; the output limits alone do not imply one.
     */
    void setIntegralLimits(float min, float max);

    /**
     * @brief Configure anti-windup strategy
     * @param mode Anti-windup mode (NONE, CLAMP, BACKCALC)
     */
    void setAntiWindup(AntiWindupMode mode);

    /**
     * @brief Configure derivative filtering
     * @param mode Filter mode (NONE, EMA)
     * @param alpha Filter coefficient, higher = more filtering. Clamped to
     *        [0.0, 0.999]: at exactly 1.0 the filtered derivative would be
     *        frozen and the D term permanently dead.
     */
    void setDerivativeFilter(DerivativeFilterMode mode, float alpha = 0.8f);

    /**
     * @brief Record the expected sample period
     * @param ms Sample time in milliseconds; values of 0 are ignored
     * @note Advisory only. It does not gate the update rate and is not used in
     *       the control math: dt comes from the elapsed time measured by
     *       update(setpoint, measurement), or from the dtMs you pass to
     *       update(setpoint, measurement, dtMs). Call update() at your own
     *       cadence.
     */
    void setSampleTime(unsigned long ms);

    /**
     * @brief Set control direction
     * @param direction DIRECT or REVERSE
     * @note Changing direction at runtime negates the carried integral and
     *       derivative state so the switch is bumpless. Setting the direction
     *       it already has is a no-op.
     */
    void setDirection(ControlDirection direction);

    /**
     * @brief Reset controller state (clear integral, derivative, errors)
     * @note Use when changing setpoint dramatically or after pause
     * @note Also restarts the automatic-timing reference, so the next
     *       update(setpoint, measurement) measures dt from now rather than
     *       from before the pause
     */
    void reset();

    /**
     * @brief Get current error value
     * @return Current error (setpoint - measurement)
     * @note Always setpoint - measurement, including in REVERSE mode. The sign
     *       inversion REVERSE applies is internal to the control terms.
     */
    float getError() const;

    /**
     * @brief Get proportional term contribution
     * @return P term value
     */
    float getPterm() const;

    /**
     * @brief Get integral term contribution
     * @return I term value, after any anti-windup correction for this cycle
     */
    float getIterm() const;

    /**
     * @brief Get derivative term contribution
     * @return D term value
     */
    float getDterm() const;

    /**
     * @brief Get last computed output
     * @return Output value
     */
    float getOutput() const;

private:
    // Tuning parameters
    float kp_;
    float ki_;
    float kd_;

    // Output limits
    float outMin_;
    float outMax_;

    // Integral limits (for advanced windup prevention)
    float integralMin_;
    float integralMax_;
    bool integralLimitsSet_;

    // State variables (based on proven tracker.h implementation)
    float error_;         ///< Reported error, always setpoint - measurement
    float controlError_;  ///< Error actually driving the terms (negated when REVERSE)
    float previousError_;
    bool firstUpdate_;   ///< No previous error yet, so no derivative can be formed
    float integral_;
    float derivative_;
    float derivativeFiltered_;

    // Term contributions (for introspection)
    float pTerm_;
    float iTerm_;
    float dTerm_;
    float output_;

    // Timing
    unsigned long lastTime_;
    unsigned long sampleTime_;

    // Configuration
    AntiWindupMode antiWindupMode_;
    DerivativeFilterMode filterMode_;
    float filterAlpha_;
    ControlDirection direction_;

    // Alternative pattern support
    float storedSetpoint_;
    float storedMeasurement_;

    // Initialization flag
    bool initialized_;

    /**
     * @brief Internal computation method
     * @param setpoint Target value
     * @param measurement Current value
     * @param dt Time delta in seconds
     * @return Computed output
     */
    float computePID(float setpoint, float measurement, float dt);

    /**
     * @brief Apply anti-windup logic
     * @param rawOutput Output before clamping
     * @param clampedOutput Output after clamping
     * @param dt Time delta in seconds
     * @param integralBefore Integrator value before this cycle's accumulation
     */
    void applyAntiWindup(float rawOutput, float clampedOutput, float dt, float integralBefore);
};
