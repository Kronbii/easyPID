/**
 * @file PIDController.cpp
 * @brief Implementation of PIDController class
 * @author Rami Kronbi
 * @date 2024
 */

#include "easyPID.h"

// Below this magnitude the integral gain is treated as absent, so
// back-calculation anti-windup has nothing to correct and 1/ki_ is not formed.
static const float EASYPID_MIN_KI = 1e-6f;

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {
    
    // Initialize state variables
    error_ = 0.0f;
    previousError_ = 0.0f;
    firstUpdate_ = true;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    derivativeFiltered_ = 0.0f;
    
    pTerm_ = 0.0f;
    iTerm_ = 0.0f;
    dTerm_ = 0.0f;
    output_ = 0.0f;
    
    // Default configuration
    antiWindupMode_ = ANTIWINDUP_CLAMP;
    filterMode_ = FILTER_NONE;
    filterAlpha_ = 0.8f;
    direction_ = DIRECT;
    
    sampleTime_ = 100; // Default 100ms
    lastTime_ = 0;
    
    // Integral limits default to output limits
    integralMin_ = outMin;
    integralMax_ = outMax;
    integralLimitsSet_ = false;
    
    storedSetpoint_ = 0.0f;
    storedMeasurement_ = 0.0f;
    
    initialized_ = false;
}

void PIDController::begin() {
    lastTime_ = millis();
    initialized_ = true;
    reset();
}

float PIDController::update(float setpoint, float measurement) {
    if (!initialized_) {
        begin();
    }
    
    unsigned long now = millis();
    float dt = (float)(now - lastTime_) / 1000.0f; // Convert to seconds
    lastTime_ = now;
    
    // Ensure minimum dt to avoid division by zero
    if (dt <= 0.0f) {
        dt = (float)sampleTime_ / 1000.0f;
    }
    
    return computePID(setpoint, measurement, dt);
}

float PIDController::update(float setpoint, float measurement, float dtMs) {
    if (!initialized_) {
        begin();
    }

    float dt = dtMs / 1000.0f; // Convert to seconds

    // Ensure minimum dt to avoid division by zero
    if (dt <= 0.0f) {
        dt = (float)sampleTime_ / 1000.0f;
    }

    // Keep the automatic-timing reference in step with manual updates. Without
    // this, a sketch that drives the controller with an explicit dt and then
    // calls the automatic overload would have the whole elapsed wall-clock
    // since the last automatic call counted as one sample period.
    lastTime_ = millis();

    return computePID(setpoint, measurement, dt);
}

void PIDController::setSetpoint(float setpoint) {
    storedSetpoint_ = setpoint;
}

void PIDController::setMeasurement(float measurement) {
    storedMeasurement_ = measurement;
}

float PIDController::compute() {
    return update(storedSetpoint_, storedMeasurement_);
}

float PIDController::computePID(float setpoint, float measurement, float dt) {
    // Calculate error (based on proven tracker.h lines 54-56, 104-105)
    error_ = setpoint - measurement;
    
    // Apply control direction
    if (direction_ == REVERSE) {
        error_ = -error_;
    }
    
    // Calculate proportional term (based on tracker.h line 78)
    pTerm_ = kp_ * error_;
    
    // Calculate and accumulate integral term with dt scaling (based on tracker.h line 75)
    // Original: integral_error_ += current_error_;
    // Enhanced: integral += error * dt (for time-aware integration)
    integral_ += error_ * dt;
    
    // Clamp integral to limits if set
    if (integralLimitsSet_) {
        if (integral_ > integralMax_) {
            integral_ = integralMax_;
        } else if (integral_ < integralMin_) {
            integral_ = integralMin_;
        }
    }
    
    iTerm_ = ki_ * integral_;
    
    // Calculate derivative term with dt scaling (based on tracker.h line 72)
    // Original: derivative_error_ = current_error_ - previous_error_;
    // Enhanced: derivative = (error - previousError) / dt (for time-aware differentiation)
    //
    // On the very first update after begin()/reset() there is no previous error
    // to difference against. Treating the stale 0.0 as a real sample produced a
    // derivative of error/dt, i.e. a large spurious kick exactly when the error
    // is typically at its largest. Start the derivative at zero instead and let
    // it develop from the second sample onward.
    if (firstUpdate_) {
        derivative_ = 0.0f;
        derivativeFiltered_ = 0.0f;
        firstUpdate_ = false;
    } else {
        derivative_ = (error_ - previousError_) / dt;
    }

    // Apply derivative filtering if enabled
    float derivativeToUse = derivative_;
    if (filterMode_ == FILTER_EMA) {
        // Exponential Moving Average (1st order low-pass filter)
        derivativeFiltered_ = filterAlpha_ * derivativeFiltered_ + (1.0f - filterAlpha_) * derivative_;
        derivativeToUse = derivativeFiltered_;
    }
    
    dTerm_ = kd_ * derivativeToUse;
    
    // Calculate total PID output (based on tracker.h lines 78-80)
    // Original: output = (kp_ * current_error_) + (ki_ * integral_error_) + (kd_ * derivative_error_);
    float rawOutput = pTerm_ + iTerm_ + dTerm_;
    
    // Clamp output to limits (based on tracker.h lines 86-90)
    float clampedOutput = rawOutput;
    if (clampedOutput > outMax_) {
        clampedOutput = outMax_;
    } else if (clampedOutput < outMin_) {
        clampedOutput = outMin_;
    }
    
    // Apply anti-windup
    applyAntiWindup(rawOutput, clampedOutput, dt);
    
    // Store for next iteration (based on tracker.h line 83)
    previousError_ = error_;
    
    output_ = clampedOutput;
    return output_;
}

void PIDController::applyAntiWindup(float rawOutput, float clampedOutput, float dt) {
    if (antiWindupMode_ == ANTIWINDUP_NONE) {
        return; // No anti-windup
    }
    
    bool saturated = (rawOutput != clampedOutput);
    
    if (antiWindupMode_ == ANTIWINDUP_CLAMP) {
        // Clamp integral when output is saturated
        if (saturated) {
            // Reverse the last integral accumulation
            integral_ -= error_ * dt;
        }
    } else if (antiWindupMode_ == ANTIWINDUP_BACKCALC) {
        // Back-calculation feeds the saturation excess back through the
        // integrator. With no meaningful integral gain there is nothing to
        // correct, and 1/ki_ would be inf: that value lands in integral_ and
        // never recovers, so every subsequent output is inf or NaN. A PD
        // controller (ki = 0) configured with BACKCALC hit this immediately.
        bool haveIntegralGain = (ki_ > EASYPID_MIN_KI) || (ki_ < -EASYPID_MIN_KI);
        if (saturated && haveIntegralGain) {
            float backCalcGain = 1.0f / ki_; // Typical approach
            float error_back = (clampedOutput - rawOutput) * backCalcGain;
            integral_ += error_back * dt;
        }
    }
}

void PIDController::setTunings(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    outMin_ = min;
    outMax_ = max;
    
    // Update integral limits if not explicitly set
    if (!integralLimitsSet_) {
        integralMin_ = min;
        integralMax_ = max;
    }
}

void PIDController::setIntegralLimits(float min, float max) {
    integralMin_ = min;
    integralMax_ = max;
    integralLimitsSet_ = true;
}

void PIDController::setAntiWindup(AntiWindupMode mode) {
    antiWindupMode_ = mode;
}

void PIDController::setDerivativeFilter(DerivativeFilterMode mode, float alpha) {
    filterMode_ = mode;
    filterAlpha_ = alpha;
    
    // Clamp alpha to valid range
    if (filterAlpha_ < 0.0f) filterAlpha_ = 0.0f;
    if (filterAlpha_ > 1.0f) filterAlpha_ = 1.0f;
}

void PIDController::setSampleTime(unsigned long ms) {
    sampleTime_ = ms;
}

void PIDController::setDirection(ControlDirection direction) {
    direction_ = direction;
}

void PIDController::reset() {
    // Reset all state variables (similar to tracker.h resetIntegral, lines 62-64)
    error_ = 0.0f;
    previousError_ = 0.0f;
    firstUpdate_ = true;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    derivativeFiltered_ = 0.0f;
    pTerm_ = 0.0f;
    iTerm_ = 0.0f;
    dTerm_ = 0.0f;
    output_ = 0.0f;

    // Restart the automatic-timing reference too. reset() is typically called
    // after a pause or a large setpoint change; without this the next
    // automatic update would treat the entire idle period as one sample and
    // integrate it in a single step.
    lastTime_ = millis();
}

float PIDController::getError() const {
    return error_;
}

float PIDController::getPterm() const {
    return pTerm_;
}

float PIDController::getIterm() const {
    return iTerm_;
}

float PIDController::getDterm() const {
    return dTerm_;
}

float PIDController::getOutput() const {
    return output_;
}
