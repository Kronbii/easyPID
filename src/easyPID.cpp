/**
 * @file easyPID.cpp
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
    controlError_ = 0.0f;
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
    
    // No integral limits until setIntegralLimits() is called. These seeds are
    // never read while integralLimitsSet_ is false; they exist only so the
    // members are not indeterminate.
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
    unsigned long elapsed = now - lastTime_; // unsigned: safe across millis() rollover

    // No time has passed since the last update, so there is no new information
    // and nothing to integrate. Return the previous output unchanged.
    //
    // Substituting a nominal sampleTime_ here (as this used to do) credited a
    // full sample period of integration to a call that took no time at all.
    // The documented usage calls update() unconditionally from loop(), which on
    // any reasonably fast board runs many times per millisecond, so the
    // integral accrued at up to 100x the true rate.
    //
    // lastTime_ is deliberately NOT advanced here, so sub-millisecond time is
    // carried into the next call rather than being discarded.
    if (elapsed == 0UL) {
        return output_;
    }

    lastTime_ = now;
    float dt = (float)elapsed / 1000.0f; // Convert to seconds

    return computePID(setpoint, measurement, dt);
}

float PIDController::update(float setpoint, float measurement, float dtMs) {
    if (!initialized_) {
        begin();
    }

    // A non-positive dt carries no information and would divide by zero in the
    // derivative. Return the previous output rather than inventing a timestep.
    if (dtMs <= 0.0f) {
        return output_;
    }

    float dt = dtMs / 1000.0f; // Convert to seconds

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
    // error_ is what getError() reports and is always setpoint - measurement,
    // independent of control direction. controlError_ is the sign-corrected
    // value that actually drives the terms.
    error_ = setpoint - measurement;
    controlError_ = (direction_ == REVERSE) ? -error_ : error_;

    // Calculate proportional term
    pTerm_ = kp_ * controlError_;

    // Calculate and accumulate integral term with dt scaling
    // Original form accumulated the raw error;
    // Enhanced: integral += error * dt (for time-aware integration)
    //
    // Remember the pre-accumulation value so conditional integration can undo
    // exactly what was added, including any truncation by the integral limits.
    float integralBefore = integral_;
    integral_ += controlError_ * dt;
    
    // Clamp integral to limits if set
    if (integralLimitsSet_) {
        if (integral_ > integralMax_) {
            integral_ = integralMax_;
        } else if (integral_ < integralMin_) {
            integral_ = integralMin_;
        }
    }
    
    iTerm_ = ki_ * integral_;
    
    // Calculate derivative term with dt scaling
    // Original form differenced consecutive errors;
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
        derivative_ = (controlError_ - previousError_) / dt;
    }

    // Apply derivative filtering if enabled
    float derivativeToUse = derivative_;
    if (filterMode_ == FILTER_EMA) {
        // Exponential Moving Average (1st order low-pass filter)
        derivativeFiltered_ = filterAlpha_ * derivativeFiltered_ + (1.0f - filterAlpha_) * derivative_;
        derivativeToUse = derivativeFiltered_;
    }
    
    dTerm_ = kd_ * derivativeToUse;
    
    // Calculate total PID output
    
    float rawOutput = pTerm_ + iTerm_ + dTerm_;
    
    // Clamp output to limits
    float clampedOutput = rawOutput;
    if (clampedOutput > outMax_) {
        clampedOutput = outMax_;
    } else if (clampedOutput < outMin_) {
        clampedOutput = outMin_;
    }
    
    // Apply anti-windup
    applyAntiWindup(rawOutput, clampedOutput, dt, integralBefore);

    // Anti-windup may have changed the integrator, so refresh the reported
    // I term. Without this getIterm() showed the pre-correction value and
    // kept climbing during saturation, making it look as though anti-windup
    // was not working.
    iTerm_ = ki_ * integral_;
    
    // Store for next iteration
    previousError_ = controlError_;
    
    output_ = clampedOutput;
    return output_;
}

void PIDController::applyAntiWindup(float rawOutput, float clampedOutput, float dt, float integralBefore) {
    if (antiWindupMode_ == ANTIWINDUP_NONE) {
        return; // No anti-windup
    }

    bool saturated = (rawOutput != clampedOutput);

    if (antiWindupMode_ == ANTIWINDUP_CLAMP) {
        // Conditional integration: inhibit only the accumulation that drives
        // FURTHER into saturation. Reverting unconditionally also cancels
        // accumulation that would relieve saturation, which freezes the
        // integrator: once the I term alone exceeds the limit, the output can
        // stay pinned at the rail forever even under a large opposing error.
        //
        // Restoring the saved value rather than subtracting error*dt also
        // makes the rollback an exact inverse when the integral limits
        // truncated this cycle's accumulation.
        bool pushingIntoSaturation =
            (rawOutput > outMax_ && controlError_ > 0.0f) ||
            (rawOutput < outMin_ && controlError_ < 0.0f);

        if (pushingIntoSaturation) {
            integral_ = integralBefore;
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
    // Inverted or degenerate limits would make every output "saturated" and
    // permanently inhibit integration. Ignore them rather than bricking the
    // controller silently.
    if (min >= max) {
        return;
    }

    outMin_ = min;
    outMax_ = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    if (min >= max) {
        return;
    }

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

    // Clamp alpha to a usable range. The upper bound is strict: at exactly 1.0
    // the EMA becomes filtered = 1*filtered + 0*raw, so the filtered derivative
    // is frozen at its initial value and the D term is permanently dead.
    if (filterAlpha_ < 0.0f) filterAlpha_ = 0.0f;
    if (filterAlpha_ > 0.999f) filterAlpha_ = 0.999f;
}

void PIDController::setSampleTime(unsigned long ms) {
    // Advisory only: the controller derives dt from the actual elapsed time or
    // from the caller-supplied value, never from this. Zero is rejected so the
    // stored value cannot claim an impossible rate.
    if (ms > 0UL) {
        sampleTime_ = ms;
    }
}

void PIDController::setDirection(ControlDirection direction) {
    if (direction == direction_) {
        return;
    }

    direction_ = direction;

    // The carried state was accumulated under the opposite sign convention.
    // Leaving it as-is makes the integrator fight the new direction until it
    // has bled off, and produces one large spurious derivative sample from the
    // sign flip in previousError_. Negating it makes the switch bumpless.
    integral_ = -integral_;
    previousError_ = -previousError_;
    derivativeFiltered_ = -derivativeFiltered_;
    controlError_ = -controlError_;
}

void PIDController::reset() {
    // Reset all state variables
    error_ = 0.0f;
    controlError_ = 0.0f;
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
