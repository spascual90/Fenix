#ifndef FOSSEN_REFERENCE_MODEL_H
#define FOSSEN_REFERENCE_MODEL_H

#include <Arduino.h>
// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

/**
 * @brief State of the reference model transition process.
 *
 * IDLE:
 *   No active transition is running.
 *
 * RUNNING:
 *   The model is integrating the reference trajectory toward the target yaw.
 *
 * FINISHED:
 *   The target yaw has been reached within the configured tolerances.
 */
enum class ReferenceState {
    IDLE,
    RUNNING,
    FINISHED
};

/**
 * @brief Third-order Fossen-style yaw reference model for Arduino.
 *
 * This class generates a smooth yaw reference trajectory from a raw target yaw.
 *
 * The raw target yaw is:
 *
 *     psi_r
 *
 * The generated smooth trajectory is:
 *
 *     psi_d : desired yaw angle
 *     r_d   : desired yaw rate
 *     a_d   : desired yaw acceleration
 *
 * The class stores its internal state and integrates it over time every time
 * update(dt) is called from the Arduino loop.
 *
 * Typical use:
 *
 *     FossenReferenceModel ref;
 *
 *     ref.begin(currentHeading);
 *     ref.setReferenceModelParams(0.3, 1.0);
 *     ref.setLimits(radians(3.0), radians(0.5));
 *     ref.startNewProcess(currentHeading, radians(120.0));
 *
 *     void loop() {
 *         ref.update(dt);
 *         float psi_d = ref.getDesiredYaw();
 *     }
 *
 * Angles are expected in radians.
 */
class FossenReferenceModel
{
public:
    /**
     * @brief Constructor.
     *
     * Functionality:
     *   Creates the reference model with safe default values.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   None directly. The internal state is initialized.
     */
    FossenReferenceModel();

    /**
     * @brief Initializes the model from the current yaw angle.
     *
     * Functionality:
     *   Sets the target yaw, desired yaw, desired yaw rate and desired yaw
     *   acceleration consistently with the current yaw. This is normally called
     *   once during setup or when the autopilot is enabled.
     *
     * Inputs:
     *   initialPsi:
     *     Current yaw angle in radians.
     *
     * Outputs:
     *   None directly. Internal state is reset to initialPsi.
     */
    void begin(float initialPsi);

    /**
     * @brief Sets the third-order reference model parameters.
     *
     * Functionality:
     *   Defines the natural frequency and damping ratio used by the Fossen-style
     *   reference model.
     *
     * Inputs:
     *   omegaN:
     *     Natural frequency in rad/s. Higher values produce a faster reference
     *     response.
     *
     *   damping:
     *     Damping ratio. Higher values usually produce a less oscillatory
     *     reference response.
     *
     * Outputs:
     *   None directly. Internal parameters omega_n and zeta are updated.
     */
    void setReferenceModelParams(float omegaN, float damping, float integral=0.1, bool recal=true);

    /**
     * @brief Sets the saturation limits for desired yaw rate and acceleration.
     *
     * Functionality:
     *   Limits r_d and a_d to physically reasonable values. This prevents the
     *   reference model from requesting impossible manoeuvres.
     *
     * Inputs:
     *   maxYawRate:
     *     Maximum absolute desired yaw rate in rad/s.
     *
     *   maxYawAcceleration:
     *     Maximum absolute desired yaw acceleration in rad/s^2.
     *
     * Outputs:
     *   None directly. Internal limits are updated.
     */
    void setLimits(float maxYawRate, float maxYawAcceleration);

    /**
     * @brief Sets the tolerances used to detect the end of a transition.
     *
     * Functionality:
     *   Defines when the process is considered finished. The transition is
     *   finished when yaw error, desired yaw rate and desired yaw acceleration
     *   are all below their respective tolerances.
     *
     * Inputs:
     *   yawTol:
     *     Maximum yaw error in radians.
     *
     *   yawRateTol:
     *     Maximum absolute desired yaw rate in rad/s.
     *
     *   yawAccelTol:
     *     Maximum absolute desired yaw acceleration in rad/s^2.
     *
     * Outputs:
     *   None directly. Internal finish tolerances are updated.
     */
    void setFinishTolerances(float yawTol,
                             float yawRateTol,
                             float yawAccelTol);

    /**
     * @brief Sets a new raw target yaw.
     *
     * Functionality:
     *   Updates psi_r and starts or continues a transition toward the new
     *   target. This function does not reset psi_d, r_d or a_d, so it can be
     *   used during a manoeuvre without creating discontinuities.
     *
     * Inputs:
     *   targetPsi:
     *     New raw target yaw in radians.
     *
     * Outputs:
     *   None directly. psi_r is updated and the state becomes RUNNING.
     */
    void setTargetYaw(float targetPsi);

    /**
     * @brief Changes the raw target yaw during an active process.
     *
     * Functionality:
     *   Semantically equivalent to setTargetYaw(), but intended to make calling
     *   code clearer when the target is being modified during a transition.
     *
     * Inputs:
     *   newTargetPsi:
     *     New raw target yaw in radians.
     *
     * Outputs:
     *   None directly. psi_r is updated and the state becomes RUNNING.
     */
    void changeTargetYaw(float newTargetPsi);

    /**
     * @brief Starts a new reference transition from the current yaw.
     *
     * Functionality:
     *   Resets the smooth reference trajectory to the current yaw and starts a
     *   new process toward targetPsi. Desired yaw rate and acceleration are
     *   reset to zero.
     *
     * Inputs:
     *   currentPsi:
     *     Current measured yaw angle in radians.
     *
     *   targetPsi:
     *     Raw target yaw angle in radians.
     *
     * Outputs:
     *   None directly. Internal state is reset and state becomes RUNNING.
     */
    void startNewProcess(float currentPsi, float targetPsi);

    /**
     * @brief Cancels the current process.
     *
     * Functionality:
     *   Stops any active transition and fixes the reference model at the current
     *   yaw. Desired yaw rate and acceleration are reset to zero.
     *
     * Inputs:
     *   currentPsi:
     *     Current measured yaw angle in radians.
     *
     * Outputs:
     *   None directly. Internal state is reset and state becomes IDLE.
     */
    void cancelProcess(float currentPsi);

    /**
     * @brief Integrates the reference model one time step.
     *
     * Functionality:
     *   Calculates a_d_dot from the third-order reference model and integrates:
     *
     *       a_d   = a_d + a_d_dot * dt
     *       r_d   = r_d + a_d     * dt
     *       psi_d = psi_d + r_d   * dt
     *
     *   This function should be called once per Arduino loop iteration.
     *
     * Inputs:
     *   dt:
     *     Elapsed time since the previous call in seconds.
     *
     * Outputs:
     *   None directly. Internal state variables psi_d, r_d, a_d and a_d_dot
     *   are updated. The process state may change to FINISHED.
     */

    void update (void);
    void update(float dt);

    /**
     * @brief Returns whether the reference process is currently running.
     *
     * Functionality:
     *   Checks if the internal state is RUNNING.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   true if the model is currently integrating a transition.
     *   false otherwise.
     */
    bool isRunning() const;

    /**
     * @brief Returns whether the reference process has finished.
     *
     * Functionality:
     *   Checks if the internal state is FINISHED.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   true if the process has reached the target within the configured
     *   tolerances.
     *   false otherwise.
     */
    bool isFinished() const;

    /**
     * @brief Gets the raw target yaw psi_r.
     *
     * Functionality:
     *   Returns the current target yaw requested by the user, route manager,
     *   tack command or external command.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Target yaw psi_r in radians.
     */
    float getTargetYaw() const;

    /**
     * @brief Gets the smooth desired yaw psi_d.
     *
     * Functionality:
     *   Returns the yaw command generated by the reference model. This is the
     *   value that should normally be used as heading setpoint by the main
     *   controller.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Desired yaw psi_d in radians.
     */
    float getDesiredYaw() const;

    /**
     * @brief Gets the desired yaw rate r_d.
     *
     * Functionality:
     *   Returns the generated desired yaw rate.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Desired yaw rate r_d in rad/s.
     */
    float getDesiredYawRate() const;

    /**
     * @brief Gets the desired yaw acceleration a_d.
     *
     * Functionality:
     *   Returns the generated desired yaw acceleration.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Desired yaw acceleration a_d in rad/s^2.
     */
    float getDesiredYawAcceleration() const;

    /**
     * @brief Gets the desired yaw jerk a_d_dot.
     *
     * Functionality:
     *   Returns the derivative of desired yaw acceleration calculated by the
     *   third-order model.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Desired yaw jerk a_d_dot in rad/s^3.
     */
    float getDesiredYawJerk() const;

    /**
     * @brief Gets the current process state.
     *
     * Functionality:
     *   Returns whether the model is IDLE, RUNNING or FINISHED.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Current ReferenceState.
     */
    ReferenceState getState() const;

    /**
     * @brief Sets the integral factor used by computePIDFromBandwidth().
     *
     * Functionality:
     *   Controls how aggressive the integral term is when PID gains are
     *   generated from bandwidth.
     *
     * Inputs:
     *   factor:
     *     Multiplier for bandwidth^3 when calculating Ki.
     *
     * Outputs:
     *   None directly. Internal integralFactor is updated.
     */
    void setIntegralFactor(float factor);

    /**
     * @brief Gets the proportional gain.
     *
     * Functionality:
     *   Returns the last Kp value calculated by computePIDFromBandwidth().
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Kp value.
     */
    float getKp() const;

    /**
     * @brief Gets the integral gain.
     *
     * Functionality:
     *   Returns the last Ki value calculated by computePIDFromBandwidth().
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Ki value.
     */
    float getKi() const;

    /**
     * @brief Gets the derivative gain.
     *
     * Functionality:
     *   Returns the last Kd value calculated by computePIDFromBandwidth().
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   Kd value.
     */
    float getKd() const;

    void printParam(void);

private:
    unsigned long lastUpdateMs;
    float psi_r;       ///< Raw target yaw, rad.
    float psi_d;       ///< Smooth desired yaw, rad.
    float r_d;         ///< Smooth desired yaw rate, rad/s.
    float a_d;         ///< Smooth desired yaw acceleration, rad/s^2.
    float a_d_dot;     ///< Smooth desired yaw jerk, rad/s^3.

    //INDEPENDENT
    float omega_b;     ///< Natural frequency of the reference model, rad/s.
    float zeta;        ///< Damping ratio of the reference model.
    float integralFactor; ///< Integral gain multiplier.

    //DEPENDENT
    float omega_n;     ///< Speed ratio of the reference model. Dependency from omega_b and zeta
    float Kp;             ///< Computed proportional gain.
    float Ki;             ///< Computed integral gain.
    float Kd;             ///< Computed derivative gain.

    float max_r_d;     ///< Maximum absolute desired yaw rate, rad/s.
    float max_a_d;     ///< Maximum absolute desired yaw acceleration, rad/s^2.

    float yawTolerance;                  ///< Finish yaw tolerance, rad.
    float yawRateTolerance;              ///< Finish yaw rate tolerance, rad/s.
    float yawAccelerationTolerance;      ///< Finish yaw acceleration tolerance, rad/s^2.


    ReferenceState state; ///< Current process state.

    /**
     * @brief Checks whether the active process has finished.
     *
     * Functionality:
     *   Compares yaw error, desired yaw rate and desired yaw acceleration with
     *   the configured tolerances. If all conditions are satisfied, the model
     *   snaps psi_d to psi_r, clears dynamic states and sets state to FINISHED.
     *
     * Inputs:
     *   None.
     *
     * Outputs:
     *   None directly. Internal state may be modified.
     */
    void checkFinished();

    /**
     * @brief Saturates a value symmetrically.
     *
     * Functionality:
     *   Limits value to the range [-limit, +limit].
     *
     * Inputs:
     *   value:
     *     Value to be saturated.
     *
     *   limit:
     *     Positive saturation limit.
     *
     * Outputs:
     *   Saturated value.
     */
    float saturate(float value, float limit) const;

    /**
     * @brief Wraps an angle to the interval [-PI, PI].
     *
     * Functionality:
     *   Normalizes angles to avoid discontinuities when crossing 0/360 degrees.
     *
     * Inputs:
     *   angle:
     *     Angle in radians.
     *
     * Outputs:
     *   Wrapped angle in radians, within [-PI, PI].
     */
    float wrapAngle(float angle) const;

    /*
     * Calcula la frecuencia natural wn a partir del factor de amortiguamiento zeta
     * y del ancho de banda wb.
     *
     * Inputs:
     *   zeta : factor de amortiguamiento [-]
     *   wb   : ancho de banda [rad/s]
     *
     * Output:
     *   wn   : frecuencia natural [rad/s]
     *
     * Fórmula:
     *   wb = wn * sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2))
     *
     * Por tanto:
     *   wn = wb / sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2))
     */
    float computeWnFromWb(void);
    /**
     * @brief Computes PID coefficients from bandwidth and damping.
     *
     * Functionality:
     *   Provides a simple bandwidth-based initial tuning rule:
     *
     *       Kp = bandwidth^2
     *       Kd = 2 * damping * bandwidth
     *       Ki = integralFactor * bandwidth^3
     *
     *   These gains are dimensionally a simplified starting point. Depending on
     *   the plant model and actuator scaling, they may require additional
     *   normalization before being used as rudder-angle gains.
     *
     * Inputs:
     *   bandwidth:
     *     Desired controller bandwidth in rad/s.
     *
     *   damping:
     *     Desired damping ratio.
     *
     * Outputs:
     *   None directly. Internal Kp, Ki and Kd are updated.
     */
    void computePIDfromWnZ(void);
};



#endif
