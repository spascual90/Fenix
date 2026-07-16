#ifndef REFERENCE_FEEDFORWARD_H
#define REFERENCE_FEEDFORWARD_H

#include <Arduino.h>
// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"
/**
 * @class ReferenceFeedforward
 * @brief Calculates the reference feedforward rudder-angle contribution for a marine heading autopilot.
 *
 * The implementation uses a first-order Nomoto-style dynamic term generated
 * from the reference yaw trajectory:
 *
 *     dynamic_term = r_dot_d + r_d / T
 *
 * The internal gain is named _Kff and scales this dynamic term:
 *
 *     tau_ff = Kff * dynamic_term
 *
 * In this implementation, tau_ff is an intermediate feedforward value kept for
 * compatibility with the previous API. It should only be interpreted as a
 * physical yaw moment if Kff has been deliberately calibrated with physical
 * moment units.
 *
 * The final rudder feedforward angle is calculated only when N_delta is valid:
 *
 *     delta_ff = tau_ff / N_delta
 *
 * For direct rudder-angle feedforward, configure:
 *
 *     Kff     = experimentally identified direct gain
 *     T       = experimentally identified Nomoto time constant
 *     N_delta = 1
 *
 * so that:
 *
 *     delta_ff = Kff * (r_dot_d + r_d / T)
 *
 * Units expected internally:
 *   - r_d: rad/s
 *   - r_dot_d: rad/s^2
 *   - T: s
 *   - Kff: direct feedforward gain or intermediate scaling gain
 *   - tau_ff: intermediate Kff-scaled value
 *   - N_delta: divisor from intermediate value to rudder angle
 *   - delta_ff: rad
 */
class ReferenceFeedforward {
public:
    /**
     * @brief Creates a feedforward object with safe default values.
     *
     * By default, the object is enabled, but invalid until Kff and T are
     * configured with positive values using setModelParams() or the internal
     * direct-delta helper.
     */
    ReferenceFeedforward();

    /**
     * @brief Configures the feedforward gain and Nomoto time constant.
     * @param m Legacy parameter name. In this implementation it is stored as Kff.
     *          Must be > 0.
     * @param T Nomoto time constant in seconds. Must be > 0.
     *
     * The name m is kept for API compatibility with older code that treated this
     * parameter as an effective yaw inertia. With the current _Kff implementation,
     * the output is:
     *
     *     tau_ff   = Kff * (r_dot_d + r_d / T)
     *     delta_ff = tau_ff / N_delta
     *
     * For direct rudder-angle output, call setRudderGain(1.0f) after this method.
     * If either Kff or T is invalid, update() will return zero outputs.
     */
    void setModelParams(float m, float T);

    /**
     * @brief Configures the divisor from the intermediate feedforward value to rudder angle.
     * @param N_delta Divisor applied as delta_ff = tau_ff / N_delta.
     *
     * For direct rudder-angle feedforward, use N_delta = 1.0f.
     * If N_delta is zero or too close to zero, delta_ff will not be calculated
     * and getDeltaFFAvailable() will return false.
     */
    void setRudderGain(int N_delta);
    int getRudderGain() const;


    /**
     * @brief Sets the saturation limits for the rudder angle feedforward output.
     * @param deltaMin Minimum allowed delta_ff in radians.
     * @param deltaMax Maximum allowed delta_ff in radians.
     *
     * If deltaMin is greater than deltaMax, the values are automatically swapped.
     */
    void setDeltaLimits(float deltaMin, float deltaMax);

    /**
     * @brief Enables the feedforward calculation.
     *
     * When enabled, update() calculates the intermediate feedforward value and
     * delta_ff if the configured parameters are valid.
     */
    void enable();

    /**
     * @brief Disables the feedforward calculation.
     *
     * When disabled, update() forces tau_ff and delta_ff to zero.
     */
    void disable();

    /**
     * @brief Returns whether the feedforward block is enabled.
     * @return true if enabled, false otherwise.
     */
    bool isEnabled() const;

    /**
     * @brief Updates the feedforward calculation for the current loop cycle.
     * @param r_d Desired yaw rate in rad/s.
     * @param r_dot_d Desired yaw acceleration in rad/s^2.
     *
     * The method stores the input values for diagnostics and calculates:
     *
     *     dynamic_term = r_dot_d + r_d / T
     *     tau_ff       = Kff * dynamic_term
     *     delta_ff     = tau_ff / N_delta
     *
     * If the object is disabled or invalid, the outputs are set to zero.
     * If N_delta is unavailable, tau_ff is still updated but delta_ff is zero.
     */
    void update(float r_d, float r_dot_d);

    /**
     * @brief Returns the last calculated rudder angle feedforward value.
     * @return delta_ff after saturation, or zero if N_delta is unavailable.
     */
    float getDeltaFF() const;

    /**
     * @brief Returns whether delta_ff can be calculated.
     * @return true if N_delta is valid and delta_ff is being calculated.
     */
    bool getDeltaFFAvailable() const;

    /**
     * @brief Returns whether the feedforward parameters are valid.
     * @return true if Kff > 0 and T > 0.
     */
    bool isValid() const;

    /**
     * @brief Returns the last desired yaw rate used by update().
     * @return r_d in rad/s.
     */
    float getLastRd() const;

    /**
     * @brief Returns the last desired yaw acceleration used by update().
     * @return r_dot_d in rad/s^2.
     */
    float getLastRDotD() const;

    /**
     * @brief Returns the configured direct feedforward gain.
     * @return Kff parameter.
     */
    float getKff() const;

    /**
     * @brief Returns the configured Nomoto time constant.
     * @return T parameter in seconds.
     */
    float getT() const;

    /**
     * @brief Returns the configured intermediate-to-rudder divisor.
     * @return N_delta parameter.
     */
    float getNDelta() const;

    /**
     * @brief Configures direct rudder-angle feedforward parameters.
     *
     * Functionality:
     *   Configures the feedforward block to calculate delta_ff directly as:
     *
     *       delta_ff = Kff * (r_dot_d + r_d / T)
     *
     *   This is achieved by storing Kff in _Kff and setting N_delta to 1.
     *
     * Inputs:
     *   Kff:
     *     Direct feedforward gain from the reference dynamic term to rudder angle.
     *
     *   T:
     *     Identified Nomoto time constant, seconds.
     *
     * Outputs:
     *   None directly. Internal parameters are configured so that getDeltaFF()
     *   returns the direct rudder angle feedforward in radians.
     *
     */
    void setDirectDeltaParams(float Kff, float T);

private:
    static const float EPSILON;

    float _Kff;
    float _T;
    float _N_delta;

    float _deltaMin;
    float _deltaMax;

    float _lastRd;
    float _lastRDotD;
    float _deltaFF;

    bool _enabled;
    bool _validModel;
    bool _deltaAvailable;

    /**
     * @brief Saturates a value between a minimum and maximum value.
     * @param value Input value.
     * @param minValue Minimum allowed value.
     * @param maxValue Maximum allowed value.
     * @return Saturated value.
     */
    float saturate(float value, float minValue, float maxValue) const;

    /**
     * @brief Resets the calculated outputs to zero.
     *
     * This is used when the object is disabled or invalid.
     */
    void clearOutputs();

    /**
     * @brief Recalculates the internal validity flags.
     *
     * _validModel depends on Kff and T.
     * _deltaAvailable depends on N_delta.
     */
    void refreshValidity();

};

#endif // REFERENCE_FEEDFORWARD_H
