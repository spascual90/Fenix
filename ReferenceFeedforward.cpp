#include "ReferenceFeedforward.h"

const float ReferenceFeedforward::EPSILON = 1.0e-6f;

/**
 * Constructor.
 *
 * Initializes the object with safe defaults:
 * - Kff and T are zero, so the model starts invalid.
 * - The block starts enabled, but produces zero output until valid parameters
 *   are set.
 * - Output limits are wide enough for most tests and may be replaced later.
 */
ReferenceFeedforward::ReferenceFeedforward()
    : _Kff(0.0f),
      _T(0.0f),
      _N_delta(0.0f),
      _deltaMin(-PI / 2.0f),
      _deltaMax(PI / 2.0f),
      _lastRd(0.0f),
      _lastRDotD(0.0f),
      _deltaFF(0.0f),
      _enabled(true),
      _validModel(false),
      _deltaAvailable(false) {
    refreshValidity();
}

/**
 * Configures the feedforward gain and Nomoto time constant.
 *
 * Inputs:
 * - Kff: Feedforward gain.
 *      It must be greater than zero.
 * - T: Nomoto time constant in seconds. Must be greater than zero.
 *
 * Output:
 * - No direct return value.
 * - Updates the internal validity flag used by update().
 *
 * Notes:
 * - The implementation calculates:
 *
 *       tau_ff   = Kff * (r_dot_d + r_d / T)
 *       delta_ff = tau_ff / N_delta
 *
 * - For direct rudder-angle feedforward, configure N_delta = 1.0f.
 */
void ReferenceFeedforward::setModelParams(float Kff, float T) {
    _Kff = Kff;
    _T = T;
    refreshValidity();

    if (!_validModel) {
        clearOutputs();
    }
}

/**
 * Configures the divisor from the intermediate feedforward value to rudder angle.
 *
 * Input:
 * - N_delta: divisor used in delta_ff = tau_ff / N_delta.
 *
 * Output:
 * - No direct return value.
 * - If N_delta is valid, update() will calculate delta_ff.
 * - If N_delta is zero or very close to zero, delta_ff is considered unavailable.
 *
 * For direct rudder-angle feedforward, use N_delta = 1.0f.
 */
void ReferenceFeedforward::setRudderGain(float N_delta) {
    _N_delta = N_delta;
    refreshValidity();

    if (!_deltaAvailable) {
        _deltaFF = 0.0f;
    }
}

/**
 * Sets the saturation limits for the rudder angle feedforward output.
 *
 * Inputs:
 * - deltaMin: minimum allowed rudder feedforward angle in radians.
 * - deltaMax: maximum allowed rudder feedforward angle in radians.
 *
 * Output:
 * - No direct return value.
 * - The last calculated delta_ff is immediately re-saturated to the new limits.
 */
void ReferenceFeedforward::setDeltaLimits(float deltaMin, float deltaMax) {
    if (deltaMin <= deltaMax) {
        _deltaMin = deltaMin;
        _deltaMax = deltaMax;
    } else {
        _deltaMin = deltaMax;
        _deltaMax = deltaMin;
    }

    _deltaFF = saturate(_deltaFF, _deltaMin, _deltaMax);
}

/**
 * Enables the feedforward calculation.
 *
 * Input:
 * - None.
 *
 * Output:
 * - No direct return value.
 * - Subsequent calls to update() will calculate feedforward if parameters are valid.
 */
void ReferenceFeedforward::enable() {
    _enabled = true;
}

/**
 * Disables the feedforward calculation.
 *
 * Input:
 * - None.
 *
 * Output:
 * - No direct return value.
 * - Current outputs are forced to zero.
 */
void ReferenceFeedforward::disable() {
    _enabled = false;
    clearOutputs();
}

/**
 * Returns the activation state of the block.
 *
 * Input:
 * - None.
 *
 * Output:
 * - true if the block is enabled.
 * - false if it is disabled.
 */
bool ReferenceFeedforward::isEnabled() const {
    return _enabled;
}

/**
 * Calculates the reference feedforward term for one control cycle.
 *
 * Inputs:
 * - r_d: desired yaw rate in rad/s.
 * - r_dot_d: desired yaw acceleration in rad/s^2.
 *
 * Outputs:
 * - No direct return value.
 * - Stores tau_ff internally, available through getTauFF().
 * - Stores delta_ff internally, available through getDeltaFF(), if N_delta is valid.
 *
 * Behaviour:
 * - If disabled, output is zero.
 * - If Kff or T are invalid, output is zero.
 * - Otherwise:
 *
 *       dynamic_term = r_dot_d + r_d / T
 *       tau_ff       = Kff * dynamic_term
 *
 * - tau_ff is saturated between tauMin and tauMax.
 * - delta_ff is calculated as tau_ff / N_delta only when N_delta is valid.
 * - For direct rudder-angle feedforward, N_delta should be 1.0f.
 */
void ReferenceFeedforward::update(float r_d, float r_dot_d) {
    _lastRd = r_d;
    _lastRDotD = r_dot_d;

    if (!_enabled || !_validModel) {
        clearOutputs();
        return;
    }

    if (_deltaAvailable) {
        float rawDeltaFF = _Kff * (r_dot_d + (r_d / _T)) / _N_delta;
        _deltaFF = saturate(rawDeltaFF, _deltaMin, _deltaMax);
    } else {
        _deltaFF = 0.0f;
    }
}


/**
 * Returns the last rudder angle feedforward value.
 *
 * Input:
 * - None.
 *
 * Output:
 * - delta_ff after saturation.
 * - Zero if N_delta has not been configured correctly.
 */
float ReferenceFeedforward::getDeltaFF() const {
    return _deltaFF;
}

/**
 * Returns whether the rudder angle feedforward output is available.
 *
 * Input:
 * - None.
 *
 * Output:
 * - true if N_delta is valid.
 * - false if N_delta is zero or too close to zero.
 */
bool ReferenceFeedforward::getDeltaFFAvailable() const {
    return _deltaAvailable;
}

/**
 * Returns whether the configured Kff/T parameters are valid.
 *
 * Input:
 * - None.
 *
 * Output:
 * - true if Kff > 0 and T > 0.
 * - false otherwise.
 */
bool ReferenceFeedforward::isValid() const {
    return _validModel;
}

/**
 * Returns the last desired yaw rate received by update().
 *
 * Input:
 * - None.
 *
 * Output:
 * - r_d in rad/s.
 */
float ReferenceFeedforward::getLastRd() const {
    return _lastRd;
}

/**
 * Returns the last desired yaw acceleration received by update().
 *
 * Input:
 * - None.
 *
 * Output:
 * - r_dot_d in rad/s^2.
 */
float ReferenceFeedforward::getLastRDotD() const {
    return _lastRDotD;
}

/**
 * Returns the configured direct feedforward gain.
 *
 * Input:
 * - None.
 *
 * Output:
 * - Kff value currently configured.
 */
float ReferenceFeedforward::getKff() const {
    return _Kff;
}

/**
 * Returns the configured Nomoto time constant.
 *
 * Input:
 * - None.
 *
 * Output:
 * - T value currently configured, in seconds.
 */
float ReferenceFeedforward::getT() const {
    return _T;
}

/**
 * Returns the configured intermediate-to-rudder divisor.
 *
 * Input:
 * - None.
 *
 * Output:
 * - N_delta value currently configured.
 */
float ReferenceFeedforward::getNDelta() const {
    return _N_delta;
}

/**
 * Saturates a value between a lower and upper limit.
 *
 * Inputs:
 * - value: input value to limit.
 * - minValue: minimum allowed value.
 * - maxValue: maximum allowed value.
 *
 * Output:
 * - The limited value.
 */
float ReferenceFeedforward::saturate(float value, float minValue, float maxValue) const {
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

/**
 * Clears the calculated outputs.
 *
 * Input:
 * - None.
 *
 * Output:
 * - No direct return value.
 * - tau_ff and delta_ff are both set to zero.
 */
void ReferenceFeedforward::clearOutputs() {
    _deltaFF = 0.0f;
}

/**
 * Refreshes internal validity flags.
 *
 * Input:
 * - None.
 *
 * Output:
 * - No direct return value.
 * - _validModel becomes true only when Kff > 0 and T > 0.
 * - _deltaAvailable becomes true only when abs(N_delta) is sufficiently large.
 */
void ReferenceFeedforward::refreshValidity() {
    _validModel = (_Kff > EPSILON) && (_T > EPSILON);
    _deltaAvailable = (fabs(_N_delta) > EPSILON);
}

/**
 * Configures direct rudder-angle feedforward parameters.
 *
 * Inputs:
 * - Kff: direct feedforward gain from dynamic term to rudder angle.
 * - T: identified Nomoto time constant in seconds.
 *
 * Output:
 * - No direct return value.
 *
 * Behaviour:
 * - The general implementation is:
 *
 *       tau_ff   = Kff * (r_dot_d + r_d / T)
 *       delta_ff = tau_ff / N_delta
 *
 * - To calculate delta_ff directly, this method configures:
 *
 *       _Kff    = Kff
 *       _T      = T
 *       N_delta = 1
 *
 * so that:
 *
 *       delta_ff = Kff * (r_dot_d + r_d / T)
 *
 * Important:
 * - getKff() returns Kff.
 * - In the current header this method is private. External code can obtain the
 *   same behaviour by calling setModelParams(Kff, T) and setRudderGain(1.0f),
 *   unless this method is moved to public.
 */
void ReferenceFeedforward::setDirectDeltaParams(float Kff, float T) {
    _Kff = Kff;
    _T = T;
    _N_delta = 1.0f;

    refreshValidity();

    if (!_validModel) {
        clearOutputs();
    }
}
