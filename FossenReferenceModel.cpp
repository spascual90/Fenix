#include "FossenReferenceModel.h"

FossenReferenceModel::FossenReferenceModel()
{
    psi_r = 0.0f;
    psi_d = 0.0f;
    r_d = 0.0f;
    a_d = 0.0f;
    a_d_dot = 0.0f;

    setReferenceModelParams(0.3f, 1.0f, 0.1f);

    max_r_d = radians(3.0f);
    max_a_d = radians(0.5f);

    yawTolerance = radians(0.5f);
    yawRateTolerance = radians(0.05f);
    yawAccelerationTolerance = radians(0.02f);

    lastUpdateMs = 0.0f;

    state = ReferenceState::IDLE;
}

void FossenReferenceModel::printParam(void) {
	DEBUG_sprintf("wb, wn, z", omega_b, omega_n, zeta);
	DEBUG_sprintf("Kp, Ki, Kd", getKp(), getKi(), getKd());
}

void FossenReferenceModel::begin(float initialPsi)
{
    float psi = wrapAngle(initialPsi);

    psi_r = psi;
    psi_d = psi;
    r_d = 0.0f;
    a_d = 0.0f;
    a_d_dot = 0.0f;

    lastUpdateMs = millis();

    state = ReferenceState::IDLE;
}

void FossenReferenceModel::setReferenceModelParams(float wb, float damping, float integral, bool recal)
{
    if (wb > 0.0f) {
        omega_b = wb;
    }

    if (damping > 0.0f) {
        zeta = damping;
    }

    if (integral > 0.0f) {
        integralFactor = integral;
    }

    if (recal) {
		// Recalculate dependant variables
		// Recalculate omega_n
		computeWnFromWb();
		// Recalculate PID params
		computePIDfromWnZ();
    }
}

void FossenReferenceModel::setLimits(float maxYawRate, float maxYawAcceleration)
{
    if (maxYawRate > 0.0f) {
        max_r_d = maxYawRate;
    }

    if (maxYawAcceleration > 0.0f) {
        max_a_d = maxYawAcceleration;
    }
}

void FossenReferenceModel::setFinishTolerances(float yawTol,
                                               float yawRateTol,
                                               float yawAccelTol)
{
    if (yawTol > 0.0f) {
        yawTolerance = yawTol;
    }

    if (yawRateTol > 0.0f) {
        yawRateTolerance = yawRateTol;
    }

    if (yawAccelTol > 0.0f) {
        yawAccelerationTolerance = yawAccelTol;
    }
}

void FossenReferenceModel::setTargetYaw(float targetPsi)
{
    psi_r = wrapAngle(targetPsi);
    state = ReferenceState::RUNNING;
}

void FossenReferenceModel::changeTargetYaw(float newTargetPsi)
{
    setTargetYaw(newTargetPsi);
}

void FossenReferenceModel::startNewProcess(float currentPsi, float targetPsi)
{
    psi_d = wrapAngle(currentPsi);
    psi_r = wrapAngle(targetPsi);

    r_d = 0.0f;
    a_d = 0.0f;
    a_d_dot = 0.0f;

    state = ReferenceState::RUNNING;
}

void FossenReferenceModel::cancelProcess(float currentPsi)
{
    float psi = wrapAngle(currentPsi);

    psi_r = psi;
    psi_d = psi;

    r_d = 0.0f;
    a_d = 0.0f;
    a_d_dot = 0.0f;

    state = ReferenceState::IDLE;
}

void FossenReferenceModel::update(void)
{
	unsigned long now = millis();
	float dt = (now - lastUpdateMs) / 1000.0;
	lastUpdateMs = now;
	update (dt);
}

void FossenReferenceModel::update(float dt)
{
	if (dt <= 0.0f) {
        return;
    }
    if (state != ReferenceState::RUNNING) {
        return;
    }

    float error = wrapAngle(psi_r - psi_d);

    float rd_sat = saturate(r_d, max_r_d);
    float ad_sat = saturate(a_d, max_a_d);

    /*
     * Third-order Fossen-style reference model:
     *
     * a_d_dot =
     *   -(zeta + 1) * omega_n       * sat(a_d)
     *   -(2*zeta + 1) * omega_n^2   * sat(r_d)
     *   + omega_n^3                 * (psi_r - psi_d)
     */
    a_d_dot =
        -(zeta + 1.0f) * omega_n * ad_sat
        -(2.0f * zeta + 1.0f) * omega_n * omega_n * rd_sat
        + omega_n * omega_n * omega_n * error;

    a_d += a_d_dot * dt;
    a_d = saturate(a_d, max_a_d);

    r_d += a_d * dt;
    r_d = saturate(r_d, max_r_d);

    psi_d += r_d * dt;
    psi_d = wrapAngle(psi_d);

    checkFinished();

}

bool FossenReferenceModel::isRunning() const
{
    return state == ReferenceState::RUNNING;
}

bool FossenReferenceModel::isFinished() const
{
    return state == ReferenceState::FINISHED;
}

float FossenReferenceModel::getTargetYaw() const
{
    return psi_r;
}

float FossenReferenceModel::getDesiredYaw() const
{
    return psi_d;
}

float FossenReferenceModel::getDesiredYawRate() const
{
    return r_d;
}

float FossenReferenceModel::getDesiredYawAcceleration() const
{
    return a_d;
}

float FossenReferenceModel::getDesiredYawJerk() const
{
    return a_d_dot;
}

ReferenceState FossenReferenceModel::getState() const
{
    return state;
}


void FossenReferenceModel::setIntegralFactor(float factor)
{
    if (factor >= 0.0f) {
        integralFactor = factor;
    }
}

float FossenReferenceModel::getKp() const
{
    return  (Kp);
}

float FossenReferenceModel::getKi() const
{
    return  (Ki);
}

float FossenReferenceModel::getKd() const
{
    return  (Kd);
}

void FossenReferenceModel::checkFinished()
{
    float error = wrapAngle(psi_r - psi_d);

    if (fabs(error) < yawTolerance &&
        fabs(r_d) < yawRateTolerance &&
        fabs(a_d) < yawAccelerationTolerance)
    {
        psi_d = psi_r;
        r_d = 0.0f;
        a_d = 0.0f;
        a_d_dot = 0.0f;

        state = ReferenceState::FINISHED;
    }
}

float FossenReferenceModel::saturate(float value, float limit) const
{
    if (limit <= 0.0f) {
        return value;
    }

    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

float FossenReferenceModel::wrapAngle(float angle) const
{
    while (angle > PI) {
        angle -= 2.0f * PI;
    }

    while (angle < -PI) {
        angle += 2.0f * PI;
    }

    return angle;
}

void FossenReferenceModel::computePIDfromWnZ(void)
{
    /*
     * These coefficients are an initial normalized tuning rule. In a real boat,
     * they normally need scaling according to the actuator, rudder geometry,
     * yaw dynamics and sign convention.
     */
	// Como 350 unidades de timón/ 35° = 10 unidades por grado de timón
	// KP, unidades/grado ​ =KP, rad/rad​ * 10
	float factor_rudder = 10.0f;
    Kp = factor_rudder * omega_n * omega_n;
    Kd = factor_rudder * 2.0f * zeta * omega_n;
    Ki = factor_rudder * integralFactor * omega_n * omega_n * omega_n;

}

float FossenReferenceModel::computeWnFromWb(void)
{
    // Validación básica
    if (zeta <= 0.0f || omega_b <= 0.0f) {
        return 0.0f;
    }

    float zeta2 = zeta * zeta;
    float zeta4 = zeta2 * zeta2;

    float inner = 4.0f * zeta4 - 4.0f * zeta2 + 2.0f;

    // Protección numérica
    if (inner <= 0.0f) {
        return 0.0f;
    }

    float factorSquared = 1.0f - 2.0f * zeta2 + sqrtf(inner);

    // Protección frente a división por cero o raíz inválida
    if (factorSquared <= 0.0f) {
        return 0.0f;
    }

    omega_n = omega_b/ sqrtf(factorSquared);

}
