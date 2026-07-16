#include "OnlineNomotoIdentifier.h"
#include "Fenix_config.h"

#include <math.h>

OnlineNomotoIdentifier::OnlineNomotoIdentifier()
    : S00(0.0f),
      S01(0.0f),
      S11(0.0f),
      q0(0.0f),
      q1(0.0f),
      yy(0.0f),
      dtSum(0.0f),
      weightSum(0.0f),
      A(0.0f),
      B(0.0f),
      T(0.0f),
      K(0.0f),
      Kff(0.0f),
      lastTimeMs(0),
      lastR(0.0f),
      lastDelta(0.0f),
      lastResidual(0.0f),
      residualRms(0.0f),
      forgettingFactor(1.0f),
      minDeterminant(1.0e-9f),
      minT(0.1f),
      maxT(60.0f),
      maxAbsK(10.0f),
      minimumSamples(30),
      useDeltaAverage(true),
      sampleCount(0),
      valid(false),
      hasPrevious(false)
{
}

void OnlineNomotoIdentifier::reset()
{
    S00 = 0.0f;
    S01 = 0.0f;
    S11 = 0.0f;
    q0 = 0.0f;
    q1 = 0.0f;
    yy = 0.0f;

    dtSum = 0.0f;
    weightSum = 0.0f;

    A = 0.0f;
    B = 0.0f;
    T = 0.0f;
    K = 0.0f;
    Kff = 0.0f;

    lastTimeMs = 0;
    lastR = 0.0f;
    lastDelta = 0.0f;

    lastResidual = 0.0f;
    residualRms = 0.0f;

    sampleCount = 0;
    valid = false;
    hasPrevious = false;
}

void OnlineNomotoIdentifier::setForgettingFactor(float lambda)
{
    if (lambda > 0.0f && lambda <= 1.0f && isFiniteFloat(lambda)) {
        forgettingFactor = lambda;
    }
}

void OnlineNomotoIdentifier::setMinSamples(unsigned long minSamples)
{
    if (minSamples > 2) {
        minimumSamples = minSamples;
    }
}

void OnlineNomotoIdentifier::setMinDeterminant(float minDet)
{
    if (minDet > 0.0f && isFiniteFloat(minDet)) {
        minDeterminant = minDet;
    }
}

void OnlineNomotoIdentifier::setPhysicalLimits(float newMinT, float newMaxT, float newMaxAbsK)
{
    if (newMinT > 0.0f && newMaxT > newMinT && newMaxAbsK > 0.0f &&
        isFiniteFloat(newMinT) && isFiniteFloat(newMaxT) && isFiniteFloat(newMaxAbsK))
    {
        minT = newMinT;
        maxT = newMaxT;
        maxAbsK = newMaxAbsK;
    }
}

void OnlineNomotoIdentifier::setUseDeltaAverage(bool enabled)
{
    useDeltaAverage = enabled;
}

bool OnlineNomotoIdentifier::update(unsigned long timeMs, float deltaNow, float rNow)
{
//	if (!isFiniteFloat(deltaNow) || !isFiniteFloat(rNow)) {
//        return valid;
//    }

    if (!hasPrevious) {
        lastTimeMs = timeMs;
        lastDelta = deltaNow;
        lastR = rNow;
        hasPrevious = true;
        return valid;
    }

    float dt = (timeMs - lastTimeMs) / 1000.0f;

    if (dt <= 0.5f) return valid;

    bool result = addRegressionSample(dt, deltaNow, rNow);

    lastTimeMs = timeMs;
    lastDelta = deltaNow;
    lastR = rNow;

    return result;
}

bool OnlineNomotoIdentifier::updateDt(float dt, float deltaNow, float rNow)
{
    if (!isFiniteFloat(dt) || !isFiniteFloat(deltaNow) || !isFiniteFloat(rNow)) {
        return valid;
    }

    if (!hasPrevious) {
        lastDelta = deltaNow;
        lastR = rNow;
        hasPrevious = true;
        return valid;
    }

    if (dt <= 0.0f) {
        return valid;
    }

    bool result = addRegressionSample(dt, deltaNow, rNow);

    lastDelta = deltaNow;
    lastR = rNow;

    return result;
}

bool OnlineNomotoIdentifier::addRegressionSample(float dt, float deltaNow, float rNow)
{
    /*
     * Regression row:
     *
     *     y  = r[k+1]
     *     x0 = r[k]
     *     x1 = delta[k] or average(delta[k], delta[k+1])
     *
     *     y = A*x0 + B*x1
     */
    const float x0 = lastR;
    const float x1 = useDeltaAverage ? 0.5f * (lastDelta + deltaNow) : lastDelta;
    const float y = rNow;

    if (valid) {
        const float predicted = A * x0 + B * x1;
        lastResidual = y - predicted;
    } else {
        lastResidual = 0.0f;
    }

    S00 *= forgettingFactor;
    S01 *= forgettingFactor;
    S11 *= forgettingFactor;
    q0  *= forgettingFactor;
    q1  *= forgettingFactor;
    yy  *= forgettingFactor;

    dtSum *= forgettingFactor;
    weightSum *= forgettingFactor;

    S00 += x0 * x0;
    S01 += x0 * x1;
    S11 += x1 * x1;
    q0  += x0 * y;
    q1  += x1 * y;
    yy  += y * y;

    dtSum += dt;
    weightSum += 1.0f;

    sampleCount++;

    return solve();
}

bool OnlineNomotoIdentifier::solve()
{
    valid = false;

    if (sampleCount < minimumSamples) {
        return valid;
    }

    if (weightSum <= 2.0f) {
        return valid;
    }

    const float det = S00 * S11 - S01 * S01;

    if (fabs(det) < minDeterminant || !isFiniteFloat(det)) {
        return valid;
    }

    const float newA = (S11 * q0 - S01 * q1) / det;
    const float newB = (-S01 * q0 + S00 * q1) / det;

    /*
     * Stable first-order discrete model:
     *     0 < A < 1
     */
    if (newA <= 0.0f || newA >= 1.0f ||
        !isFiniteFloat(newA) || !isFiniteFloat(newB))
    {
        return valid;
    }

    const float dtMean = dtSum / weightSum;

    if (dtMean <= 0.0f || !isFiniteFloat(dtMean)) {
        return valid;
    }

    const float newT = -dtMean / log(newA);

    if (newT < minT || newT > maxT || !isFiniteFloat(newT)) {
        return valid;
    }

    const float denom = 1.0f - newA;

    if (fabs(denom) < 1.0e-9f) {
        return valid;
    }

    const float newK = newB / denom;

    if (fabs(newK) > maxAbsK || !isFiniteFloat(newK)) {
        return valid;
    }

    if (fabs(newK) < 1.0e-9f) {
        return valid;
    }

    const float newKff = newT / newK;

    /*
     * Least-squares residual:
     *
     * SSE = y'y - theta'X'y
     *
     * theta = [A B]
     */
    float sse = yy - (newA * q0 + newB * q1);

    if (sse < 0.0f) {
        sse = 0.0f;
    }

    const float dof = weightSum - 2.0f;

    if (dof > 0.0f) {
        residualRms = sqrt(sse / dof);
    } else {
        residualRms = 0.0f;
    }

    A = newA;
    B = newB;
    T = newT;
    K = newK;
    Kff = newKff;

    valid = true;
    return valid;
}

bool OnlineNomotoIdentifier::isValid() const
{
    return valid;
}

bool OnlineNomotoIdentifier::hasPreviousSample() const
{
    return hasPrevious;
}

float OnlineNomotoIdentifier::getA() const
{
    return A;
}

float OnlineNomotoIdentifier::getB() const
{
    return B;
}

float OnlineNomotoIdentifier::getT() const
{
    return T;
}

float OnlineNomotoIdentifier::getK() const
{
    return K;
}

float OnlineNomotoIdentifier::getKff() const
{
    return Kff;
}

float OnlineNomotoIdentifier::getTeff() const
{
    return Teff;
}

float OnlineNomotoIdentifier::getKeff() const
{
    return Keff;
}

float OnlineNomotoIdentifier::getKffeff() const
{
    return Kffeff;
}

float OnlineNomotoIdentifier::getDtMean() const
{
    if (weightSum <= 0.0f) {
        return 0.0f;
    }

    return dtSum / weightSum;
}

unsigned long OnlineNomotoIdentifier::getSampleCount() const
{
    return sampleCount;
}

float OnlineNomotoIdentifier::getWeightSum() const
{
    return weightSum;
}

float OnlineNomotoIdentifier::predictNextR(float rCurrent, float deltaCurrent) const
{
    if (!valid) {
        return rCurrent;
    }

    return A * rCurrent + B * deltaCurrent;
}

float OnlineNomotoIdentifier::getLastResidual() const
{
    return lastResidual;
}

float OnlineNomotoIdentifier::getResidualRms() const
{
    return residualRms;
}

bool OnlineNomotoIdentifier::isFiniteFloat(float value) const
{
    return !isnan(value) && !isinf(value);
}

void OnlineNomotoIdentifier::scaleKTeff(float ratio)
{
// Ya viene con constrain aplicado
//    const float minRatio = 0.5f;  // evita ganancias absurdas a baja velocidad
//    const float maxRatio = 1.5f;  // evita extrapolar demasiado
//
//    if (ratio < minRatio) ratio = minRatio;
//    if (ratio > maxRatio) ratio = maxRatio;

    Teff = getT() / ratio;
    Keff = getK() * ratio;
    Kffeff = getKff() * ratio;
}

void OnlineNomotoIdentifier::setTK(float aT, float aK, float ratio) {
    T = aT;
    K = aK;
    Kff = aT/aK;
    scaleKTeff(ratio);

}
