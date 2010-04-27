#ifndef SRUKF_H
#define SRUKF_H

#include "Matrix.h"

class SRUKF
{
public:
    SRUKF();
    SRUKF(unsigned int numStates);
    SRUKF(const SRUKF& source);
    ~SRUKF();
    void CalculateSigmaWeights(float kappa = 1.0f);
    Matrix GenerateSigmaPoints() const;
    Matrix CalculateMeanFromSigmas(const Matrix& sigmaPoints) const;
    Matrix CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const;
    Matrix PerformTimeUpdate(const Matrix& updateSigmaPoints);
    void setMean(const Matrix& newMean) {m_mean = newMean;};
    void setCovariance(const Matrix& newCovariance) {m_sqrtCovariance = cholesky(newCovariance);};
    double getMean(int stateId) const;
    double calculateSd(int stateId) const;
    bool setState(Matrix mean, Matrix covariance);
    bool measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& predictedMeasurementSigmas, const Matrix& stateEstimateSigmas);

protected:
   unsigned int m_numStates;
   Matrix m_mean;
   Matrix m_sqrtCovariance;
   Matrix m_sigmaWeights;
   Matrix m_sqrtSigmaWeights;
   float m_kappa;
   float m_sigmaSqrtCovWeight;
};

#endif // SRUKF_H
