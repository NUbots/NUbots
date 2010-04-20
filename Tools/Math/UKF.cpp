#include "UKF.h"


UKF::UKF(): m_numStates(0)
{
}

UKF::UKF(unsigned int numStates): m_numStates(numStates)
{
    m_mean = Matrix(m_numStates,1,false);
    m_covariance = Matrix(m_numStates,m_numStates,true);
    m_sigmaWeights = GenerateSigmaWeights();
}

UKF::UKF(const UKF& source)
{
    m_numStates = source.m_numStates;
    m_mean = source.m_mean;
    m_covariance = source.m_covariance;
    m_sigmaWeights = GenerateSigmaWeights();
}

UKF::~UKF()
{
}

Matrix UKF::CalculateMeanFromSigmas(const Matrix& sigmaPoints) const
{
    unsigned int numPoints = sigmaPoints.getn();
    Matrix mean(sigmaPoints.getm(),1,false);
    for(unsigned int i = 0; i < numPoints; i++)
    {
        mean = mean + m_sigmaWeights[0][i]*sigmaPoints.getCol(i);
    }
    return mean;
}

Matrix UKF::CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const
{
    unsigned int numPoints = sigmaPoints.getn();
    Matrix covariance(m_numStates,m_numStates, false);
    Matrix diff;
    for(unsigned int i = 0; i < numPoints; i++)
    {
        diff = sigmaPoints.getCol(i) - mean;
        covariance = covariance + m_sigmaWeights[0][i]*diff*diff.transp();
    }
    return covariance;
}

Matrix UKF::GenerateSigmaWeights(float kappa) const
{
    unsigned int numPoints = 2*m_numStates + 1;
    Matrix sigmaWeights(1,numPoints, false);
    double meanWeight = kappa/(m_numStates+kappa);
    double outerWeight = (1.0-meanWeight)/(2*m_numStates);
    sigmaWeights[0][0] = meanWeight;
    for(unsigned int i = 1; i < numPoints; i++)
    {
        sigmaWeights[0][i] = outerWeight;
    }
    return sigmaWeights;
}

Matrix UKF::GenerateSigmaPoints() const
{
    int numberOfSigmaPoints = 2*m_numStates+1;
    Matrix sigmaPoints(m_mean.getm(), numberOfSigmaPoints, false);

    sigmaPoints.setCol(0,m_mean); // First sigma point is the current mean with no deviation
    Matrix deviation;
    Matrix sqtCovariance = cholesky(m_numStates / (1-m_sigmaWeights[0][0]) * m_covariance);

    for(unsigned int i = 1; i < m_numStates + 1; i++){
        int negIndex = i+m_numStates;
        deviation = sqtCovariance.getCol(i - 1);        // Get weighted deviation
        sigmaPoints.setCol(i, (m_mean + deviation));                // Add mean + deviation
        sigmaPoints.setCol(negIndex, (m_mean - deviation));  // Add mean - deviation
    }
    return sigmaPoints;
}

double UKF::getMean(int stateId) const
{
    return m_mean[stateId][0];
}

double UKF::calculateSd(int stateId) const
{
    return sqrt(m_covariance[stateId][stateId]);
}

bool UKF::setState(Matrix mean, Matrix covariance)
{
    if( (mean.getm() == covariance.getm()) && (mean.getm() == covariance.getn()) )
    {
        m_numStates = mean.getm();
        m_mean = mean;
        m_covariance = covariance;
        m_sigmaWeights = GenerateSigmaWeights();
        return true;
    }
    else
    {
        return false;
    }
}
