#include "UKF.h"
#include <QDebug>
#include <iostream>
#include <sstream>
using namespace std;


UKF::UKF(): m_numStates(0)
{
}

UKF::UKF(unsigned int numStates): m_numStates(numStates)
{
    m_mean = Matrix(m_numStates,1,false);
    m_covariance = Matrix(m_numStates,m_numStates,true);
}

UKF::UKF(const UKF& source)
{
    m_numStates = source.m_numStates;
    m_mean = source.m_mean;
    m_covariance = source.m_covariance;
}

UKF::~UKF()
{
}

Matrix UKF::CalculateSigmaPointsMean(const Matrix& sigmaPoints) const
{
    unsigned int numPoints = sigmaPoints.getn();
    float kappa = 1.0;
    Matrix mean(sigmaPoints.getm(),1,false);
    Matrix sigmaWeights = GenerateSigmaWeights(kappa);
    for(unsigned int i = 0; i < numPoints; i++)
    {
        mean = mean + sigmaWeights[0][i]*sigmaPoints.getCol(i);
    }
    return mean;
}

Matrix UKF::CalculateSigmaPointsCovariance(const Matrix& sigmaPoints, const Matrix& mean) const
{
    unsigned int numPoints = sigmaPoints.getn();
    Matrix covariance(m_numStates,m_numStates, false);
    Matrix sigmaWeights = GenerateSigmaWeights();
    Matrix diff;
    stringstream buffer (stringstream::in | stringstream::out);
    for(unsigned int i = 0; i < numPoints; i++)
    {
        diff = sigmaPoints.getCol(i) - mean;
        buffer << sigmaWeights[0][i]*diff*diff.transp() << endl;
        covariance = covariance + sigmaWeights[0][i]*diff*diff.transp();

    }
    qDebug() << buffer.str().c_str();
    return covariance;
}

Matrix UKF::GenerateSigmaWeights(float kappa) const
{
    unsigned int numPoints = 2*m_numStates + 1;
    Matrix sigmaWeights(1,numPoints, false);
    double meanWeight = kappa/(m_numStates+kappa);
    double outerWeight = 1.0/(2*(m_numStates+kappa));
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
    float kappa = m_kappa;
   
    Matrix sigmaPoints(m_mean.getm(), numberOfSigmaPoints, false);

    sigmaPoints.setCol(0,m_mean); // First sigma point is the current mean with no deviation
    Matrix deviation;
    for(unsigned int i = 1; i < m_numStates + 1; i++){
        int negIndex = i+m_numStates;
        deviation = m_covariance.getCol(i - 1);               // Get weighted deviation
        sigmaPoints.setCol(i, (m_mean + deviation));                // Add mean + deviation
        sigmaPoints.setCol(negIndex, (m_mean - deviation));  // Add mean - deviation
    }
    return sigmaPoints;
}

double UKF::getMean(int stateId) const
{
    return m_mean[stateId][0];
}

bool UKF::setState(Matrix mean, Matrix covariance)
{
    if( (mean.getm() == covariance.getm()) && (mean.getm() == covariance.getn()) )
    {
        m_numStates = mean.getm();
        m_mean = mean;
        m_covariance = covariance;
        return true;
    }
    else
    {
        return false;
    }
}
