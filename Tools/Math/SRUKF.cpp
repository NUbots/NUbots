#include "SRUKF.h"
#include "debug.h"
using namespace std;


SRUKF::SRUKF(): m_numStates(0)
{
}

SRUKF::SRUKF(unsigned int numStates): m_numStates(numStates)
{
    m_mean = Matrix(m_numStates,1,false);
    m_sqrtCovariance = Matrix(m_numStates,m_numStates,true);
    CalculateSigmaWeights();
}

SRUKF::SRUKF(const SRUKF& source)
{
    m_numStates = source.m_numStates;
    m_mean = source.m_mean;
    m_sqrtCovariance = source.m_sqrtCovariance;
    CalculateSigmaWeights();
}

SRUKF::~SRUKF()
{
}

Matrix SRUKF::CalculateMeanFromSigmas(const Matrix& sigmaPoints) const
{
    //unsigned int numPoints = sigmaPoints.getn();
    Matrix mean(sigmaPoints.getm(),1,false);
    mean = sigmaPoints * m_sigmaWeights.transp();
    /*
    for(unsigned int i = 0; i < numPoints; i++)
    {
        mean = mean + m_sigmaWeights[0][i]*sigmaPoints.getCol(i);
    }
    */
    return mean;
}

Matrix SRUKF::CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const
{

    unsigned int numPoints = sigmaPoints.getn();
    Matrix diff;
    Matrix temp(m_numStates, numPoints, false);
    //debug << "mean" << endl << mean << endl;
    for(unsigned int i = 0; i < numPoints; i++)
    {
        //debug << "sigma" << endl << sigmaPoints.getCol(i) << endl;
        diff = sigmaPoints.getCol(i) - mean;
        //debug << "diff" << endl << diff << endl;
        temp.setCol(i, m_sqrtSigmaWeights[0][i] * diff);
    }
    debug << "temp" << endl << temp << endl;
    return HT(temp);
}

void SRUKF::CalculateSigmaWeights(float kappa)
{
    unsigned int numPoints = 2*m_numStates + 1;
    m_sigmaWeights = Matrix(1,numPoints, false);
    m_sqrtSigmaWeights = Matrix(1,numPoints, false);

    double meanWeight = kappa/(m_numStates+kappa);
    double outerWeight = (1.0-meanWeight)/(2*m_numStates);
    m_sigmaSqrtCovWeight = sqrt(m_numStates+kappa);

    // First weight
    m_sigmaWeights[0][0] = meanWeight;
    m_sqrtSigmaWeights[0][0] = sqrt(meanWeight);
    // The rest
    for(unsigned int i = 1; i < numPoints; i++)
    {
        m_sigmaWeights[0][i] = outerWeight;
        m_sqrtSigmaWeights[0][i] = sqrt(outerWeight);
    }
}

Matrix SRUKF::GenerateSigmaPoints() const
{
    int numberOfSigmaPoints = 2*m_numStates+1;
    Matrix sigmaPoints(m_mean.getm(), numberOfSigmaPoints, false);

    sigmaPoints.setCol(0,m_mean); // First sigma point is the current mean with no deviation
    Matrix deviation;
//    Matrix sqtCovariance = cholesky(m_numStates / (1-m_sigmaWeights[0][0]) * m_covariance);

    for(unsigned int i = 1; i < m_numStates + 1; i++){
        int negIndex = i+m_numStates;
        //deviation = sqtCovariance.getCol(i - 1);              // Get weighted deviation

        deviation = m_sigmaSqrtCovWeight*m_sqrtCovariance.getCol(i-1);  // Get weighted deviation
        sigmaPoints.setCol(i, (m_mean + deviation));            // Add mean + deviation
        sigmaPoints.setCol(negIndex, (m_mean - deviation));     // Add mean - deviation
    }
    return sigmaPoints;
}

double SRUKF::getMean(int stateId) const
{
    return m_mean[stateId][0];
}

double SRUKF::calculateSd(int stateId) const
{
    return m_sqrtCovariance[stateId][stateId];
}

bool SRUKF::setState(Matrix mean, Matrix sqrtCovariance)
{
    if( (mean.getm() == sqrtCovariance.getm()) && (mean.getm() == sqrtCovariance.getn()) )
    {
        m_numStates = mean.getm();
        m_mean = mean;
        m_sqrtCovariance = sqrtCovariance;
        CalculateSigmaWeights();
        return true;
    }
    else
    {
        return false;
    }
}

bool SRUKF::measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& predictedMeasurementSigmas, const Matrix& stateEstimateSigmas)
{

    int numberOfSigmaPoints = stateEstimateSigmas.getn();
    debug << "Predicted measurement sigmas:" << endl << predictedMeasurementSigmas;

    Matrix predictedMeasurement = CalculateMeanFromSigmas(predictedMeasurementSigmas);

    Matrix Mz(m_numStates,numberOfSigmaPoints, false);
    Matrix Mx(m_numStates,numberOfSigmaPoints, false);

    for(int i = 0; i < numberOfSigmaPoints; i++)
    {
        Mz.setCol(i, m_sqrtSigmaWeights[0][i] * (predictedMeasurementSigmas.getCol(i) - predictedMeasurement));
        Mx.setCol(i, m_sqrtSigmaWeights[0][i] * (stateEstimateSigmas.getCol(i) - m_mean));
    }

    Matrix Sz = horzcat(Mz,measurementNoise);
    Matrix Pxz = Mx*Mz.transp();
    Matrix K = Pxz * Invert22(Sz*Sz.transp());

    debug << "K:" << endl << K;
    m_mean  = m_mean + K * (measurement - predictedMeasurement);

    m_sqrtCovariance = HT(horzcat(Mx-K*Mz,K*measurementNoise));
    //m_covariance = HT(horzcat(sigmaPoints-m_mean*m_sigmaWeights - K*predictedObservationSigmas +
    //                          K*predictedObservation*m_sigmaWeights,K*measurementNoise));
    return true;
}
