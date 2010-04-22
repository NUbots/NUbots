#include "AngleUKF.h"
#include "Tools/Math/General.h"
#include "debug.h"

using namespace std;

AngleUKF::AngleUKF(): UKF(NumStates)
{
    initialiseFilter(0.0);
}


void AngleUKF::initialiseFilter(double timestamp)
{
    m_timeOfLastUpdate = timestamp;
    // Initial angle estimate is zero.
    m_mean[Angle][0] = 0.0;
    // Initial gryro offset is 10.0.
    m_mean[GyroOffset][0] = 10.0;

    // Set the initial angle variance to pi^2 (means that it could be any angle)
    m_covariance[Angle][Angle] = mathGeneral::PI;
    // Set the gyro offset variance to 10.0^2 rad / sec
    m_covariance[GyroOffset][GyroOffset] = 10.0;

}

void AngleUKF::TimeUpdate(float gyroReading, double timestamp)
{
    // Find delta 't', the time that has passed since the previous time update.
    const float dt = timestamp - m_timeOfLastUpdate;
    // Store the current time for reference during next update.
    m_timeOfLastUpdate = timestamp;

    // Time Update Function is equation of the form:
    // ? = Ax + By
    //   = [ 1 -dt] [Theta     ] + [ dt ] [ gyroReading ]
    //     [ 0  1 ] [GyroOffset]   [  0 ]

    // A Matrix
    Matrix A(NumStates,NumStates,false);
    A[0][0] = 1;
    A[0][1] = -dt;
    A[1][0] = 0;
    A[1][1] = 1;

    // B Matrix
    Matrix B(NumStates,1,false);
    B[0][0] = dt;
    B[1][0] = 0;

    Matrix sigmaPoints = GenerateSigmaPoints();
    m_updateSigmaPoints = Matrix(sigmaPoints.getm(), sigmaPoints.getn(), false);
    Matrix tempResult;
    for(int i = 0; i < sigmaPoints.getn(); i++)
    {
        tempResult = A*sigmaPoints.getCol(i) + B*gyroReading;
        m_updateSigmaPoints.setCol(i,tempResult);
    }
    m_mean = CalculateMeanFromSigmas(m_updateSigmaPoints);
    // Normalise the angle.
    m_mean[Angle][0] = mathGeneral::normaliseAngle(m_mean[Angle][0]);
    m_covariance = CalculateCovarianceFromSigmas(m_updateSigmaPoints, m_mean);
    debug << "Time Update Complete." << endl << "Mean:" << endl << m_mean << endl;
    debug << "Covariance:" << endl << m_covariance << endl;
}


void AngleUKF::AccelerometerMeasurementUpdate(float horizontalAccel, float verticalAccel)
{
    const float gravityAccel = 980; // cm/s^2
    // Generate sigma points from current state estimation.
    Matrix sigmaPoints = GenerateSigmaPoints();
    int numberOfSigmaPoints = sigmaPoints.getn();

    debug << "Sigma Points:" << endl << sigmaPoints;

    // List of predicted observation for each sigma point.
    Matrix predictedObservationSigmas(2, numberOfSigmaPoints, false);

    // Put observation into matrix form
    Matrix observation(2,1,false);
    observation[0][0] = horizontalAccel;
    observation[1][0] = verticalAccel;

    Matrix S_Obs(2,2,false);
    S_Obs[0][0]=1;
    S_Obs[1][1]=1;

    // Temp working variables
    Matrix temp(2,1,false);
    double angleState;

    // Convert estimated state sigma points to estimates observation sigma points.
    for(int i = 0; i < numberOfSigmaPoints; i++)
    {
        angleState = mathGeneral::normaliseAngle(sigmaPoints[Angle][i]);

        // Calculate predicted horizontal acceleration due to gravity.
        temp[0][0] = gravityAccel * sin(angleState);

        // Calculate predicted vertical acceleration due to gravity.
        temp[1][0] = gravityAccel * cos(angleState);

        // Add to measurement sigma point list.
        predictedObservationSigmas.setCol(i,temp);
    }
    measurementUpdate(observation, S_Obs, predictedObservationSigmas, sigmaPoints);
    m_mean[Angle][0] = mathGeneral::normaliseAngle(m_mean[Angle][0]);
}


//
//void AngleUKF::AccelerometerMeasurementUpdate(float horizontalAccel, float verticalAccel)
//{
//    const float gravityAccel = 980; // cm/s^2
//    // Generate sigma points from current state estimation.
//    Matrix sigmaPoints = GenerateSigmaPoints();
//    int numberOfSigmaPoints = sigmaPoints.getn();
//
//    debug << "Sigma Points:" << endl << sigmaPoints;
//
//    // List of predicted observation for each sigma point.
//    Matrix predictedObservationSigmas(2, numberOfSigmaPoints, false);
//
//    // Put observation into matrix form
//    Matrix observation(2,1,false);
//    observation[0][0] = horizontalAccel;
//    observation[1][0] = verticalAccel;
//
//    // Temp working variables
//    Matrix temp(2,1,false);
//    double angleState;
//
//    Matrix predictedObservation(2,1,false);
//
//    // Convert estimated state sigma points to estimates observation sigma points.
//    for(unsigned int i = 0; i < numberOfSigmaPoints; i++)
//    {
//        angleState = mathGeneral::normaliseAngle(sigmaPoints[Angle][i]);
//
//        // Calculate predicted horizontal acceleration due to gravity.
//        temp[0][0] = gravityAccel * sin(angleState);
//
//        // Calculate predicted vertical acceleration due to gravity.
//        temp[1][0] = gravityAccel * cos(angleState);
//
//        // Add to measurement sigma point list.
//        predictedObservationSigmas.setCol(i,temp);
//    }
//
//    debug << "Predicted observations:" << endl << predictedObservationSigmas;
//
//    predictedObservation = CalculateMeanFromSigmas(predictedObservationSigmas);
//
//    debug << "Predicted observation (mean):" << endl << predictedObservation;
//    debug << "Observation:" << endl << observation;
//
//    Matrix Pyy(2,2,false);
//    Matrix Pxy(2,2,false);
//
//    for(int i =0; i < numberOfSigmaPoints; i++)
//    {
//        temp = predictedObservationSigmas.getCol(i) - predictedObservation;
//        Pyy = Pyy + m_sigmaWeights[0][i]*temp * temp.transp();
//        Pxy = Pxy + m_sigmaWeights[0][i]*(sigmaPoints.getCol(i) - m_mean) * temp.transp();
//    }
//    Matrix K = Pxy * Invert22(Pyy);
//
//    debug << "Pyy:" << endl << Pyy;
//
//    debug << "Pxy:" << endl << Pxy;
//
//
//    debug << "K:" << endl << K;
//
//    Matrix S_Obs(2,2,false);
//    S_Obs[0][0]=1;
//    S_Obs[1][1]=1;
//    m_mean  = m_mean + K * (observation - predictedObservation);
//    m_mean[Angle][0] = mathGeneral::normaliseAngle(m_mean[Angle][0]);
//
//    debug << "K*Pyy = " << endl << K*Pyy << endl;
//    debug << "K*Pyy*K.transp() = " << endl << K*Pyy*K.transp() << endl;
//
//    //m_covariance = m_covariance - K*Pyy*K.transp();
//
//    m_covariance = HT(horzcat(sigmaPoints-m_mean*m_sigmaWeights - K*predictedObservationSigmas +
//                              K*predictedObservation*m_sigmaWeights,K*S_Obs));
//
//    // Stolen from last years code... does not all seem right for this iplementation.
///*
//  WORKING (Sort of)
//    temp = predictedObservationSigmas - predictedObservation*m_sigmaWeights;
//    Matrix Pyy = temp * temp.transp();
//    Matrix Pxy = (sigmaPoints - m_mean * m_sigmaWeights) * temp.transp();
//    Matrix K = Pxy * Invert22(Pyy);
//
//    debug << "Pyy:" << endl << Pyy;
//
//    debug << "Pxy:" << endl << Pxy;
//
//
//    debug << "K:" << endl << K;
//    Matrix S_Obs(2,2,false);
//    S_Obs[0][0]=1;
//    S_Obs[1][1]=1;
//    m_mean  = m_mean + K * (observation - predictedObservation);
//    m_mean[Angle][0] = mathGeneral::normaliseAngle(m_mean[Angle][0]);
//    m_covariance = HT(horzcat(sigmaPoints-m_mean*m_sigmaWeights - K*predictedObservationSigmas +
//                              K*predictedObservation*m_sigmaWeights,K*S_Obs));
//                              */
//}
