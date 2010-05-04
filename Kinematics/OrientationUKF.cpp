#include "OrientationUKF.h"
#include "Tools/Math/General.h"
#include "debug.h"

OrientationUKF::OrientationUKF(): UKF(numStates), m_initialised(false)
{
}

void OrientationUKF::initialise(double timestamp, float pitchGyro, float rollGyro, float accX, float accY, float accZ)
{
    m_timeOfLastUpdate = timestamp;

    // Assume there is little or no motion to start for best intial estimate.
    m_mean[pitchGyroOffset][0] = pitchGyro;
    m_mean[rollGyroOffset][0] = rollGyro;

    m_covariance[pitchGyroOffset][pitchGyroOffset] = 0.1f;
    m_covariance[rollGyroOffset][rollGyroOffset] = 0.1f;

    debug << "Accel Values: [" << accX << ","  << accY << "," << accZ <<"]" << std::endl;
    // Assume there is little to no acceleration apart from gravity for initial estimation.
    m_mean[pitchAngle][0] = -atan2(-accX,-accZ);
    m_mean[rollAngle][0] = atan2(accY,-accZ);

    m_mean[xAcceleration][0] = 0.0;
    m_mean[yAcceleration][0] = 0.0;
    m_mean[zAcceleration][0] = 0.0;


    debug << "Angles: [" << m_mean[pitchAngle][0] << "," << m_mean[rollAngle][0] <<"]" << std::endl;

    // Determine starting covariance (error)
    const float idealGravityAccelVectorLength = 981.0f;
    const float accelVectorLength = sqrt(pow(accX,2) + pow(accY,2) + pow(accZ,2));
    const float accelVectorLengthError = accelVectorLength - idealGravityAccelVectorLength;
    const float errorPercentage = fabs(accelVectorLengthError/idealGravityAccelVectorLength);
    const float estAngleError = errorPercentage * mathGeneral::PI / 2;
    m_covariance[pitchAngle][pitchAngle] = 0.5;
    m_covariance[rollAngle][rollAngle] = 0.5;

    m_covariance[xAcceleration][xAcceleration] = 10.0f;
    m_covariance[yAcceleration][yAcceleration] = 10.0f;
    m_covariance[zAcceleration][zAcceleration] = 10.0f;

    debug << "Orientation Filter Initialised" << std::endl;
    debug << m_mean << std::endl;
    m_initialised = true;
}

void OrientationUKF::TimeUpdate(float pitchGyroReading, float rollGyroReading, double timestamp)
{
    // Find delta 't', the time that has passed since the previous time update.
    const float dt = (timestamp - m_timeOfLastUpdate) / 1000.0f;

    // Store the current time for reference during next update.
    m_timeOfLastUpdate = timestamp;

    // Time Update Function is equation of the form:
    // x' = Ax + By
    //    = [ 1 -dt 0  0  ] [ pitchAngle      ] + [ dt  0  ] [ pitchGyroReading ]
    //      [ 0  1  0  0  ] [ pitchGyroOffset ]   [  0  0  ] [ rollGyroReading  ]
    //      [ 0  0  1 -dt ] [ rollAngle       ]   [  0  dt ]
    //      [ 0  0  0  1  ] [ rollGyroOffset  ]   [  0  0  ]

    // A Matrix
    Matrix A(numStates,numStates,true);
    A[0][0] = 1;
    A[0][1] = -dt;
    A[1][1] = 1;
    A[2][2] = 1;
    A[2][3] = -dt;
    A[3][3] = 1;

    // B Matrix
    Matrix B(numStates,2,false);
    B[0][0] = dt;
    B[2][1] = dt;

    // Sensor value matrix
    Matrix sensorData(2,1,false);
    sensorData[0][0] = pitchGyroReading;
    sensorData[1][0] = rollGyroReading;

    debug << "Time Update: Time elapsed = " << dt << std::endl;
    debug << "sensorData" << std::endl << sensorData;

    // Generate the sigma points and update using transfer function
    Matrix sigmaPoints = GenerateSigmaPoints();
    m_updateSigmaPoints = Matrix(sigmaPoints.getm(), sigmaPoints.getn(), false);
    Matrix tempResult;
    for(int i = 0; i < sigmaPoints.getn(); i++)
    {
        tempResult = A*sigmaPoints.getCol(i) + B*sensorData;
        m_updateSigmaPoints.setCol(i,tempResult);
    }

    Matrix processNoise(numStates,numStates,true);
    processNoise = 0.1* processNoise;
    processNoise[pitchGyroOffset][pitchGyroOffset] = 1.0;
    processNoise[rollGyroOffset][rollGyroOffset] = 1.0;
    processNoise[xAcceleration][xAcceleration] = 50.0;
    processNoise[yAcceleration][yAcceleration] = 50.0;
    processNoise[zAcceleration][zAcceleration] = 50.0;

    // Find the new mean
    m_mean = CalculateMeanFromSigmas(m_updateSigmaPoints);
    // Normalise the angles.
    m_mean[pitchAngle][0] = mathGeneral::normaliseAngle(m_mean[pitchAngle][0]);
    m_mean[rollAngle][0] = mathGeneral::normaliseAngle(m_mean[rollAngle][0]);
    // Find the new covariance
    m_covariance = CalculateCovarianceFromSigmas(m_updateSigmaPoints, m_mean);
    //m_covariance=HT(horzcat(m_covariance, processNoise));

    debug << "Time Update Complete." << std::endl << "Mean:" << std::endl << m_mean << std::endl;
    //debug << "Covariance:" << endl << m_sqrtCovariance << endl;
    //debug << "Covariance:" << endl << m_covariance << endl;
}

void OrientationUKF::AccelerometerMeasurementUpdate(float xAccel, float yAccel, float zAccel)
{
    const int numMeasurements = 3;
    const float gravityAccel = 981; // cm/s^2
    // Generate sigma points from current state estimation.
    Matrix sigmaPoints = GenerateSigmaPoints();
    int numberOfSigmaPoints = sigmaPoints.getn();

    //debug << "Sigma Points:" << endl << sigmaPoints;

    // List of predicted observation for each sigma point.
    Matrix predictedObservationSigmas(numMeasurements, numberOfSigmaPoints, false);

    // Put observation into matrix form
    Matrix observation(numMeasurements,1,false);
    observation[0][0] = xAccel;
    observation[1][0] = yAccel;
    observation[2][0] = zAccel;

    const float idealGravityAccelVectorLength = 981.0f;
    const float accelVectorLength = sqrt(pow(xAccel,2) + pow(yAccel,2) + pow(zAccel,2));
    const float accelVectorLengthError = accelVectorLength - idealGravityAccelVectorLength;

    Matrix S_Obs(numMeasurements,numMeasurements,true);
    S_Obs = 1.0*S_Obs;

    // Temp working variables
    Matrix temp(3,1,false);
    double pitch, roll, accX, accY, accZ;

    // Convert estimated state sigma points to estimates observation sigma points.
    for(int i = 0; i < numberOfSigmaPoints; i++)
    {
        pitch = mathGeneral::normaliseAngle(sigmaPoints[pitchAngle][i]);
        roll = mathGeneral::normaliseAngle(sigmaPoints[rollAngle][i]);
        roll = mathGeneral::normaliseAngle(sigmaPoints[rollAngle][i]);
        accX = sigmaPoints[xAcceleration][i];
        accY = sigmaPoints[yAcceleration][i];
        accZ = sigmaPoints[zAcceleration][i];

        // Calculate predicted x acceleration due to gravity.
        temp[0][0] = gravityAccel * sin(pitch) + accX;

        // Calculate predicted y acceleration due to gravity.
        temp[1][0] = gravityAccel * cos(pitch) * sin(roll) + accY;

        // Calculate predicted z acceleration due to gravity.
        temp[2][0] = -gravityAccel * cos(pitch) * cos(roll) + accZ;

        // Add to measurement sigma point list.
        predictedObservationSigmas.setCol(i,temp);
    }
    debug << "observation" << std::endl << observation;
    debug << "predictedObservationSigmas" << std::endl << predictedObservationSigmas;
    debug << "covariance" << std::endl << m_covariance;
    measurementUpdate(observation, S_Obs, predictedObservationSigmas, sigmaPoints);
    m_mean[pitchAngle][0] = mathGeneral::normaliseAngle(m_mean[pitchAngle][0]);
    m_mean[rollAngle][0] = mathGeneral::normaliseAngle(m_mean[rollAngle][0]);

    debug << "Accelerometer Update Complete." << std::endl << "Mean:" << std::endl << m_mean << std::endl;
}

void OrientationUKF::KinematicsMeasurementUpdate(float pitchAngle, float rollAngle)
{
}
