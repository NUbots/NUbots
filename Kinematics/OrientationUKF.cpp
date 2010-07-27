#include "OrientationUKF.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "nubotdataconfig.h"
OrientationUKF::OrientationUKF(): UKF(numStates), m_initialised(false)
{
//    std::fstream file;
//    file.open((std::string(DATA_DIR) + std::string("OrientationUKF.log")).c_str(),ios_base::trunc | ios_base::out);
//    file.close();
}

void OrientationUKF::initialise(double time, const std::vector<float>& gyroReadings, const std::vector<float>& accelerations)
{
    m_timeOfLastUpdate = time;

    // Assume there is little or no motion to start for best intial estimate.
    m_mean[pitchGyroOffset][0] = gyroReadings[1];
    m_mean[rollGyroOffset][0] = gyroReadings[0];

    m_covariance[pitchGyroOffset][pitchGyroOffset] = 0.1f * 0.1f;
    m_covariance[rollGyroOffset][rollGyroOffset] = 0.1f * 0.1f;

    // Assume there is little to no acceleration apart from gravity for initial estimation.
    m_mean[pitchAngle][0] = -atan2(-accelerations[0],-accelerations[2]);
    m_mean[rollAngle][0] = atan2(accelerations[1],-accelerations[2]);

    m_covariance[pitchAngle][pitchAngle] = 0.5f*0.5f;
    m_covariance[rollAngle][rollAngle] = 0.5f * 0.5f;

    m_processNoise = Matrix(numStates,numStates,false);
    m_processNoise[pitchAngle][pitchAngle] = 1e-2;
    m_processNoise[pitchGyroOffset][pitchGyroOffset] = 1e-5;
    m_processNoise[rollAngle][rollAngle] = 1e-2;
    m_processNoise[rollGyroOffset][rollGyroOffset] = 1e-5;

    m_initialised = true;
}

void OrientationUKF::TimeUpdate(const std::vector<float>& gyroReadings, double timestamp)
{
//    std::fstream file;
//    file.open((std::string(DATA_DIR) + std::string("OrientationUKF.log")).c_str(),ios_base::app | ios_base::out);
//    file << "-- Time Update (" << timestamp << ") [" << rollGyroReading << "," << pitchGyroReading << "] --" << std::endl;
//    file << "Previous Mean:" << std::endl;
//    file << m_mean;
    //double startTime = System->getThreadTime();
    // Find delta 't', the time that has passed since the previous time update.
    const float dt = (timestamp - m_timeOfLastUpdate) / 1000.0f;

    // Store the current time for reference during next update.
    m_timeOfLastUpdate = timestamp;

    // Time Update Function is equation of the form (emitting accelerations):
    // x' = Ax + By
    //    = [ 1 -dt 0  0  ] [ pitchAngle      ] + [ dt  0  ] [ pitchGyroReading ]
    //      [ 0  1  0  0  ] [ pitchGyroOffset ]   [  0  0  ] [ rollGyroReading  ]
    //      [ 0  0  1 -dt ] [ rollAngle       ]   [  0  dt ]
    //      [ 0  0  0  1  ] [ rollGyroOffset  ]   [  0  0  ]

    // A Matrix
    Matrix A(numStates,numStates,true);
    A[0][1] = -dt;
    A[2][3] = -dt;

    // B Matrix
    Matrix B(numStates,2,false);
    B[0][0] = dt;
    B[2][1] = dt;

    // Sensor value matrix
    Matrix sensorData(2,1,false);
    sensorData[0][0] = gyroReadings[1];
    sensorData[1][0] = gyroReadings[0];

    // Generate the sigma points and update using transfer function
    Matrix sigmaPoints = GenerateSigmaPoints();
    m_updateSigmaPoints = Matrix(sigmaPoints.getm(), sigmaPoints.getn(), false);
    Matrix tempResult;
    for(int i = 0; i < sigmaPoints.getn(); i++)
    {
        tempResult = A*sigmaPoints.getCol(i) + B*sensorData;
        m_updateSigmaPoints.setCol(i,tempResult);
    }

    // Find the new mean
    m_mean = CalculateMeanFromSigmas(m_updateSigmaPoints);
    // Normalise the angles so they lie between +pi and -pi.
    m_mean[pitchAngle][0] = mathGeneral::normaliseAngle(m_mean[pitchAngle][0]);
    m_mean[rollAngle][0] = mathGeneral::normaliseAngle(m_mean[rollAngle][0]);

    // Find the new covariance
    m_covariance = CalculateCovarianceFromSigmas(m_updateSigmaPoints, m_mean) + m_processNoise;
    //double runTime = System->getThreadTime() - startTime;
    //debug << "TimeUpdate took: " << runTime << " ms" << std::endl;

//    file << "New Mean:" << std::endl;
//    file << m_mean << std::endl;
//    file.close();
}

void OrientationUKF::MeasurementUpdate(const std::vector<float>& accelerations, bool validKinematics, const std::vector<float>& kinematicsOrientation)
{
//    std::fstream file;
//    file.open((std::string(DATA_DIR) + std::string("OrientationUKF.log")).c_str(),ios_base::app | ios_base::out);
//    file << "-- Measurement Update [" << accelerations[0] << "," << accelerations[1] << ","<< accelerations[2] <<  "] ";
//    if(validKinematics)
//    {
//        file << "[" << kinematicsOrientation[0] << "," << kinematicsOrientation[1] << "," << kinematicsOrientation[2] << "] ";
//    }
//    file << "--" << std::endl;
//    file << "Previous Mean:" << std::endl;
//    file << m_mean << std::endl;

    //double startTime = System->getThreadTime();
    const float gravityAccel = 981.0f; // cm/s^2
    int numMeasurements = 3;
    if(validKinematics) numMeasurements+=2;

    // Generate sigma points from current state estimation.
    Matrix sigmaPoints = GenerateSigmaPoints();
    int numberOfSigmaPoints = sigmaPoints.getn();

    // List of predicted observation for each sigma point.
    Matrix predictedObservationSigmas(numMeasurements, numberOfSigmaPoints, false);

    // Put observation into matrix form so we can use if for doing math
    Matrix observation(numMeasurements,1,false);
    observation[0][0] = accelerations[0];
    observation[1][0] = accelerations[1];
    observation[2][0] = accelerations[2];
    if(validKinematics)
    {
        observation[3][0] = kinematicsOrientation[0];
        observation[4][0] = kinematicsOrientation[1];
    }

    // Observation noise
    Matrix S_Obs(numMeasurements,numMeasurements,true);
    //double accelNoise = 200.0*200.0;
    double accelVectorMag = sqrt(accelerations[0]*accelerations[0] + accelerations[1]*accelerations[1] + accelerations[2]*accelerations[2]);
    double errorFromIdealGravity = accelVectorMag - fabs(gravityAccel);
    double accelNoise = 25.0 + fabs(errorFromIdealGravity);
//    file << "Using accel noise: " << accelNoise << std::endl;
    accelNoise = accelNoise*accelNoise;

    S_Obs[0][0] = accelNoise;
    S_Obs[1][1] = accelNoise;
    S_Obs[2][2] = accelNoise;
    if(validKinematics)
    {
        double kinematicsNoise = 0.01*0.01;
        S_Obs[3][3] = kinematicsNoise;
        S_Obs[4][4] = kinematicsNoise;
    }

    // Temp working variables
    Matrix temp(numMeasurements,1,false);
    double pitch, roll;

    // Convert estimated state sigma points to estimates observation sigma points.
    for(int i = 0; i < numberOfSigmaPoints; i++)
    {
        pitch = mathGeneral::normaliseAngle(sigmaPoints[pitchAngle][i]);
        roll = mathGeneral::normaliseAngle(sigmaPoints[rollAngle][i]);

        // Calculate predicted x acceleration due to gravity + external acceleration.
        temp[0][0] = gravityAccel * sin(pitch);

        // Calculate predicted y acceleration due to gravity + external acceleration.
        temp[1][0] = -gravityAccel * sin(roll);

        // Calculate predicted z acceleration due to gravity + external acceleration.
        temp[2][0] = -gravityAccel * cos(pitch) * cos(roll);

        if(validKinematics)
        {
            // Calculate predicted measurement.
            temp[3][0] = roll;
            temp[4][0] = pitch;
        }

        // Add to measurement sigma point list.
        predictedObservationSigmas.setCol(i,temp);
    }

    measurementUpdate(observation, S_Obs, predictedObservationSigmas, sigmaPoints);
    m_mean[pitchAngle][0] = mathGeneral::normaliseAngle(m_mean[pitchAngle][0]);
    m_mean[rollAngle][0] = mathGeneral::normaliseAngle(m_mean[rollAngle][0]);
    //double runTime = System->getThreadTime() - startTime;
    //debug << "Acceleration Measurement Update took: " << runTime << " ms" << std::endl;
//    file << "Predicted Observation Mean:" << std::endl;
//    file << predictedObservationSigmas.getCol(0);
//    file << "Observation:" << std::endl << observation;
//    file << "New Mean:" << std::endl;
//    file << m_mean << std::endl;
//    file.close();
}
