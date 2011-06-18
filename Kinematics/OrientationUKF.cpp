#include "OrientationUKF.h"
#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"

#include "debug.h"
#include "nubotdataconfig.h"


//#define DEBUG_ME
//#define LOG_ME
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

OrientationUKF::OrientationUKF(): UKF(numStates), m_initialised(false)
{
    #ifdef DEBUG_ME
        std::fstream file;
        file.open("/var/volatile/OrientationUKF.log",ios_base::trunc | ios_base::out);
        file.close();
    #endif
    #ifdef LOG_ME
        std::fstream logfile;
        logfile.open("/var/volatile/OrientationUKF.csv",ios_base::trunc | ios_base::out);
        logfile.close();
    #endif
    m_initialised_count = 0;
    m_scale = 0;
    m_offset = vector<float>(3,0);
}

void OrientationUKF::initialise(double time, const std::vector<float>& gyroReadings, const std::vector<float>& accelerations, bool validkinematics, const std::vector<float> kinematicorientation)
{
    float a_mag = sqrt(pow(accelerations[0],2) + pow(accelerations[1],2) + pow(accelerations[2],2));
    float a_roll = -atan2(accelerations[1], sqrt(pow(accelerations[0],2) + pow(accelerations[2],2)));
    float a_pitch = atan2(accelerations[0], sqrt(pow(accelerations[1],2) + pow(accelerations[2],2)));
    
    if (validkinematics and fabs(a_mag - 981) < 98.1 and fabs(a_pitch) < 0.2 and fabs(a_roll) < 0.2)
    {
        m_timeOfLastUpdate = time;
        
        vector<float> k_pred(3,0);
        float k_mag = OrientationUKF::g;
        float k_roll = kinematicorientation[0];
        float k_pitch = kinematicorientation[1];
        k_pred[2] = -sqrt(pow(OrientationUKF::g,2)/(1 + pow(tan(k_pitch),2) + pow(tan(k_roll),2)));   // Calculate predicted z acceleration due to gravity + external acceleration.
        k_pred[0] = -k_pred[2] * tan(k_pitch);                                     // Calculate predicted x acceleration due to gravity + external acceleration.
        k_pred[1] = k_pred[2] * tan(k_roll);                                       // Calculate predicted y acceleration due to gravity + external acceleration.
    
        m_scale += (a_mag/k_mag);
        m_offset = m_offset + accelerations - (a_mag/k_mag)*k_pred;
        
        m_initialised_count++;
        if (m_initialised_count > 50)
        {
            m_offset = (1.0/m_initialised_count)*m_offset;
            m_scale /= m_initialised_count;
            // Assume there is little or no motion to start for best intial estimate.
            m_mean[pitchGyroOffset][0] = gyroReadings[1];
            m_mean[rollGyroOffset][0] = gyroReadings[0];
            
            // Assume there is little to no acceleration apart from gravity for initial estimation.
            m_mean[pitchAngle][0] = k_pitch;
            m_mean[rollAngle][0] = k_roll;
            
            m_covariance[pitchAngle][pitchAngle] = 0.25f * 0.25f;
            m_covariance[pitchGyroOffset][pitchGyroOffset] = 0.01f * 0.01f;
            m_covariance[rollAngle][rollAngle] = 0.25f * 0.25f;
            m_covariance[rollGyroOffset][rollGyroOffset] = 0.01f * 0.01f;
            
            m_processNoise = Matrix(numStates,numStates,false);
            m_processNoise[pitchAngle][pitchAngle] = 1e-5;
            m_processNoise[pitchGyroOffset][pitchGyroOffset] = 1e-12;
            m_processNoise[rollAngle][rollAngle] = 1e-5;
            m_processNoise[rollGyroOffset][rollGyroOffset] = 1e-12;
            
            m_initialised = true;
            #ifdef DEBUG_ME
                std::fstream file;
                file.open("/var/volatile/OrientationUKF.log",ios_base::app | ios_base::out);
                file << "-- Initialise --" << endl;
                file << "Mean:" << std::endl;
                file << m_mean;
                file << "Offsets: " << m_offset << endl;
                file.close();
            #endif
        }
    }
}

void OrientationUKF::TimeUpdate(const std::vector<float>& gyroReadings, double timestamp)
{
    #ifdef DEBUG_ME
        std::fstream file;
        file.open("/var/volatile/OrientationUKF.log",ios_base::app | ios_base::out);
        file << "-- Time Update (" << timestamp << ") [" << gyroReadings[0] << "," << gyroReadings[1] << "] --" << std::endl;
        file << "Previous Mean:" << std::endl;
        file << m_mean;
    #endif

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

    #ifdef DEBUG_ME
        file << "New Mean:" << std::endl;
        file << m_mean << std::endl;
        file.close();
    #endif
}

void OrientationUKF::MeasurementUpdate(const std::vector<float>& accelerations, bool validKinematics, const std::vector<float>& kinematicsOrientation)
{
    #ifdef DEBUG_ME
        std::fstream file;
        file.open("/var/volatile/OrientationUKF.log",ios_base::app | ios_base::out);
        file << "-- Measurement Update [" << accelerations[0] << "," << accelerations[1] << ","<< accelerations[2] <<  "] ";
        if(validKinematics)
            file << "[" << kinematicsOrientation[0] << "," << kinematicsOrientation[1] << "," << kinematicsOrientation[2] << "] ";
        file << "--" << std::endl;
        file << "Previous Mean:" << std::endl;
        file << m_mean << std::endl;
    #endif

    int numMeasurements = 3;
    if(validKinematics) numMeasurements+=2;

    // Generate sigma points from current state estimation.
    Matrix sigmaPoints = GenerateSigmaPoints();
    int numberOfSigmaPoints = sigmaPoints.getn();

    // List of predicted observation for each sigma point.
    Matrix predictedObservationSigmas(numMeasurements, numberOfSigmaPoints, false);

    // Put observation into matrix form so we can use if for doing math
    Matrix observation(numMeasurements,1,false);
    observation[0][0] = accelerations[0] - m_offset[0];
    observation[1][0] = accelerations[1] - m_offset[1];
    observation[2][0] = accelerations[2] - m_offset[2];
    if(validKinematics)
    {
        observation[3][0] = kinematicsOrientation[0];
        observation[4][0] = kinematicsOrientation[1];
    }

    // Observation noise
    Matrix S_Obs(numMeasurements,numMeasurements,true);
    double accelVectorMag = sqrt(observation[0][0]*observation[0][0] + observation[1][0]*observation[1][0] + observation[2][0]*observation[2][0]);
    double errorFromIdealGravity = fabs(accelVectorMag - m_scale*g);
    double accelNoise = pow(50.0 + 10*errorFromIdealGravity, 2);

    S_Obs[0][0] = accelNoise;
    S_Obs[1][1] = accelNoise;
    S_Obs[2][2] = accelNoise;
    if(validKinematics)
    {
        double kinematicsNoise = 0.05*0.05;
        accelNoise *= 2;
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

        // Calculate predicted x acceleration due to gravity
        if (fabs(pitch) < mathGeneral::PI/2)
            temp[0][0] = accelVectorMag*tan(pitch)/sqrt(1+pow(tan(pitch),2));
        else
            temp[0][0] = -accelVectorMag*tan(pitch)/sqrt(1+pow(tan(pitch),2));
        
        // Calculate predicted y acceleration due to gravity
        if (fabs(roll) < mathGeneral::PI/2)
            temp[1][0] = -accelVectorMag*tan(roll)/sqrt(1+pow(tan(roll),2));
        else
            temp[1][0] = accelVectorMag*tan(roll)/sqrt(1+pow(tan(roll),2));
        
        // Calculate predicted z acceleration due to gravity
        float zsqrd = fabs(pow(accelVectorMag,2) - pow(temp[0][0],2) - pow(temp[1][0],2));
        if ((fabs(pitch) > mathGeneral::PI/2) xor (fabs(roll) > mathGeneral::PI/2))
            temp[2][0] = sqrt(zsqrd);
        else
            temp[2][0] = -sqrt(zsqrd);
        
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

    #ifdef DEBUG_ME
        file << "Predicted Observation Mean:" << std::endl;
        file << predictedObservationSigmas.getCol(0);
        file << "Observation:" << std::endl << observation;
        file << "New Mean:" << std::endl;
        file << m_mean << std::endl;
    #endif
    #ifdef LOG_ME
        std::fstream logfile;
        logfile.open("/var/volatile/OrientationUKF.csv",ios_base::app | ios_base::out);
        float a_roll = -atan2(observation[1][0], sqrt(pow(observation[0][0],2) + pow(observation[2][0],2)));
        float a_pitch = atan2(observation[0][0], -mathGeneral::sign(observation[2][0])*sqrt(pow(observation[1][0],2) + pow(observation[2][0],2)));
        logfile << m_timeOfLastUpdate << "," << m_mean[rollAngle][0] << "," << m_mean[pitchAngle][0] << "," << m_mean[rollGyroOffset][0] << "," << m_mean[pitchGyroOffset][0] << ",";
        if (validKinematics)
            logfile << kinematicsOrientation[0] << "," << kinematicsOrientation[1];
        else
            logfile << "0, 0";
        logfile << "," << a_roll << "," << a_pitch << ",";
        vector<float> alorientation;
        Blackboard->Sensors->get(NUSensorsData::OrientationHardware, alorientation);
        logfile << alorientation[0] << "," << alorientation[1] << endl;
        logfile.close();
    #endif
}
