#ifndef ORIENTATIONUKF_H
#define ORIENTATIONUKF_H

#include "Tools/Math/UKF.h"

class OrientationUKF : public UKF
{
public:
    OrientationUKF();
    enum State
    {
        pitchAngle,
        pitchGyroOffset,
        rollAngle,
        rollGyroOffset,
        numStates
    };
    void initialise(double time, const std::vector<float>& gyroReadings, const std::vector<float>& accelerations);
    void TimeUpdate(const std::vector<float>& gyroReadings, double timestamp);
    void MeasurementUpdate(const std::vector<float>& accelerations, bool validKinematics, const std::vector<float>& kinematicsOrientation);
    bool Initialised(){return m_initialised;};

private:
    double m_timeOfLastUpdate;
    Matrix m_updateSigmaPoints;
    Matrix m_processNoise;
    bool m_initialised;
};

#endif // ORIENTATIONUKF_H
