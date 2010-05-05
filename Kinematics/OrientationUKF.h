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
        xAcceleration,
        yAcceleration,
        zAcceleration,
        numStates
    };
    void initialise(double time, float pitchGyro, float rollGyro, float accX, float accY, float accZ);
    void TimeUpdate(float pitchGyroReading, float rollGyroReading, double timestamp);
    void AccelerometerMeasurementUpdate(float xAccel, float yAccel, float zAccel);
    void KinematicsMeasurementUpdate(float pitchMeasurement, float rollMeasurement);
    bool Initialised(){return m_initialised;};

private:
    double m_timeOfLastUpdate;
    Matrix m_updateSigmaPoints;
    bool m_initialised;
};

#endif // ORIENTATIONUKF_H
