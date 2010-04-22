#ifndef ANGLEUKF_H
#define ANGLEUKF_H
#include "Tools/Math/UKF.h"

class AngleUKF : public UKF
{
public:
    AngleUKF();
    enum State
    {
        Angle,
        GyroOffset,
        NumStates
    };
    void initialiseFilter(double timestamp);
    void TimeUpdate(float gyroReading, double timestamp);
    void AccelerometerMeasurementUpdate(float horizontalAccel, float verticalAccel);

private:
    double m_timeOfLastUpdate;
    Matrix m_updateSigmaPoints;

};

#endif // ANGLEUKF_H
