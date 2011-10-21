#ifndef MEASUREMENTERROR_H
#define MEASUREMENTERROR_H
#include "Tools/Math/Matrix.h"

class MeasurementError
{
public:
    MeasurementError();
    MeasurementError(float distance_error, float heading_error);
    float distance() const {return m_distance;}
    float heading() const {return m_heading;}
    void setDistance(float newDistance) {m_distance = newDistance;}
    void setHeading(float newHeading) {m_heading = newHeading;}
    Matrix errorCovariance() const
    {
        Matrix result(2,2,false);
        result[0][0] = m_distance;
        result[1][1] = m_heading;
        return result;
    }

private:
    float m_distance;
    float m_heading;
};

#endif // MEASUREMENTERROR_H
