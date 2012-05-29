#pragma once

#include "UKF.h"

class MobileObjectUKF : public UKF
{
public:
    enum State
    {
        x_pos,
        y_pos,
        x_vel,
        y_vel,
        total_states
    };

    MobileObjectUKF();
    ~MobileObjectUKF();
    Matrix processEquation(const Matrix& sigma_point, double deltaT, const Matrix& measurement);
    Matrix measurementEquation(const Matrix& sigma_point, const Matrix& measurementArgs);
protected:
    float m_velocity_decay;
};
