#ifndef ROBOTMOTIONMODEL_H
#define ROBOTMOTIONMODEL_H

#include "Tools/Math/Matrix.h"

class RobotMotionModel
{
public:
    RobotMotionModel();
    RobotMotionModel(const Matrix& linearTransform);
    Matrix applyOdometry(const Matrix& position, float x_odom, float y_odom, float turn_odom);
    Matrix calculateVariance(float x_odom, float y_odom, float turn_odom);
private:
    Matrix m_linearTransform;
};

#endif // ROBOTMOTIONMODEL_H
