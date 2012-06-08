#include "RobotMotionModel.h"

RobotMotionModel::RobotMotionModel()
{
}

RobotMotionModel::RobotMotionModel(const Matrix& linearTransform): m_linearTransform(linearTransform)
{
}

Matrix RobotMotionModel::applyOdometry(const Matrix &position, float x_odom, float y_odom, float turn_odom)
{
    Matrix result(position);
    float orientation = position[2][0];

    Matrix rawOdometry(3,1,false);
    rawOdometry[0][0] = x_odom;
    rawOdometry[1][0] = y_odom;
    rawOdometry[2][0] = turn_odom;

    Matrix transformedOdometry = m_linearTransform * rawOdometry;

    // x position
    result[0][0] += transformedOdometry[0][0] * cos(orientation) + transformedOdometry[1][0] * sin(orientation);
    // y position
    result[1][0] += transformedOdometry[0][0] * sin(orientation) + transformedOdometry[1][0] * cos(orientation);
    // orientation
    result[2][0] += transformedOdometry[2][0];

    return result;
}
