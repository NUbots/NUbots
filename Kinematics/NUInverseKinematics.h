#ifndef H_NUINVERSEKINEMATICS_H_DEFINED
#define H_NUINVERSEKINEMATICS_H_DEFINED

/*!
    @file NUInverseKinematics.h
    @author Steven Nicklin
    @brief Declaration of NUInverseKinematics (abstract) class

    @class NUInverseKinematics
    @brief Base module for inverse kinematics calculations, specialised for each robot model.
 */

#include "Tools/Math/Matrix.h"


class NUInverseKinematics
{
public:
    virtual bool calculateLegJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)=0;
    virtual bool calculateArmJoints(const Matrix& leftPosition, const Matrix& rightPosition, std::vector<float>& jointPositions)=0;
};

#endif
