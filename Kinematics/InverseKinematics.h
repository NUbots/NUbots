#ifndef H_INVERSEKINEMATICS_H_DEFINED
#define H_INVERSEKINEMATICS_H_DEFINED

/*!
    @file NUInverseKinematics.h
    @author Steven Nicklin
    @brief Declaration of NUInverseKinematics (abstract) class

    @class NUInverseKinematics
    @brief Base module for inverse kinematics calculations, specialised for each robot model.
 */

#include "Tools/Math/Matrix.h"
#include "DarwinInverseKinematics.h"
#include "NAOInverseKinematics.h"

class InverseKinematics
{
public:
static NUInverseKinematics* getInverseKinematicModel()
{
    static NUInverseKinematics* model = new DarwinInverseKinematics();
    return model;
}
};

#endif
