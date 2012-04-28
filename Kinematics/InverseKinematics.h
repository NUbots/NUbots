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
#include "Autoconfig/motionconfig.h"

#if defined(USE_MODEL_DARWIN)
#include "DarwinInverseKinematics.h"
#elif defined(USE_MODEL_NAO)
#include "NAOInverseKinematics.h"
#else
#include "NUInverseKinematics.h"
#endif


class InverseKinematics
{
public:
static NUInverseKinematics* getInverseKinematicModel()
{
#if defined(USE_MODEL_NAO)
    static NUInverseKinematics* model = new NAOInverseKinematics();
#elif defined(USE_MODEL_DARWIN)
    static NUInverseKinematics* model = new DarwinInverseKinematics();
#else
    static NUInverseKinematics* model = NULL;
#endif
    return model;
}
};

#endif
