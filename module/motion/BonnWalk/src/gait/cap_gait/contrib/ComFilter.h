// clang-format off
#ifndef COMFILTER_H_
#define COMFILTER_H_

#include "KalmanFilter.h"
#include "LimpState.h"
#include "Limp.h"
#include "../cap_gait_config.h"

namespace margait_contrib
{

class ComFilter
{
    KalmanFilter lateralKf;
    KalmanFilter sagittalKf;

public:
    LimpState limpState;

    double smoothingX;
    double smoothingY;

public:
    ComFilter();

    void reset(double x=0, double vx=0, double ax=0, double y=0, double vy=0, double ay=0);

    LimpState update(qglviewer::Vec z);

    void setTimeStep(double dt);
};

}

#endif // COMFILTER_H_
