#define _USE_MATH_DEFINES
#include <cmath>

#include "iCanHazPolarCoordz.h"

void ICanHazPolarCoordz::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f)
{
    f.resize(2,1);
    f(0)    = std::atan2(x(0), x(1));
    f(1)    = x.norm();
}

void ICanHazPolarCoordz::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f, Eigen::MatrixXd &SR)
{
    operator()(x, f);
    SR.resize(f.size(),f.size());
    SR.fill(0.);
    SR(0,0)     = 1.0*M_PI/180;
    SR(1,1)     = 0.1;
}

void ICanHazPolarCoordz::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f, Eigen::MatrixXd &SR, Eigen::MatrixXd &J)
{
    operator()(x, f, SR);
    J.resize(f.size(),x.size());
    J(0,0)  =  x(1)/x.squaredNorm();
    J(0,1)  = -x(0)/x.squaredNorm();

    J(1,0)  =  x(0)/x.norm();
    J(1,1)  =  x(1)/x.norm();
}
