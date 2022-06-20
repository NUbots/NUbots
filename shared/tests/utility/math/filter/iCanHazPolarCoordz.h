#ifndef ICANHAZPOLARCOORDZ_H
#define ICANHAZPOLARCOORDZ_H

#include <Eigen/Core>

struct ICanHazPolarCoordz
{
    void operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f);
    void operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f, Eigen::MatrixXd &SR);
    void operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f, Eigen::MatrixXd &SR, Eigen::MatrixXd &J);
};

#endif