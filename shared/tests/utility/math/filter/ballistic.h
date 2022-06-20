#ifndef BALLISTIC_H
#define BALLISTIC_H

#include <Eigen/Core>

struct BallisticParameters
{
    const double p0   = 101.325e3;    // Air pressure at sea level [Pa]
    const double M    = 0.0289644;    // Molar mass of dry air [kg/mol]
    const double R    = 8.31447;      // Gas constant [J/(mol.K)]
    const double L    = 0.0065;       // Temperature gradient [K/m]
    const double T0   = 288.15;       // Temperature at sea level [K]
    const double g    = 9.81;         // Acceleration due to gravity [m/s^2]
    const double r1   = 5000;         // Horizontal position of sensor [m]
    const double r2   = 5000;         // Vertical position of sensor [m]
};

struct BallisticProcessModel
{
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f);
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ);
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ, Eigen::MatrixXd & dfdx);
};

struct BallisticMeasurementModel
{
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h);
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR);
    void operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR, Eigen::MatrixXd & dhdx);
};


#endif