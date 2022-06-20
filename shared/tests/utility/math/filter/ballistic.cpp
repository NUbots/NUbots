
#include "ballistic.h"
#include <iostream>

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// BallisticProcessModel
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f)
{
    const double & p0   = param.p0;
    const double & M    = param.M;
    const double & R    = param.R;
    const double & L    = param.L;
    const double & T0   = param.T0;
    const double & g    = param.g;
    // TODO: mean function, x = [v;d-g;0]
    f = Eigen::MatrixXd::Zero(x.rows(),1);
    f.resize(x.rows(),1);
    f(0) = x(1);
    f(1) = 0.5*(M*p0/R)*(1/(T0-L*x(0)))*std::pow((1-(L*x(0))/T0),(g*M)/(R*L))*x(1)*x(1)*x(2) - g;
    f(2) = 0;
    
}

void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ)
{
    const double & p0   = param.p0;
    const double & M    = param.M;
    const double & R    = param.R;
    const double & L    = param.L;
    const double & T0   = param.T0;
    const double & g    = param.g;
    // TODO: mean function, x = [v;d-g;0]
    f = Eigen::MatrixXd::Zero(x.rows(),1);
    f.resize(x.rows(),1);
    f(0) = x(1);
    f(1) = 0.5*(M*p0/R)*(1/(T0-L*x(0)))*std::pow((1-(L*x(0))/T0),(g*M)/(R*L))*x(1)*x(1)*x(2) - g;
    f(2) = 0;
    // TODO: upper Cholesky factor of process covariance
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(x.rows(),x.rows());
    Q(0,0) = 0;
    Q(1,1) = 1e-10;
    Q(2,2) = 5.0000e-06;
    SQ = Q;
}

void BallisticProcessModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & f, Eigen::MatrixXd & SQ, Eigen::MatrixXd & dfdx)
{
    const double & p0   = param.p0;
    const double & M    = param.M;
    const double & R    = param.R;
    const double & L    = param.L;
    const double & T0   = param.T0;
    const double & g    = param.g;
    // TODO: mean function, x = [v;d-g;0]
    f = Eigen::MatrixXd::Zero(x.rows(),1);
    f.resize(x.rows(),1);
    f(0) = x(1);
    f(1) = 0.5*(M*p0/R)*(1/(T0-L*x(0)))*std::pow((1-(L*x(0))/T0),(g*M)/(R*L))*x(1)*x(1)*x(2) - g;
    f(2) = 0;
    // TODO: upper Cholesky factor of process covariance
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(x.rows(),x.rows());
    Q(0,0) = 0;
    Q(1,1) = 1e-10;
    Q(2,2) = 5.0000e-06;
    SQ = Q;
    // TODO: Jacobian of mean dynamics w.r.t x.
    dfdx.resize(x.rows(),x.rows());
    //Row 1
    dfdx(0,0) = 0;
    dfdx(0,1) = 1;
    dfdx(0,2) = 0;
    //Row 2
    dfdx(1,0) = (L*M*p0*x(1)*x(1)*x(2)*std::pow((1 - (L*x(0))/T0),((M*g)/(L*R))))/(2*R*(T0 - L*x(0))*(T0 - L*x(0))) - 
    (M*M*g*p0*x(1)*x(1)*x(2)*std::pow((1 - (L*x(0))/T0),(((M*g)/(L*R)) - 1)))/(2*R*R*T0*(T0 - L*x(0)));
    dfdx(1,1) = (M*p0/R)*(1/(T0-L*x(0)))*std::pow((1-(L*x(0))/T0),(g*M)/(R*L))*x(1)*x(2);
    dfdx(1,2) = 0.5*(M*p0/R)*(1/(T0-L*x(0)))*std::pow((1-(L*x(0))/T0),(g*M)/(R*L))*x(1)*x(1);
    //Row 3
    dfdx(2,0) = 0;
    dfdx(2,1) = 0;
    dfdx(2,2) = 0;
}


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// 
// BallisticMeasurementModel
// 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h)
{
    const double & r1   = param.r1;
    const double & r2   = param.r2;
    // TODO: mean function
    h = Eigen::MatrixXd::Zero(1,1);
    h.resize(1,1);
    h(0) = std::sqrt(r1*r1 + (x(0)-r2)*(x(0)-r2));
}

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR)
{
    const double & r1   = param.r1;
    const double & r2   = param.r2;
    // TODO: mean function
    h = Eigen::MatrixXd::Zero(1,1);
    h.resize(1,1);
    h(0) = std::sqrt(r1*r1 + (x(0)-r2)*(x(0)-r2));
    // TODO: upper Cholesky factor of measurement covariance
    SR = Eigen::MatrixXd::Zero(1,1);
    SR.resize(1,1);
    SR(0) = 50;
}

void BallisticMeasurementModel::operator()(const Eigen::VectorXd & x, const Eigen::VectorXd & u, const BallisticParameters & param, Eigen::VectorXd & h, Eigen::MatrixXd & SR, Eigen::MatrixXd & dhdx)
{
    const double & r1   = param.r1;
    const double & r2   = param.r2;
    // TODO: mean function
    h = Eigen::MatrixXd::Zero(1,1);
    h.resize(1,1);
    h(0) = std::sqrt(r1*r1 + (x(0)-r2)*(x(0)-r2));
    // TODO: upper Cholesky factor of measurement covariance
    SR = Eigen::MatrixXd::Zero(1,1);
    SR.resize(1,1);
    SR(0) = 50;
    // TODO: Jacobian of mean measurement w.r.t x.
    dhdx.resize(1,x.rows());
    dhdx(0) = -(2*r2 - 2*x(0))/(2*std::sqrt((r2 - x(0))*(r2 - x(0)) + r1*r1));
    dhdx(1) = 0;
    dhdx(2) = 0;
}







