
#include "localisation.h"

#include <iostream>

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
//
// LocalisationProcessModel
//
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void LocalisationProcessModel::operator()(const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const LocalisationParameters& param,
                                          Eigen::VectorXd& f) {
    // mean function, x = [x;y]
    // Process model is effectively Xk+1 = Xk + Some Noise LMAO
}

void LocalisationProcessModel::operator()(const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const LocalisationParameters& param,
                                          Eigen::VectorXd& f,
                                          Eigen::MatrixXd& SQ) {
    operator()(x, u, param, f);
    SQ.resize(x.rows(), x.rows());
    SQ.setZero();
    // Fill SQ with process model noise covariance
    SQ = param.process_noise;
    std::cout << "SQ in pm: " << std::endl << SQ << std::endl;
}


void LocalisationProcessModel::operator()(const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const LocalisationParameters& param,
                                          Eigen::VectorXd& f,
                                          Eigen::MatrixXd& SQ,
                                          Eigen::MatrixXd& dfdx) {
    int nx = x.rows();
    operator()(x, u, param, f, SQ);
    // Fill dfdx with nx by nx identity
    dfdx.resize(nx, nx);
    dfdx.setIdentity();
}


// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
//
// LocalisationMeasurementModel
//
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void LocalisationMeasurementModel::operator()(const Eigen::VectorXd& x,
                                              const Eigen::VectorXd& u,
                                              const LocalisationParameters& param,
                                              Eigen::VectorXd& h) {
    // mean function
    int nx = x.rows();
    h.resize(nx, 1);
    h.setZero();
    // print nx
    std::cout << "nx: " << nx << std::endl;

    double xDiff = param.goal_posts[0].x() - x(0);
    double yDiff = param.goal_posts[0].y() - x(1);
    h(0)         = std::sqrt(xDiff * xDiff + yDiff * yDiff);
    xDiff        = param.goal_posts[1].x() - x(0);
    yDiff        = param.goal_posts[1].y() - x(1);
    h(1)         = std::sqrt(xDiff * xDiff + yDiff * yDiff);
}

void LocalisationMeasurementModel::operator()(const Eigen::VectorXd& x,
                                              const Eigen::VectorXd& u,
                                              const LocalisationParameters& param,
                                              Eigen::VectorXd& h,
                                              Eigen::MatrixXd& SR) {
    operator()(x, u, param, h);
    int nx = x.rows();
    // upper Cholesky factor of measurement covariance
    SR.resize(nx, nx);
    SR = Eigen::MatrixXd::Zero(nx, nx);
    // Fill SR with measurement noise covariance
    SR = param.measurement_noise;
    // Print SR
    std::cout << "SR in mm: " << std::endl << SR << std::endl;
}

void LocalisationMeasurementModel::operator()(const Eigen::VectorXd& x,
                                              const Eigen::VectorXd& u,
                                              const LocalisationParameters& param,
                                              Eigen::VectorXd& h,
                                              Eigen::MatrixXd& SR,
                                              Eigen::MatrixXd& dhdx) {
    // call function above
    operator()(x, u, param, h, SR);
    int nx = x.rows();
    int nh = h.rows();
    // Calcualte Jacobian of measurement model with respect to x
    dhdx.resize(nh, nx);
    dhdx.setZero();
    dhdx(0, 0) = (param.goal_posts[0].x() - x(0)) / h(0);
    dhdx(0, 1) = (param.goal_posts[0].y() - x(1)) / h(0);
    dhdx(1, 0) = (param.goal_posts[1].x() - x(0)) / h(1);
    dhdx(1, 1) = (param.goal_posts[1].y() - x(1)) / h(1);
}

// using std::sqrt;

// // Templated version of localisationLogLikelihood
// // Note: templates normally should live in a template header (.hpp), but
// //       since all instantiations of this template are used only in this
// //       compilation unit, its definition can live here
// template <typename Scalar>
// static Scalar localisationLogLikelihood(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& y,
//                                         const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
//                                         const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& u,
//                                         const LocalisationParameters& param) {
//     // mean function
//     h.resize(2, 1);
//     h = Eigen::MatrixXd::Zero(2, 1);

//     double xDiff = param.goal_posts[0].x() - x(0);
//     double yDiff = param.goal_posts[0].y() - x(1);
//     h(0)         = std::sqrt(xDiff * xDiff + yDiff * yDiff);
//     xDiff        = param.goal_posts[1].x() - x(0);
//     yDiff        = param.goal_posts[1].y() - x(1);
//     h(1)         = std::sqrt(xDiff * xDiff + yDiff * yDiff);
//     // upper Cholesky factor of measurement covariance
//     SR.resize(2, 2);
//     SR = Eigen::MatrixXd::Zero(2, 2);
//     // Fill SR with measurement noise covariance
//     SR = param.measurement_noise;

//     return logGaussian(y, h, SR);
// }

// double LocalisationLogLikelihood::operator()(const Eigen::VectorXd& y,
//                                              const Eigen::VectorXd& x,
//                                              const Eigen::VectorXd& u,
//                                              const LocalisationParameters& param) {
//     // Evaluate log N(y;h(x),R)
//     return localisationLogLikelihood(y, x, u, param);
// }


// double LocalisationLogLikelihood::operator()(const Eigen::VectorXd& y,
//                                              const Eigen::VectorXd& x,
//                                              const Eigen::VectorXd& u,
//                                              const LocalisationParameters& param,
//                                              Eigen::VectorXd& g) {
//     Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual>();
//     autodiff::dual fdual;
//     g = gradient(localisationLogLikelihood<autodiff::dual>, wrt(xdual), at(y, xdual, u, param), fdual);

//     return val(fdual);
// }

// double LocalisationLogLikelihood::operator()(const Eigen::VectorXd& y,
//                                              const Eigen::VectorXd& x,
//                                              const Eigen::VectorXd& u,
//                                              const LocalisationParameters& param,
//                                              Eigen::VectorXd& g,
//                                              Eigen::MatrixXd& H) {
//     using dual2nd                                   = autodiff::HigherOrderDual<2>;
//     Eigen::Matrix<dual2nd, Eigen::Dynamic, 1> xdual = x.cast<dual2nd>();
//     dual2nd fdual;
//     H = hessian(localisationLogLikelihood<dual2nd>, wrt(xdual), at(y, xdual, u, param), fdual, g);
//     return val(fdual);
// }
