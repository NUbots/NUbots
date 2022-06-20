// #include "gaussian.hpp"

// #include <Eigen/Core>
// #include <cassert>
// #include <iostream>


// // --------------------------------------------------------------------------------------
// // --------------------------------------------------------------------------------------
// //
// // conditionGaussianOnMarginal
// //
// // --------------------------------------------------------------------------------------
// // --------------------------------------------------------------------------------------

// // void conditionGaussianOnMarginal(const Eigen::VectorXd& muyx,
// //                                  const Eigen::MatrixXd& Syx,
// //                                  const Eigen::VectorXd& y,
// //                                  Eigen::VectorXd& muxGy,
// //                                  Eigen::MatrixXd& SxGy) {
// //     // TODO: Copy from Lab 4
// //     int ny              = y.rows();
// //     int nx              = Syx.cols() - ny;
// //     Eigen::MatrixXd S1  = Syx.topLeftCorner(ny, ny);
// //     Eigen::MatrixXd S2  = Syx.topRightCorner(ny, nx);
// //     Eigen::MatrixXd S3  = Syx.bottomRightCorner(nx, nx);
// //     Eigen::MatrixXd mux = muyx.tail(nx);
// //     Eigen::MatrixXd muy = muyx.head(ny);
// //     muxGy               = mux + S2.transpose() * (S1.triangularView<Eigen::Upper>().transpose().solve(y - muy));
// //     SxGy                = S3;
// // }
