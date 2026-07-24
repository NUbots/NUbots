#ifndef KINEMATICS_HELPER_HPP
#define KINEMATICS_HELPER_HPP

#include <Eigen/Core>

namespace utility::slam {
    Eigen::Matrix3d TKfromTheta(const Eigen::VectorXd& Theta);
    // Templated version
    template <typename Scalar>
    Eigen::Matrix3<Scalar> TKfromThetaTemplated(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Theta) {
        using std::sin, std::cos, std::tan;
        const Scalar phi   = Theta(0);
        const Scalar theta = Theta(1);
        // const Scalar psi = Theta(2);

        Eigen::Matrix3<Scalar> TK;
        TK << Scalar(1.0), sin(phi) * tan(theta), cos(phi) * tan(theta), Scalar(0.0), cos(phi), -sin(phi), Scalar(0.0),
            sin(phi) / cos(theta), cos(phi) / cos(theta);
        return TK;
    }

    Eigen::MatrixXd JKfromEta(const Eigen::VectorXd& eta);

}  // namespace utility::slam

#endif  // KINEMATICS_HELPER_H
