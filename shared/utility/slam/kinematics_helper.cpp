#include "kinematics_helper.hpp"

#include <Eigen/Core>
#include <cmath>

#include "rotation.hpp"

namespace utility::slam {

    using std::sin, std::cos, std::tan;

    Eigen::Matrix3d TKfromTheta(const Eigen::VectorXd& Theta) {
        const double phi   = Theta(0);
        const double theta = Theta(1);

        Eigen::Matrix3d TK;
        TK << 1, sin(phi) * tan(theta), cos(phi) * tan(theta), 0, cos(phi), -sin(phi), 0, sin(phi) / cos(theta),
            cos(phi) / cos(theta);
        return TK;
    }

    Eigen::MatrixXd JKfromEta(const Eigen::VectorXd& eta) {
        const Eigen::Vector3d Theta = eta.segment<3>(3);
        const Eigen::Matrix3d TK    = TKfromTheta(Theta);

        Eigen::MatrixXd JK   = Eigen::MatrixXd::Zero(6, 6);
        JK.block<3, 3>(0, 0) = rpy2rot(Theta);
        JK.block<3, 3>(3, 3) = TK;
        return JK;
    }

}  // namespace utility::slam
