#include "kinematics_helper.hpp"

#include <Eigen/Core>
#include <cmath>

#include "rotation.hpp"

namespace utility::gaussian_filtering {

    // Forwards to the templated version so the pitch singularity is guarded in one
    // place; a second copy of the formula would only be a second place to forget it.
    Eigen::Matrix3d TKfromTheta(const Eigen::VectorXd& Theta) {
        return TKfromThetaTemplated<double>(Theta);
    }

    Eigen::MatrixXd JKfromEta(const Eigen::VectorXd& eta) {
        const Eigen::Vector3d Theta = eta.segment<3>(3);
        const Eigen::Matrix3d TK    = TKfromTheta(Theta);

        Eigen::MatrixXd JK   = Eigen::MatrixXd::Zero(6, 6);
        JK.block<3, 3>(0, 0) = rpy2rot(Theta);
        JK.block<3, 3>(3, 3) = TK;
        return JK;
    }

}  // namespace utility::gaussian_filtering
