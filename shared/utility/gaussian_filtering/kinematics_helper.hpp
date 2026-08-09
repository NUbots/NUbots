#ifndef KINEMATICS_HELPER_HPP
#define KINEMATICS_HELPER_HPP

#include <Eigen/Core>

namespace utility::gaussian_filtering {
    Eigen::Matrix3d TKfromTheta(const Eigen::VectorXd& Theta);

    /// Smallest |cos(pitch)| the Euler-rate transform is evaluated at, bounding tan(pitch) at 1e3.
    constexpr double kTKMinCosPitch = 1e-3;

    // Templated version
    //
    // The roll-pitch-yaw rate transform is singular at pitch = +-90 deg, and a
    // forward or backward fall passes straight through it. Left unguarded, the
    // dynamics Jacobian there is infinite: RK4SDEHelper propagates that into the
    // predicted covariance and the NewtonTrustEig update is handed a NaN prior, so
    // a single fall poisons the filter for the remainder of the run rather than
    // just for its duration.
    //
    // cos(pitch) is therefore saturated at kTKMinCosPitch, keeping its sign. This is
    // a genuine saturation, not an approximation: inside the clamped band the
    // derivative w.r.t. pitch is zero, which is what stops the blow-up. The pose
    // estimate is meaningless at the singularity either way -- yaw is not defined
    // when the torso z axis is horizontal -- so the value of guarding is purely that
    // the filter stays finite and can recover once the robot is upright again. At
    // |pitch| < 89.94 deg the clamp never binds, so upright operation is unchanged.
    //
    // SystemLocalisation no longer routes its attitude through here -- it carries a
    // quaternion, whose kinematics (quatXi) have no singularity at all. This remains
    // for the roll-pitch-yaw pose helpers below and any estimator still built on them.
    template <typename Scalar>
    Eigen::Matrix3<Scalar> TKfromThetaTemplated(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Theta) {
        using std::sin, std::cos;
        const Scalar phi   = Theta(0);
        const Scalar theta = Theta(1);
        // const Scalar psi = Theta(2);

        Scalar ctheta = cos(theta);
        if (ctheta * ctheta < Scalar(kTKMinCosPitch * kTKMinCosPitch)) {
            ctheta = ctheta < Scalar(0.0) ? Scalar(-kTKMinCosPitch) : Scalar(kTKMinCosPitch);
        }
        const Scalar ttheta = sin(theta) / ctheta;

        // clang-format off
        Eigen::Matrix3<Scalar> TK;
        TK << Scalar(1.0), sin(phi)*ttheta, cos(phi)*ttheta,
              Scalar(0.0),        cos(phi),       -sin(phi),
              Scalar(0.0), sin(phi)/ctheta, cos(phi)/ctheta;
        // clang-format on
        return TK;
    }

    Eigen::MatrixXd JKfromEta(const Eigen::VectorXd& eta);

}  // namespace utility::gaussian_filtering

#endif  // KINEMATICS_HELPER_H
