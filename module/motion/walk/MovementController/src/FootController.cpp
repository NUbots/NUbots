#include "FootController.hpp"

#include "utility/motion/walk/vectorfield.hpp"

using utility::motion::walk::f_x;
using utility::motion::walk::f_z;
using utility::motion::walk::pathlength;

namespace module {
    namespace motion {
        namespace walk {
            /**
             * Calculates the next swing foot position we should be at in time_horizon amount of time
             *
             * @param time_horizon Time into the future with which we project the next position to
             * @param time_left The amount of time remaining until we should hit our target
             * @param Hwg Homogeneous transform from ground space (g) to current s"wing" (w) foot space
             * @param Hw_tg Homogeneous transform from ground space (g) to s"wing" target (t) foot space
             *
             * @return Hng Homogeneous transform from ground space to next target s"wing" (n) foot space
             */
            Eigen::Affine3d FootController::next_swing(const double& time_horizon,
                                                       const double& time_left,
                                                       const Eigen::Affine3d& Hwg,
                                                       const Eigen::Affine3d& Htg) {
                // Amount to move by in the next update to get to the target position in the given amount of time
                double factor = time_horizon / time_left;


                //---------------------------------------------------------------
                //-------------VECTOR FIELD--------------------------------------
                //---------------------------------------------------------------

                // Get the vector from target to ground in ground space and ground to (s)wing foot in ground space
                // Htg.rotation().transpose() = Rgt
                // Htg.translation() = rGTt
                // -rGTt = rTGt
                // Rgt * rGTt = rGTg
                const Eigen::Vector3d rTGg = -Htg.rotation().transpose() * Htg.translation();
                const Eigen::Vector3d rWGg = -Hwg.rotation().transpose() * Hwg.translation();

                // Get the vector from target to (s)wing foot in ground space
                const Eigen::Vector3d rWTg = rWGg - rTGg;

                // Use the vector field to get the (s)wing target to next target position
                // The vector field gives the vector from the swing foot to the next swing foot position in ground space (rNWg)
                // Normalize the vector and multiply it by the distance and factor to reach the target at the right
                // time
                float distance = pathlength(rWTg, config.integral_steps, config.scaling_factor, config.step_height);
                Eigen::Vector3d rNWg =
                    Eigen::Vector3d(f_x(rWTg, config.scaling_factor),
                                    0,
                                    f_z(rWTg, config.step_height, config.scaling_factor))
                        .normalized() * factor * distance;
                Eigen::Vector3d rNTg = rNWg + rWTg;

                // todo: why is this here?
                if (rWTg.z() + rNWg.z() < 0) {
                    rNTg.z() = 0;
                }

                // Vector field does not take into account the y-value so we linearly interpolate so it will reach the
                // target
                rNTg.y() = rWTg.y() * (1 - factor);

                // Retrieve the final vector for the translation component of Hgn
                Eigen::Vector3d rNGg = rNTg + rTGg;

                //---------------------------------------------------------------
                //---------------------------------------------------------------

                // Get the distance to the target
                double distance = rWTg.norm();

                // Create the Hgn matrix
                // Slerp the rotation
                // Set the translation
                Eigen::Affine3d Hgn;

                Hgn.linear() = Eigen::Quaterniond(Htg.rotation())
                                   .slerp(factor, Eigen::Quaterniond(Hwg.rotation()))
                                   .toRotationMatrix()
                                   .transpose();

                // If we are very close to the target, just go to the target directly
                // if (rWTg.norm() < 0.025) {
                //    rNGg = rTGg;
                //}
                if ((Htg.rotation() - Hwg.rotation()).maxCoeff() < 0.001 || factor == 1) {
                    Hgn.linear() = Htg.rotation().transpose();
                }

                Hgn.translation() = rNGg;

                return Hgn.inverse();
            }

        }  // namespace walk
    }      // namespace motion
}  // namespace module
