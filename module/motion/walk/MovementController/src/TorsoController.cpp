#include "TorsoController.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        /**
         * Calculates the next torso postion we should be at in time_horizon amount of time
         *
         * @param time_horizon Time into the future with which we project the next position to
         * @param time_left The amount of time remaining until we should hit our target
         * @param Htg Homogeneous transform from ground space (g) to current torso space (t)
         * @param Ht_tg Homogeneous transform from ground space (g) to our target torso space (t_t)
         *
         * @return Hng Homogeneous transform from ground space (g) to the horizon torso space (next torso space (n))
         */
        Eigen::Affine3d TorsoController::next_torso(const double& time_horizon,
                                                    const double& time_left,
                                                    const Eigen::Affine3d& Htg,
                                                    const Eigen::Affine3d& Ht_tg) {
            // Interpolate towards target
            double factor = time_horizon / time_left;

            // The final homogeneous transform which we will build over the course of this function
            Eigen::Affine3d Hgn;

            // Interpolate the translation between current torso position and target position

            // Get ground to torso in ground space and ground to torso target in ground space
            // Htg.rotation().transpose() = Rgt
            // Htg.translation() = rGTt
            // -rGTt = rTGt
            // Rgt * rTGt = rTGg
            Eigen::Vector3d rTGg   = -Htg.rotation().transpose() * Htg.translation();
            Eigen::Vector3d rT_tGg = -Ht_tg.rotation().transpose() * Ht_tg.translation();

            // Get torso to torso target in ground space
            // and then multiply it by factor to find the next position.
            Eigen::Vector3d rT_tTg = rT_tGg - rTGg;
            Eigen::Vector3d rNTg   = rT_tTg * factor;

            if (rT_tTg.norm() < 0.001) {
                rNTg = rT_tTg;
            }

            // Set the translation of the final homogeneous transform
            // rNGg = rNTg + rTGg
            Hgn.translation() = rNTg + rTGg;

            // Slerp the rotation of the torso
            Hgn.linear() = Eigen::Quaterniond(Ht_tg.rotation())
                               .slerp(factor, Eigen::Quaterniond(Htg.rotation()))
                               .toRotationMatrix()
                               .transpose();

            // If the difference between the current rotation and target rotation is small, just go straight to target
            // rotation
            if ((Htg.rotation() - Ht_tg.rotation()).maxCoeff() < 0.001 || factor == 1) {
                Hgn.linear() = Ht_tg.rotation().transpose();
            }

            return Hgn.inverse();
        }


    }  // namespace walk
}  // namespace motion
}  // namespace module
