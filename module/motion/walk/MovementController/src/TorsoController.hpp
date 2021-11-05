#ifndef MODULE_MOTION_WALK_TORSOCONTROLLER_HPP
#define MODULE_MOTION_WALK_TORSOCONTROLLER_HPP

#include <Eigen/Geometry>

namespace module {
    namespace motion {
        namespace walk {

            class TorsoController {
            public:
                Eigen::Affine3d next_torso(const double& time_horizon,
                                           const double& time_left,
                                           const Eigen::Affine3d& Htg,
                                           const Eigen::Affine3d& Ht_tg);

                struct {
                    // Limit the amount that the robot can move in one time step to prevent large sudden movements
                    double max_translation;
                    double max_rotation;
                    // If the distance to the target is less than this threshold, go directly to the target
                    double translation_threshold;
                    // If the distance to the target rotation is less than this threshold, go directly to the target
                    // rotation
                    double rotation_threshold;
                } config;

            private:
                Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hgo;
            };

        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_TORSOCONTROLLER_HPP
