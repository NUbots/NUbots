#ifndef MODULE_MOTION_WALK_TORSOCONTROLLER_H
#define MODULE_MOTION_WALK_TORSOCONTROLLER_H

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
                double max_rotation;
                double max_translation;
            } config;

        private:
            Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hgo;
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_TORSOCONTROLLER_H
