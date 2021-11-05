#ifndef MODULE_MOTION_WALK_FOOTCONTROLLER_HPP
#define MODULE_MOTION_WALK_FOOTCONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>


namespace module {
    namespace motion {
        namespace walk {

            class FootController {
            public:
                // Calculates next swing position
                Eigen::Affine3d next_swing(const double& time_horizon,
                                           const double& time_left,
                                           const Eigen::Affine3d& Hwg,
                                           const Eigen::Affine3d& Hw_tg);

                // Stores config variables for the vector field
                struct {
                    double step_height;
                    double well_width;
                    double step_steep;
                    double scaling_factor;
                    double c;
                    double integral_steps;
                } config;
            };

        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_FOOTCONTROLLER_HPP
