#ifndef MODULE_MOTION_WALK_FOOTCONTROLLER_H
#define MODULE_MOTION_WALK_FOOTCONTROLLER_H

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
                double c;
            } config;

        private:
            // Returns x and z positions of vector field. See vectorfield.py for graphical representation of vector
            // field
            double f_x(const Eigen::Vector3d& pos);
            double f_z(const Eigen::Vector3d& pos);

            Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hw_ong;
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_FOOTCONTROLLER_H
