#ifndef MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
#define MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/TorsoTarget.h"

namespace module {
namespace motion {
    namespace walk {
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::TorsoTarget;
        class MovementController : public NUClear::Reactor {
        private:
            // Returns vector from the vector field of where the foot should move on x-axis
            double f_x(const Eigen::Vector3d& vec);
            // Returns vector from the vector field of where the foot should move on y-axis (in plane space)
            double f_y(const Eigen::Vector3d& vec);
            // Plans where the torso will move to next
            Eigen::Affine3d plan_torso(const NUClear::clock::time_point& now,
                                       const Sensors& sensors,
                                       const TorsoTarget& target);
            // Plans where the swing foot will move to next
            Eigen::Affine3d plan_swing(const NUClear::clock::time_point& now,
                                       const Sensors& sensors,
                                       const FootTarget& target,
                                       const Eigen::Affine3d& Htf_s);

            // Height that the robot lifts its foot to step
            double step_height;
            // How steep the foot comes down to its target
            double step_steep;
            // Size of the well near the target
            double well_width;
            // Constant for mathematical function based on step_height, step_steep and well_width
            double c;
            // Gain for the feet movements
            double gain;
            // How far in the future we are sending the movements
            double time_horizon;
            // Make the step complete slightly earlier to make sure it is at the target on time
            NUClear::clock::duration offset_time;

        public:
            /// @brief Called by the powerplant to build and setup the MovementController reactor.
            explicit MovementController(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
