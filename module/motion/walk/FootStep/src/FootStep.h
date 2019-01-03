#ifndef MODULE_MOTION_WALK_FOOTSTEP_H
#define MODULE_MOTION_WALK_FOOTSTEP_H

#include <Eigen/Core>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class FootStep : public NUClear::Reactor {
        private:
            // Returns vector from the vector field of where the foot should move on x-axis
            double f_x(const Eigen::Vector3d& vec);
            // Returns vector from the vector field of where the foot should move on y-axis (in plane space)
            double f_y(const Eigen::Vector3d& vec);
            // Returns factor by which the foot should move this iteration to reach target at specified time
            double factor(const Eigen::Vector3d& vec, double t);

            size_t subsumptionId;
            ReactionHandle update_handle;
            double step_height;
            double step_steep;
            double well_width;
            double time_horizon;
            /// Constant for mathematical function based on step_height, step_steep and well_width
            double c;

        public:
            /// @brief Called by the powerplant to build and setup the FootStep reactor.
            explicit FootStep(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_FOOTSTEP_H
