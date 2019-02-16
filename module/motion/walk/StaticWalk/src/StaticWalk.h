#ifndef MODULE_MOTION_WALK_STATICWALK_H
#define MODULE_MOTION_WALK_STATICWALK_H

#include <Eigen/Geometry>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class StaticWalk : public NUClear::Reactor {

        private:
            // The states that the robots enters to complete the steps
            enum State { INITIAL, LEFT_LEAN, RIGHT_STEP, RIGHT_LEAN, LEFT_STEP } state;
            // The time each phase takes to complete
            NUClear::clock::duration phase_time;
            // The time at the start of the current phase
            NUClear::clock::time_point start_phase;
            // The height of the robots torso (m)
            double torso_height;
            // The width of the robots stance as it walks (m)
            double stance_width;
            // Offset of the foot in the x direction as that the robot leans into its support polygon
            double foot_offset;
            size_t subsumptionId;
            // Transform from support foot to swing foot
            Eigen::Affine3d Hff_s;

        public:
            /// @brief Called by the powerplant to build and setup the StaticWalk reactor.
            explicit StaticWalk(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_STATICWALK_H
