
#ifndef MODULE_MOTION_WALK_STATICWALK_H
#define MODULE_MOTION_WALK_STATICWALK_H

#include <Eigen/Geometry>
#include <nuclear>
#include "message/motion/KinematicsModel.h"

namespace module {
namespace motion {
    namespace walk {

        class StaticWalk : public NUClear::Reactor {

        private:
            // Transform from ground to swing foot
            Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hwg;
            // The states that the robots enters to complete the steps
            enum State { INITIAL, LEFT_LEAN, RIGHT_STEP, RIGHT_LEAN, LEFT_STEP, STOP } state;
            // The time each phase takes to complete
            NUClear::clock::duration phase_time;
            // The time at the start of the current phase
            NUClear::clock::time_point start_phase;
            // Model of the robot
            message::motion::KinematicsModel model;
            // The height of the robots torso (m)
            double torso_height;
            // The width of the robots stance as it walks (m)
            double stance_width;
            bool start_right_lean;
            double y_offset;
            double x_offset;
            double time;
            double rotation_limit;
            size_t subsumptionId;

            // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
            ReactionHandle updateHandle;

            // Returns ground to torso target for specified lean
            Eigen::Affine3d getLeanTarget(double y_offset_local, const Eigen::Vector3d& rCTt);

            // Returns ground to foot target for specified step
            Eigen::Affine3d getFootTarget(const enum State state,
                                          const Eigen::Vector3d& walkcommand,
                                          const bool isLeft);

        public:
            /// @brief Called by the powerplant to build and setup the StaticWalk reactor.
            explicit StaticWalk(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_STATICWALK_H
