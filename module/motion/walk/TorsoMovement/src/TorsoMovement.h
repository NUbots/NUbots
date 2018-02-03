#ifndef MODULE_MOTION_WALK_TORSOMOVEMENT_H
#define MODULE_MOTION_WALK_TORSOMOVEMENT_H

#include <Eigen/Core>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class TorsoMovement : public NUClear::Reactor {
        private:
            size_t subsumptionId;


        public:
            /// @brief Called by the powerplant to build and setup the TorsoMovement reactor.
            explicit TorsoMovement(std::unique_ptr<NUClear::Environment> environment);
        };
    }
}
}

#endif  // MODULE_MOTION_WALK_TORSOMOVEMENT_H
