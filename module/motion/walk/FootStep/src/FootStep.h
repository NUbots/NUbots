#ifndef MODULE_MOTION_WALK_FOOTSTEP_H
#define MODULE_MOTION_WALK_FOOTSTEP_H

#include <nuclear>

namespace module {
namespace motion {
namespace walk {

    class FootStep : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FootStep reactor.
        explicit FootStep(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULE_MOTION_WALK_FOOTSTEP_H
