#ifndef MODULE_MOTION_BONNWALK_H
#define MODULE_MOTION_BONNWALK_H

#include <nuclear>

#include "gait/GaitEngine.h"

namespace module {
namespace motion {

    class BonnWalk : public NUClear::Reactor {
    private:
        gait::GaitEngine engine;
        ReactionHandle handle;
        NUClear::clock::duration updateRate;
        size_t subsumptionId;

    public:
        /// @brief Called by the powerplant to build and setup the BonnWalk reactor.
        explicit BonnWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_BONNWALK_H
