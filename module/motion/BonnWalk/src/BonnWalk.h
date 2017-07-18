#ifndef MODULE_MOTION_BONNWALK_H
#define MODULE_MOTION_BONNWALK_H

#include <nuclear>

namespace module {
namespace motion {

    class BonnWalk : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the BonnWalk reactor.
        explicit BonnWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_MOTION_BONNWALK_H
