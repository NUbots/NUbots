#ifndef MODULE_VISION_FIELDLINEDETECTOR_H
#define MODULE_VISION_FIELDLINEDETECTOR_H

#include <nuclear>

namespace module {
namespace vision {

    class FieldLineDetector : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FieldLineDetector reactor.
        explicit FieldLineDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_VISION_FIELDLINEDETECTOR_H
