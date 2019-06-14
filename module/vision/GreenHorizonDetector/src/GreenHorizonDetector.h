#ifndef MODULE_VISION_GREENHORIZONDETECTOR_H
#define MODULE_VISION_GREENHORIZONDETECTOR_H

#include <nuclear>

namespace module {
namespace vision {

    class GreenHorizonDetector : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the GreenHorizonDetector reactor.
        explicit GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment);
    private:
        struct {float seed_confidence;
        float end_confidence;
        float cluster_points;
        } config;

    };

}
}

#endif  // MODULE_VISION_GREENHORIZONDETECTOR_H
