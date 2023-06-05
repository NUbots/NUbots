#ifndef MODULE_VISION_FIELDLINEDETECTOR_HPP
#define MODULE_VISION_FIELDLINEDETECTOR_HPP

#include <nuclear>

namespace module::vision {

    class FieldLineDetector : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            float cluster_points       = 0.0f;
            float confidence_threshold = 0.0f;
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the FieldLineDetector reactor.
        explicit FieldLineDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_FIELDLINEDETECTOR_HPP
