#ifndef MODULE_VISION_FEATUREDETECTOR_HPP
#define MODULE_VISION_FEATUREDETECTOR_HPP

#include <nuclear>

namespace module::vision {

class FeatureDetector : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the FeatureDetector reactor.
    explicit FeatureDetector(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::vision

#endif  // MODULE_VISION_FEATUREDETECTOR_HPP
