#ifndef MODULE_VISION_SEGMENTATION_HPP
#define MODULE_VISION_SEGMENTATION_HPP

#include <nuclear>

namespace module::vision {

class Segmentation : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Segmentation reactor.
    explicit Segmentation(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::vision

#endif  // MODULE_VISION_SEGMENTATION_HPP
