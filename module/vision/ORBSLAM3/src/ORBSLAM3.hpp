#ifndef MODULE_VISION_ORBSLAM3_HPP
#define MODULE_VISION_ORBSLAM3_HPP

#include <nuclear>

namespace module::vision {

class ORBSLAM3 : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the ORBSLAM3 reactor.
    explicit ORBSLAM3(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::vision

#endif  // MODULE_VISION_ORBSLAM3_HPP
