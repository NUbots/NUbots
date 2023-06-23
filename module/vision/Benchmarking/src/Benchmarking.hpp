#ifndef MODULE_VISION_BENCHMARKING_HPP
#define MODULE_VISION_BENCHMARKING_HPP

#include <nuclear>

namespace module::vision {

class Benchmarking : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Benchmarking reactor.
    explicit Benchmarking(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::vision

#endif  // MODULE_VISION_BENCHMARKING_HPP
