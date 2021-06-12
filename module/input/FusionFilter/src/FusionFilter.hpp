#ifndef MODULE_INPUT_FUSIONFILTER_HPP
#define MODULE_INPUT_FUSIONFILTER_HPP

#include <nuclear>

namespace module::input {

class FusionFilter : public NUClear::Reactor {
private:
    /// The configuration variables for this reactor
    struct {
    } config;

public:
    /// @brief Called by the powerplant to build and setup the FusionFilter reactor.
    explicit FusionFilter(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::input

#endif  // MODULE_INPUT_FUSIONFILTER_HPP
