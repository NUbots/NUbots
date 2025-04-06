#ifndef MODULE_FIELDPLAYER_HPP
#define MODULE_FIELDPLAYER_HPP

#include <nuclear>

namespace module {

class FieldPlayer : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the FieldPlayer reactor.
    explicit FieldPlayer(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_FIELDPLAYER_HPP
