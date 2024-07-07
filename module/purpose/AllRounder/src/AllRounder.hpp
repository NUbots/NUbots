#ifndef MODULE_PURPOSE_ALLROUNDER_HPP
#define MODULE_PURPOSE_ALLROUNDER_HPP

#include <nuclear>

namespace module::purpose {

class AllRounder : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the AllRounder reactor.
    explicit AllRounder(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_ALLROUNDER_HPP
