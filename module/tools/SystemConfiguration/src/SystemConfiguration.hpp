#ifndef MODULE_TOOLS_SYSTEMCONFIGURATION_HPP
#define MODULE_TOOLS_SYSTEMCONFIGURATION_HPP

#include <nuclear>

namespace module::tools {

    class SystemConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SystemConfiguration reactor.
        explicit SystemConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_SYSTEMCONFIGURATION_HPP
