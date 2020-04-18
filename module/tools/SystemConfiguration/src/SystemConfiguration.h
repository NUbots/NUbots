#ifndef MODULE_TOOLS_SYSTEMCONFIGURATION_H
#define MODULE_TOOLS_SYSTEMCONFIGURATION_H

#include <nuclear>

namespace module {
namespace tools {

    class SystemConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SystemConfiguration reactor.
        explicit SystemConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_SYSTEMCONFIGURATION_H
