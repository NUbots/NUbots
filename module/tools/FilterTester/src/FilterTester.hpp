#ifndef MODULE_TOOLS_FILTERTESTER_HPP
#define MODULE_TOOLS_FILTERTESTER_HPP

#include <nuclear>

namespace module::tools {

    class FilterTester : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FilterTester reactor.
        explicit FilterTester(std::unique_ptr<NUClear::Environment> environment);

        double total_error = 0;
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_FILTERTESTER_HPP
