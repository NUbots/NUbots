#ifndef MODULE_ADDER_HPP
#define MODULE_ADDER_HPP

#include <nuclear>

namespace module {

    class adder : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            int running_total = 0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the adder reactor.
        explicit adder(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module

#endif  // MODULE_ADDER_HPP
