#ifndef MODULE_OBTASK_TESTSTART_HPP
#define MODULE_OBTASK_TESTSTART_HPP

#include <nuclear>

namespace module::obtask {

    class testStart : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            int num   = 1;
            int total = 1;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the testStart reactor.
        explicit testStart(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::obtask

#endif  // MODULE_OBTASK_TESTSTART_HPP
