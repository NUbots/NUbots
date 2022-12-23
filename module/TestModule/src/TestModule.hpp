#ifndef MODULE_TESTMODULE_HPP
#define MODULE_TESTMODULE_HPP

#include <Eigen/Core>
#include <nuclear>
// #include <vector>

namespace module {

    class TestModule : public NUClear::Reactor {
    private:
        uint BROADCAST_IP = 0xFFFFFFFF;

        std::chrono::time_point<std::chrono::steady_clock> prev_timestamp =
            std::chrono::time_point<std::chrono::steady_clock>::min();
        Eigen::Vector3f prev_position = Eigen::Vector3f::Zero();
        double test                   = 2;

        /// @brief Stores configuration values
        struct Config {
            /// @brief How much to increment the count by each time a new Ping or Pong message is emitted
            int increment = 0;
            uint recieve_port;
            uint send_port;
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the TestModule reactor.
        explicit TestModule(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module

#endif  // MODULE_TESTMODULE_HPP
