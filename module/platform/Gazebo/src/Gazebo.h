#ifndef MODULE_PLATFORM_GAZEBO_H
#define MODULE_PLATFORM_GAZEBO_H

#include <nuclear>

namespace module {
namespace platform {

    class Gazebo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Gazebo reactor.
        explicit Gazebo(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            std::string simulator_name;
            std::string model_name;
            double clock_smoothing;
        } config;

        double sim_time;
        double real_time;
    };

}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_GAZEBO_H
