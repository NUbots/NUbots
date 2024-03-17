#ifndef MODULE_INPUT_ORBSLAM_HPP
#define MODULE_INPUT_ORBSLAM_HPP

#include <nuclear>
#include <opencv2/core/core.hpp>

#include "utility/input/ORBSLAM3/src/System.h"

namespace module::input {

    class ORBSLAM : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        /// @brief The ORB_SLAM3 system
        std::unique_ptr<ORB_SLAM3::System> slam_system;

    public:
        /// @brief Called by the powerplant to build and setup the ORBSLAM reactor.
        explicit ORBSLAM(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_ORBSLAM_HPP
