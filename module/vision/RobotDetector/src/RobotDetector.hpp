#ifndef MODULE_VISION_ROBOTDETECTOR_HPP
#define MODULE_VISION_ROBOTDETECTOR_HPP

#include <nuclear>

namespace module::vision {

    class RobotDetector : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double confidence_threshold   = 0.0;
            int cluster_points            = 0;
            double minimum_robot_distance = 0.0;
            double minimum_robot_height   = 0.0;
            double maximum_robot_height   = 0.0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the RobotDetector reactor.
        explicit RobotDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_ROBOTDETECTOR_HPP
