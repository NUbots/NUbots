#ifndef MODULE_INPUT_K1SENSORS_HPP
#define MODULE_INPUT_K1SENSORS_HPP

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>

namespace module::input {

    struct PoseSharedMemory;

    class K1Sensors : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string pose_segment;
            Eigen::Isometry3d Hpk = Eigen::Isometry3d::Identity();
        } cfg;

        std::mutex pose_mutex;
        std::unique_ptr<PoseSharedMemory> pose_shared_memory;
        bool pose_unavailable_logged = false;

        std::mutex odometry_mutex;
        bool booster_odometry_has_offset = false;
        std::array<double, 3> booster_odometry_offset{};

        bool left_down   = false;
        bool middle_down = false;

        void connect_head_pose();
        bool read_head_pose(std::array<double, 3>& position, std::array<double, 4>& orientation);

    public:
        /// @brief Called by the powerplant to build and setup the K1Sensors reactor.
        explicit K1Sensors(std::unique_ptr<NUClear::Environment> environment);
        ~K1Sensors();
    };

}  // namespace module::input

#endif  // MODULE_INPUT_K1SENSORS_HPP
