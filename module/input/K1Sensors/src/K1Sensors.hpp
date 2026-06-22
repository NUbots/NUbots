#ifndef MODULE_INPUT_K1SENSORS_HPP
#define MODULE_INPUT_K1SENSORS_HPP

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "k1sensors/k1_model.hpp"

namespace module::input {

    struct PoseSharedMemory;

    using message::input::Sensors;
    using message::platform::RawSensors;

    class K1Sensors : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string pose_segment;
            Eigen::Isometry3d Hpk        = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d Hrp_offset = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d Hpk_offset = Eigen::Isometry3d::Identity();
        } cfg;

        std::mutex pose_mutex;
        std::unique_ptr<PoseSharedMemory> pose_shared_memory;
        bool pose_unavailable_logged = false;

        std::mutex odometry_mutex;
        bool booster_odometry_has_offset = false;
        std::array<double, 3> booster_odometry_offset{};

        bool left_down   = false;
        bool middle_down = false;

        /// @brief Number of actuatable joints in the K1 robot
        static constexpr int n_servos = 22;

        /// @brief Opaque tinyrobotics model — instantiated in k1_model.cpp only
        K1Model k1_kinematics;

        void connect_head_pose();
        bool read_head_pose(std::array<double, 3>& position, std::array<double, 4>& orientation);

        /// @brief Updates the sensors message with raw sensor data, including servo joint information
        /// @param sensors The sensors message to update
        /// @param raw_sensors The raw sensor data
        void update_raw_sensors(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors);

    public:
        /// @brief Called by the powerplant to build and setup the K1Sensors reactor.
        explicit K1Sensors(std::unique_ptr<NUClear::Environment> environment);
        ~K1Sensors();
    };

}  // namespace module::input

#endif  // MODULE_INPUT_K1SENSORS_HPP
