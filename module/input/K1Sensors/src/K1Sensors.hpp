#ifndef MODULE_INPUT_K1SENSORS_HPP
#define MODULE_INPUT_K1SENSORS_HPP

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <booster/idl/geometry_msgs/Pose.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "k1sensors/k1_model.hpp"

namespace module::input {

    using message::input::Sensors;
    using message::platform::RawSensors;

    class K1Sensors : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string pose_topic;
            Eigen::Isometry3d Hhp = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d Hpc = Eigen::Isometry3d::Identity();
            /// @brief Deadband below which normalized odometry components are snapped to zero, to
            /// avoid tiny floating-point residuals (e.g. ~1e-10) being treated as real motion
            double odometry_deadband = 0.0;
        } cfg;

        /// @brief DDS reader for the head pose topic, created once at startup
        booster::robot::ChannelPtr<geometry_msgs::msg::Pose> pose_channel;
        /// @brief True once the reader exists, so a config reload can't create a second one
        bool channel_created = false;

        /// @brief Guards Hrh and have_pose, which are written from the DDS callback thread
        std::mutex pose_mutex;
        /// @brief Head frame in the robot base frame, from the most recent head pose message
        Eigen::Isometry3d Hrh = Eigen::Isometry3d::Identity();
        /// @brief False until the first head pose message arrives, while Hrh is still identity
        bool have_pose = false;
        /// @brief Set once the missing head pose warning has been logged
        std::atomic<bool> pose_warned{false};

        std::mutex odometry_mutex;
        bool booster_odometry_has_offset = false;
        std::array<double, 3> booster_odometry_offset{};
        /// @brief Last Booster motion mode seen, used to clear the odometry offset whenever the mode
        /// changes (-1 = no mode seen yet)
        int last_booster_mode = -1;

        bool left_down   = false;
        bool middle_down = false;

        /// @brief Number of actuatable joints in the K1 robot
        static constexpr int n_servos = 22;

        /// @brief DDS callback for the head pose topic, caches the pose as Hrh
        /// @param msg The received geometry_msgs::msg::Pose
        void pose_handler(const void* msg);

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
