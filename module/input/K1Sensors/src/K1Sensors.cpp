#include "K1Sensors.hpp"

#include <cmath>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterModeState.hpp"
#include "message/booster/BoosterOdometry.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/Booster/channel_factory.hpp"
#include "utility/platform/RawSensors.hpp"


namespace module::input {

    using booster::robot::ChannelFactory;
    using extension::Configuration;
    using message::booster::BoosterModeState;
    using message::booster::BoosterOdometry;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::input::Sensors;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::RawSensors;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;


    void K1Sensors::pose_handler(const void* msg) {
        const auto& pose = *static_cast<const geometry_msgs::msg::Pose*>(msg);
        const auto& p    = pose.position();
        const auto& q    = pose.orientation();

        log<DEBUG>("Head pose position xyz=", p.x(), p.y(), p.z(), "orientation xyzw=", q.x(), q.y(), q.z(), q.w());

        Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
        H.translation() << p.x(), p.y(), p.z();
        H.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized().toRotationMatrix();

        std::lock_guard<std::mutex> lock(pose_mutex);
        Hrh       = H;
        have_pose = true;
    }

    K1Sensors::K1Sensors(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("K1Sensors.yaml").then([this](const Configuration& config) {
            // Use configuration here from file K1Sensors.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // The DDS reader is created once at startup, so a reload cannot re-point it at a new
            // topic. Keep using the topic we subscribed to rather than silently ignoring the change.
            const auto pose_topic = config["head_pose"]["topic"].as<std::string>();
            if (channel_created && pose_topic != cfg.pose_topic) {
                log<WARN>("head_pose topic changed but the reader already exists, restart to apply");
            }
            else {
                cfg.pose_topic = pose_topic;
            }

            cfg.odometry_deadband = config["odometry_deadband"].as<double>();

            // Hpc: pitch frame to camera optical frame
            const auto& Hpc_config = config["Hpc"];
            cfg.Hpc                = Eigen::Isometry3d::Identity();
            const auto Hpc_trans   = Hpc_config["translation"];
            cfg.Hpc.translation() << Hpc_trans[0].as<double>(), Hpc_trans[1].as<double>(), Hpc_trans[2].as<double>();
            const auto Hpc_rpy = Hpc_config["rotation_rpy"];
            cfg.Hpc.linear()   = rpy_intrinsic_to_mat(
                Eigen::Vector3d(Hpc_rpy[0].as<double>(), Hpc_rpy[1].as<double>(), Hpc_rpy[2].as<double>()));

            // Hhp: head frame to pitch frame
            const auto& Hhp_config = config["Hhp"];
            cfg.Hhp                = Eigen::Isometry3d::Identity();
            const auto Hhp_trans   = Hhp_config["translation"];
            cfg.Hhp.translation() << Hhp_trans[0].as<double>(), Hhp_trans[1].as<double>(), Hhp_trans[2].as<double>();
            const auto Hhp_rpy = Hhp_config["rotation_rpy"];
            cfg.Hhp.linear()   = rpy_intrinsic_to_mat(
                Eigen::Vector3d(Hhp_rpy[0].as<double>(), Hhp_rpy[1].as<double>(), Hhp_rpy[2].as<double>()));

            std::lock_guard<std::mutex> odometry_lock(odometry_mutex);
            booster_odometry_has_offset = false;
            booster_odometry_offset     = {};
        });


        on<Startup>().then("Subscribe to head pose", [this] {
            utility::platform::Booster::ensure_channel_factory();

            log<INFO>("Subscribing to head pose on", cfg.pose_topic);

            pose_channel = ChannelFactory::Instance()->CreateRecvChannel<geometry_msgs::msg::Pose>(
                cfg.pose_topic,
                [this](const void* msg) { pose_handler(msg); },
                /* reliable = */ true);

            channel_created = true;
        });


        // When field localisation is reset, re-capture the BoosterOdometry zero offset so that odometry is
        // reported relative to the robot's new post-reset pose.
        on<Trigger<ResetFieldLocalisation>>().then([this] {
            std::lock_guard<std::mutex> odometry_lock(odometry_mutex);
            booster_odometry_has_offset = false;
            booster_odometry_offset     = {};
            log<INFO>("K1Sensors clearing BoosterOdometry offset after field localisation reset");
        });


        // Any motion mode change causes HardwareIO to reset the Booster odometry, so re-capture the
        // zero offset from the first sample after the change, regardless of which mode was entered.
        on<Trigger<BoosterModeState>>().then([this](const BoosterModeState& mode_state) {
            std::lock_guard<std::mutex> odometry_lock(odometry_mutex);
            const int mode = static_cast<int>(mode_state.mode);
            if (last_booster_mode != -1 && mode != last_booster_mode) {
                booster_odometry_has_offset = false;
                booster_odometry_offset     = {};
                log<INFO>("K1Sensors clearing BoosterOdometry offset after mode change");
            }
            last_booster_mode = mode;
        });


        on<Trigger<RawSensors>, With<BoosterOdometry>>().then([this](const RawSensors& raw_sensors,
                                                                     const BoosterOdometry& odo) {
            std::array<double, 3> normalized_odometry{};
            {
                std::lock_guard<std::mutex> lock(odometry_mutex);
                if (!booster_odometry_has_offset) {
                    booster_odometry_offset     = {odo.x, odo.y, odo.theta};
                    booster_odometry_has_offset = true;
                    log<INFO>("K1Sensors stored BoosterOdometry zero offset",
                              booster_odometry_offset[0],
                              booster_odometry_offset[1],
                              booster_odometry_offset[2]);
                }

                normalized_odometry = {
                    odo.x - booster_odometry_offset[0],
                    odo.y - booster_odometry_offset[1],
                    odo.theta - booster_odometry_offset[2],
                };
            }

            // Snap tiny residuals to zeroes
            for (double& v : normalized_odometry) {
                if (std::abs(v) < cfg.odometry_deadband) {
                    v = 0.0;
                }
            }

            log<DEBUG>("Received odometry: x=" + std::to_string(odo.x) + ", y=" + std::to_string(odo.y) + ", theta="
                       + std::to_string(odo.theta) + " normalized x=" + std::to_string(normalized_odometry[0]) + ", y="
                       + std::to_string(normalized_odometry[1]) + ", theta=" + std::to_string(normalized_odometry[2]));


            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            Hwr.translation() << normalized_odometry[0], normalized_odometry[1], 0.0;
            // Convert yaw to rotation matrix
            Eigen::Vector3d rpy(0.0, 0.0, normalized_odometry[2]);
            Hwr.linear() = rpy_intrinsic_to_mat(rpy);

            // Hrh: head frame in robot base frame, from the most recent head pose message.
            Eigen::Isometry3d Hrh_now;
            bool got_pose = false;
            {
                std::lock_guard<std::mutex> lock(pose_mutex);
                Hrh_now  = Hrh;
                got_pose = have_pose;
            }
            if (!got_pose && !pose_warned.exchange(true)) {
                log<WARN>("No head pose received on", cfg.pose_topic, "- using identity");
            }

            // Hrc: camera optical frame in robot base frame
            Eigen::Isometry3d Hrc = Hrh_now * cfg.Hhp * cfg.Hpc;

            Eigen::Isometry3d Hwc = Hwr * Hrc;
            log<DEBUG>("Computed head pose in world frame: position xyz=",
                       Hwc.translation().x(),
                       Hwc.translation().y(),
                       Hwc.translation().z(),
                       "orientation xyzw=",
                       Eigen::Quaterniond(Hwc.linear()).x(),
                       Eigen::Quaterniond(Hwc.linear()).y(),
                       Eigen::Quaterniond(Hwc.linear()).z(),
                       Eigen::Quaterniond(Hwc.linear()).w());


            // Populate and emit the Sensors message
            auto sensors       = std::make_unique<Sensors>();
            sensors->timestamp = raw_sensors.timestamp;
            sensors->Hcw       = Hwc.inverse();
            sensors->Hrw       = Hwr.inverse();

            // Update raw sensor data including servo/joint information
            update_raw_sensors(sensors, raw_sensors);

            // Compute Htw using forward kinematics.
            // compute_Htp gives Htp: Head_2 pitch_link in Trunk (base) frame.
            // Full chain: camera_optical(c) → pitch_link(p) → Trunk(t)
            //   Htc = Htp * Hpc
            // Then: Htw = Htc * Hcw  (world → camera_optical → Trunk)
            {
                const Eigen::Isometry3d Htp = compute_Htp(sensors);
                const Eigen::Isometry3d Htc = Htp * cfg.Hpc;
                sensors->Htw                = Htc * sensors->Hcw;
            }

            bool new_left_down   = raw_sensors.buttons.left;
            bool new_middle_down = raw_sensors.buttons.middle;

            if (left_down != new_left_down) {
                left_down = new_left_down;
                if (left_down) {
                    log<INFO>("Left Button Down");
                    emit<Scope::INLINE>(std::make_unique<ButtonLeftDown>());
                }
                else {
                    log<INFO>("Left Button Up");
                    emit<Scope::INLINE>(std::make_unique<ButtonLeftUp>());
                }
            }

            if (middle_down != new_middle_down) {
                middle_down = new_middle_down;
                if (middle_down) {
                    log<INFO>("Middle Button Down");
                    emit<Scope::INLINE>(std::make_unique<ButtonMiddleDown>());
                }
                else {
                    log<INFO>("Middle Button Up");
                    emit<Scope::INLINE>(std::make_unique<ButtonMiddleUp>());
                }
            }

            emit(sensors);
        });

        on<Shutdown>().then([this] {
            if (pose_channel != nullptr) {
                log<INFO>("Closing head pose channel");
                ChannelFactory::Instance()->CloseReader(cfg.pose_topic);
            }
        });
    }

    K1Sensors::~K1Sensors() = default;

}  // namespace module::input
