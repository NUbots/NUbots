#include "K1Sensors.hpp"

#include <cmath>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterOdometry.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/nusight/NUhelpers.hpp"


namespace bip = boost::interprocess;

namespace module::input {

    using extension::Configuration;
    using message::booster::BoosterOdometry;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;


    struct SharedPoseHeader {
        bip::interprocess_mutex mutex;
        bip::interprocess_condition has_new_data;
        uint64_t sequence{0};
        double position[3]{0.0, 0.0, 0.0};
        double orientation[4]{0.0, 0.0, 0.0, 1.0};
    };

    struct PoseSharedMemory {
        explicit PoseSharedMemory(const std::string& segment)
            : shm(bip::open_only, segment.c_str(), bip::read_write), region(shm, bip::read_write) {
            header = reinterpret_cast<SharedPoseHeader*>(region.get_address());
        }

        bip::shared_memory_object shm;
        bip::mapped_region region;
        SharedPoseHeader* header = nullptr;
    };

    void K1Sensors::connect_head_pose() {
        if (cfg.pose_segment.empty() || pose_shared_memory != nullptr) {
            return;
        }

        try {
            pose_shared_memory      = std::make_unique<PoseSharedMemory>(cfg.pose_segment);
            pose_unavailable_logged = false;
            log<INFO>("K1Sensors mapped head_pose segment", cfg.pose_segment);
        }
        catch (const bip::interprocess_exception& ex) {
            if (!pose_unavailable_logged) {
                log<WARN>("K1Sensors head_pose segment unavailable", cfg.pose_segment, ex.what());
                pose_unavailable_logged = true;
            }
        }
    }

    bool K1Sensors::read_head_pose(std::array<double, 3>& position, std::array<double, 4>& orientation) {
        std::lock_guard<std::mutex> lock(pose_mutex);

        connect_head_pose();
        if (pose_shared_memory == nullptr) {
            return false;
        }

        try {
            bip::scoped_lock<bip::interprocess_mutex> pose_lock(pose_shared_memory->header->mutex, bip::try_to_lock);
            if (!pose_lock.owns()) {
                return false;
            }

            for (int i = 0; i < 3; ++i) {
                position[static_cast<size_t>(i)] = pose_shared_memory->header->position[i];
            }
            for (int i = 0; i < 4; ++i) {
                orientation[static_cast<size_t>(i)] = pose_shared_memory->header->orientation[i];
            }
            return true;
        }
        catch (const bip::interprocess_exception& ex) {
            pose_shared_memory.reset();
            if (!pose_unavailable_logged) {
                log<WARN>("K1Sensors lost access to head_pose segment", cfg.pose_segment, ex.what());
                pose_unavailable_logged = true;
            }
        }

        return false;
    }

    K1Sensors::K1Sensors(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("K1Sensors.yaml").then([this](const Configuration& config) {
            // Use configuration here from file K1Sensors.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.pose_segment = config["head_pose"][0]["segment"].as<std::string>();

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

            std::lock_guard<std::mutex> lock(pose_mutex);
            pose_shared_memory.reset();
            pose_unavailable_logged = false;
            connect_head_pose();

            std::lock_guard<std::mutex> odometry_lock(odometry_mutex);
            booster_odometry_has_offset = false;
            booster_odometry_offset     = {};
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

            // Snap tiny residuals (e.g. ~1e-10) to zero so they aren't treated as real motion
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

            std::array<double, 3> position{};
            std::array<double, 4> orientation{};
            if (read_head_pose(position, orientation)) {
                log<DEBUG>("Head pose position xyz=",
                           position[0],
                           position[1],
                           position[2],
                           "orientation xyzw=",
                           orientation[0],
                           orientation[1],
                           orientation[2],
                           orientation[3]);
            }

            // Hrh: head frame in robot base frame (from shared memory head pose)
            Eigen::Isometry3d Hrh = Eigen::Isometry3d::Identity();
            Hrh.translation() << position[0], position[1], position[2];
            Hrh.linear() =
                Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]).toRotationMatrix();

            // Hrc: camera optical frame in robot base frame = Hrh * Hhp * Hpc
            Eigen::Isometry3d Hrc = Hrh * cfg.Hhp * cfg.Hpc;

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
    }

    K1Sensors::~K1Sensors() = default;

}  // namespace module::input
