#include "K1Sensors.hpp"

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cmath>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterOdometry.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

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

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::support::Expression;


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

            cfg.pose_segment = "_head_pose";

            try {
                cfg.pose_segment = config["head_pose"][0]["segment"].as<std::string>();
            }
            catch (const std::exception&) {
                try {
                    cfg.pose_segment = config["head_pose"]["segment"].as<std::string>();
                }
                catch (const std::exception&) {
                    try {
                        cfg.pose_segment = config["head_pose"].as<std::string>();
                    }
                    catch (const std::exception&) {
                        log<INFO>("K1Sensors using default head_pose segment", cfg.pose_segment);
                    }
                }
            }

            try {
                const auto& Hpk_config = config["Hpk"];

                const auto translation = Hpk_config["translation"];
                cfg.Hpk.translation() << translation[0].as<double>(), translation[1].as<double>(),
                    translation[2].as<double>();

                const auto rotation = Hpk_config["rotation"];
                Eigen::Matrix3d Rpk;
                Rpk << rotation[0][0].as<double>(), rotation[0][1].as<double>(), rotation[0][2].as<double>(),
                    rotation[1][0].as<double>(), rotation[1][1].as<double>(), rotation[1][2].as<double>(),
                    rotation[2][0].as<double>(), rotation[2][1].as<double>(), rotation[2][2].as<double>();
                cfg.Hpk.linear() = Rpk;
            }
            catch (const std::exception&) {
                cfg.Hpk.translation() << -0.0122268423, 0.0440084077, -0.0283811111;
                Eigen::Matrix3d Rpk;
                Rpk << 0.0753363073, 0.0190046262, 0.996977091, -0.99715662, -0.000328667404, 0.0753561407,
                    0.00175978895, -0.999819338, 0.018925827;
                cfg.Hpk.linear() = Rpk;
            }

            // Camera extrinsic offsets (roll, pitch, yaw) [rad] applied to Hpc, same convention as the Camera module
            cfg.roll_offset  = config["roll_offset"].as<Expression>();
            cfg.pitch_offset = config["pitch_offset"].as<Expression>();
            cfg.yaw_offset   = config["yaw_offset"].as<Expression>();

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

            Eigen::Isometry3d Hrp = Eigen::Isometry3d::Identity();
            Hrp.translation() << position[0], position[1], position[2];
            Hrp.linear() =
                Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]).toRotationMatrix();

            // TRANSFORM CAMERA TO NUBOTS FRAME
            Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
            // rotate positive 90d about x to convert from camera frame (x forward, y right, z down) to NUbots frame (x
            // forward, y left, z up)
            // Hkc.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(M_PI_2, 0.0, -M_PI_2));
            Hkc.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

            // Head-pitch {p} from camera {c}
            Eigen::Isometry3d Hpc = cfg.Hpk * Hkc;

            // Apply roll, pitch, and yaw offsets, same convention as the Camera module
            log<DEBUG>("Applying camera offsets (deg): roll =",
                       cfg.roll_offset * 180.0 / M_PI,
                       "pitch =",
                       cfg.pitch_offset * 180.0 / M_PI,
                       "yaw =",
                       cfg.yaw_offset * 180.0 / M_PI);
            Hpc = Eigen::AngleAxisd(cfg.yaw_offset, Eigen::Vector3d::UnitX()).toRotationMatrix()
                  * Eigen::AngleAxisd(cfg.pitch_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                  * Eigen::AngleAxisd(cfg.roll_offset, Eigen::Vector3d::UnitY()).toRotationMatrix() * Hpc;

            Eigen::Isometry3d Hrc = Hrp * Hpc;

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

            // For K1 the robot base {r} acts as the torso {t}: publish the (offset-free) robot-from-world
            // transform so consumers (e.g. ExtrinsicsCalibration) can recover the world<-torso pose.
            sensors->Htw = sensors->Hrw;

            // Publish the offset-free head-pitch {p} frame in the robot/torso frame so the extrinsics
            // calibration can build Hwp = Hwt * Htp = Hwr * Hrp.
            sensors->Htx[FrameID::HEAD_PITCH] = Hrp.matrix();

            // Publish the head servo positions (derived from the head-pose orientation, not joint encoders)
            // so movement-gated consumers like the extrinsics calibration can tell when the head has swept.
            const Eigen::Vector3d head_rpy = mat_to_rpy_intrinsic(Hrp.linear());
            sensors->servo.resize(static_cast<size_t>(ServoID::NUMBER_OF_SERVOS));
            sensors->servo[ServoID::HEAD_YAW].id                 = static_cast<uint32_t>(ServoID::HEAD_YAW);
            sensors->servo[ServoID::HEAD_YAW].present_position   = head_rpy.z();
            sensors->servo[ServoID::HEAD_PITCH].id               = static_cast<uint32_t>(ServoID::HEAD_PITCH);
            sensors->servo[ServoID::HEAD_PITCH].present_position = head_rpy.y();

            // Buttons
            sensors->button.reserve(2);
            sensors->button.emplace_back(0, raw_sensors.buttons.left);
            sensors->button.emplace_back(1, raw_sensors.buttons.middle);

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
