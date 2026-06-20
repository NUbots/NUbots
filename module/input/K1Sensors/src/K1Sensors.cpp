#include "K1Sensors.hpp"

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterOdometry.hpp"
#include "message/input/Sensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace bip = boost::interprocess;

namespace module::input {

    using extension::Configuration;
    using message::booster::BoosterOdometry;
    using message::input::Sensors;

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

            std::lock_guard<std::mutex> lock(pose_mutex);
            pose_shared_memory.reset();
            pose_unavailable_logged = false;
            connect_head_pose();
        });


        on<Trigger<BoosterOdometry>>().then([this](const BoosterOdometry& odo) {
            log<DEBUG>("Received odometry: x=" + std::to_string(odo.x) + ", y=" + std::to_string(odo.y)
                       + ", theta=" + std::to_string(odo.theta));


            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            Hwr.translation() << odo.x, odo.y, 0.0;
            // Convert yaw to rotation matrix
            Eigen::Vector3d rpy(0.0, 0.0, odo.theta);
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

            // Pitch to camera
            // -
            //   - 0.0753363073
            //   - 0.0190046262
            //   - 0.996977091
            //   - -0.0122268423
            // -
            //   - -0.99715662
            //   - -0.000328667404
            //   - 0.0753561407
            //   - 0.0440084077
            // -
            //   - 0.00175978895
            //   - -0.999819338
            //   - 0.018925827
            //   - -0.0283811111
            // -
            //   - 0
            //   - 0
            //   - 0
            //   - 1
            Eigen::Isometry3d Hpk = Eigen::Isometry3d::Identity();
            Hpk.translation() << -0.0122268423, 0.0440084077, -0.0283811111;
            Eigen::Matrix3d Rpk;
            Rpk << 0.0753363073, 0.0190046262, 0.996977091, -0.99715662, -0.000328667404, 0.0753561407, 0.00175978895,
                -0.999819338, 0.018925827;
            Hpk.linear() = Rpk;

            Eigen::Isometry3d Hrk = Hrp * Hpk;


            // TRANSFORM CAMERA TO NUBOTS FRAME
            Eigen::Isometry3d Hkc = Eigen::Isometry3d::Identity();
            // rotate positive 90d about x to convert from camera frame (x forward, y right, z down) to NUbots frame (x
            // forward, y left, z up)
            // Hkc.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(M_PI_2, 0.0, -M_PI_2));
            Hkc.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
            Eigen::Isometry3d Hrc = Hrk * Hkc;

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
            sensors->timestamp = NUClear::clock::now();
            sensors->Hcw       = Hwc.inverse();

            // TEMP: Set Htw to Hcw for now until we have a torso pose source
            sensors->Htw = sensors->Hcw;


            emit(sensors);
        });
    }

    K1Sensors::~K1Sensors() = default;

}  // namespace module::input
