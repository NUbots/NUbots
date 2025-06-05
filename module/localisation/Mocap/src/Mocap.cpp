#include "Mocap.hpp"

#include "extension/Configuration.hpp"

#include "message/input/MotionCapture.hpp"
#include "message/localisation/Field.hpp"

#include "utility/math/euler.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::input::MotionCapture;
    using message::localisation::RobotPoseGroundTruth;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    Mocap::Mocap(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mocap.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mocap.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.robot_rigid_body_id = config["robot_rigid_body_id"].as<uint32_t>();
        });

        on<Trigger<MotionCapture>>().then([this](const MotionCapture& motion_capture) {
            log<DEBUG>("Motion Capture Data Received - Number of Rigid Bodies: ", motion_capture.rigid_bodies.size());
            for (const auto& rigid_body : motion_capture.rigid_bodies) {
                if (rigid_body.id == cfg.robot_rigid_body_id) {
                    log<DEBUG>("Found robot rigid body");
                    log<DEBUG>("Rigid Body: ", rigid_body.name);
                    log<DEBUG>("Rigid Body ID: ", rigid_body.id);
                    log<DEBUG>("Rigid Body Position: ", rigid_body.position);
                    log<DEBUG>("Rigid Body Rotation: ", rigid_body.rotation);
                    auto robot_pose_ground_truth = std::make_unique<message::localisation::RobotPoseGroundTruth>();

                    // Build transform from field {b} frame to torso {t} frame
                    Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
                    Hft.translation() =
                        Eigen::Vector3d(-rigid_body.position.y(), rigid_body.position.x(), rigid_body.position.z());
                    auto rpy                      = mat_to_rpy_intrinsic(Eigen::Quaterniond(rigid_body.rotation(0),
                                                                       rigid_body.rotation(1),
                                                                       rigid_body.rotation(2),
                                                                       rigid_body.rotation(3))
                                                        .toRotationMatrix());
                    Eigen::Vector3d rpy_corrected = Eigen::Vector3d(rpy.y(), -(rpy.z() - M_PI), rpy.x());
                    Hft.linear()                  = rpy_intrinsic_to_mat(rpy_corrected);

                    robot_pose_ground_truth->Hft = Hft;
                    emit(robot_pose_ground_truth);
                }
            }
        });
    }

}  // namespace module::localisation
