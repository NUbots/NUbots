#include "MocapToReal.hpp"

#include <chrono>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/MotionCapture.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::input::MotionCapture;
    using utility::input::ServoID;
    using utility::nusight::graph;

    MocapToReal::MocapToReal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("MocapToReal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MocapToReal.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.output_to_servos = config["output_to_servos"];
            cfg.servo_gain       = config["servo_gain"].as<float>();
            cfg.servo_torque     = config["servo_torque"].as<float>();
            cfg.time_horizon     = std::chrono::milliseconds(config["time_horizon_ms"].as<int>());
            cfg.hinge_axis[0]    = config["hinge_axis"][0].as<float>();
            cfg.hinge_axis[1]    = config["hinge_axis"][1].as<float>();
            cfg.hinge_axis[2]    = config["hinge_axis"][2].as<float>();
            cfg.r_elbow_sign     = config["joint_signs"]["r_elbow"].as<float>();
            cfg.l_elbow_sign     = config["joint_signs"]["l_elbow"].as<float>();
            cfg.r_knee_sign      = config["joint_signs"]["r_knee"].as<float>();
            cfg.l_knee_sign      = config["joint_signs"]["l_knee"].as<float>();
            cfg.r_ankle_sign     = config["joint_signs"]["r_ankle"].as<float>();
            cfg.l_ankle_sign     = config["joint_signs"]["l_ankle"].as<float>();
        });

        on<Trigger<MotionCapture>>().then([this](const MotionCapture& mocap) {
            // log<DEBUG>("Got NatNet Mocap Message");
            // log<DEBUG>("Frame number: ", mocap.frame_number);
            for (const auto& skeleton : mocap.skeletons) {
                log<DEBUG>("Skeleton ID: ", skeleton.id);
                for (const auto& bone : skeleton.bones) {
                    log<DEBUG>("Bone ID: ", bone.id, " Name: ", bone.name);
                    log<DEBUG>("Bone Position: ", bone.position.transpose());
                    log<DEBUG>("Bone Rotation: ", bone.rotation.transpose());
                }

                // Right elbow
                MocapToReal::qRot qRUpperArm = {skeleton.bones[10].rotation(0),
                                                skeleton.bones[10].rotation(1),
                                                skeleton.bones[10].rotation(2),
                                                skeleton.bones[10].rotation(3)};
                MocapToReal::qRot qRForeArm  = {skeleton.bones[11].rotation(0),
                                                skeleton.bones[11].rotation(1),
                                                skeleton.bones[11].rotation(2),
                                                skeleton.bones[11].rotation(3)};

                float r_elbow_angle = cfg.r_elbow_sign * signedAngleBetween(qRUpperArm, qRForeArm, cfg.hinge_axis);


                // Left elbow
                MocapToReal::qRot qLUpperArm = {skeleton.bones[6].rotation(0),
                                                skeleton.bones[6].rotation(1),
                                                skeleton.bones[6].rotation(2),
                                                skeleton.bones[6].rotation(3)};
                MocapToReal::qRot qLForeArm  = {skeleton.bones[7].rotation(0),
                                                skeleton.bones[7].rotation(1),
                                                skeleton.bones[7].rotation(2),
                                                skeleton.bones[7].rotation(3)};

                float l_elbow_angle = cfg.l_elbow_sign * signedAngleBetween(qLUpperArm, qLForeArm, cfg.hinge_axis);


                // Right knee
                MocapToReal::qRot qRThigh = {skeleton.bones[17].rotation(0),
                                             skeleton.bones[17].rotation(1),
                                             skeleton.bones[17].rotation(2),
                                             skeleton.bones[17].rotation(3)};
                MocapToReal::qRot qRShin  = {skeleton.bones[18].rotation(0),
                                             skeleton.bones[18].rotation(1),
                                             skeleton.bones[18].rotation(2),
                                             skeleton.bones[18].rotation(3)};

                float r_knee_angle = cfg.r_knee_sign * signedAngleBetween(qRThigh, qRShin, cfg.hinge_axis);


                // Left knee
                MocapToReal::qRot qLThigh = {skeleton.bones[13].rotation(0),
                                             skeleton.bones[13].rotation(1),
                                             skeleton.bones[13].rotation(2),
                                             skeleton.bones[13].rotation(3)};
                MocapToReal::qRot qLShin  = {skeleton.bones[14].rotation(0),
                                             skeleton.bones[14].rotation(1),
                                             skeleton.bones[14].rotation(2),
                                             skeleton.bones[14].rotation(3)};

                float l_knee_angle = cfg.l_knee_sign * signedAngleBetween(qLThigh, qLShin, cfg.hinge_axis);


                // Right ankle
                MocapToReal::qRot qRFoot = {skeleton.bones[19].rotation(0),
                                            skeleton.bones[19].rotation(1),
                                            skeleton.bones[19].rotation(2),
                                            skeleton.bones[19].rotation(3)};

                float r_ankle_angle = cfg.r_ankle_sign * signedAngleBetween(qRShin, qRFoot, cfg.hinge_axis);


                // Left ankle
                MocapToReal::qRot qLFoot = {skeleton.bones[15].rotation(0),
                                            skeleton.bones[15].rotation(1),
                                            skeleton.bones[15].rotation(2),
                                            skeleton.bones[15].rotation(3)};

                float l_ankle_angle = cfg.l_ankle_sign * signedAngleBetween(qLShin, qLFoot, cfg.hinge_axis);

                constexpr float rad_to_deg = 180.0f / 3.14159265358979323846f;
                emit(graph("MocapToReal",
                           r_elbow_angle * rad_to_deg,
                           l_elbow_angle * rad_to_deg,
                           r_knee_angle * rad_to_deg,
                           l_knee_angle * rad_to_deg,
                           r_ankle_angle * rad_to_deg,
                           l_ankle_angle * rad_to_deg));

                if (cfg.output_to_servos) {
                    const NUClear::clock::time_point target_time = NUClear::clock::now() + cfg.time_horizon;
                    auto waypoints                               = std::make_unique<ServoTargets>();
                    waypoints->targets
                        .emplace_back(target_time, ServoID::R_ELBOW, r_elbow_angle, cfg.servo_gain, cfg.servo_torque);
                    waypoints->targets
                        .emplace_back(target_time, ServoID::L_ELBOW, l_elbow_angle, cfg.servo_gain, cfg.servo_torque);
                    waypoints->targets
                        .emplace_back(target_time, ServoID::R_KNEE, r_knee_angle, cfg.servo_gain, cfg.servo_torque);
                    waypoints->targets
                        .emplace_back(target_time, ServoID::L_KNEE, l_knee_angle, cfg.servo_gain, cfg.servo_torque);
                    waypoints->targets.emplace_back(target_time,
                                                    ServoID::R_ANKLE_PITCH,
                                                    r_ankle_angle,
                                                    cfg.servo_gain,
                                                    cfg.servo_torque);
                    waypoints->targets.emplace_back(target_time,
                                                    ServoID::L_ANKLE_PITCH,
                                                    l_ankle_angle,
                                                    cfg.servo_gain,
                                                    cfg.servo_torque);
                    emit(std::move(waypoints));
                }
            }
        });
    }
    MocapToReal::qRot MocapToReal::qInvert(MocapToReal::qRot qRotation) {
        return {-qRotation.x, -qRotation.y, -qRotation.z, qRotation.t};
    }

    MocapToReal::qRot MocapToReal::qMultiply(
        MocapToReal::qRot qRotationA,
        MocapToReal::qRot qRotationB) {  // this is not commutative so lock in before using
        MocapToReal::qRot qResult;
        qResult.t = qRotationA.t * qRotationB.t - qRotationA.x * qRotationB.x - qRotationA.y * qRotationB.y
                    - qRotationA.z * qRotationB.z;
        qResult.x = qRotationA.t * qRotationB.x + qRotationA.x * qRotationB.t + qRotationA.y * qRotationB.z
                    - qRotationA.z * qRotationB.y;
        qResult.y = qRotationA.t * qRotationB.y - qRotationA.x * qRotationB.z + qRotationA.y * qRotationB.t
                    + qRotationA.z * qRotationB.x;
        qResult.z = qRotationA.t * qRotationB.z + qRotationA.x * qRotationB.y - qRotationA.y * qRotationB.x
                    + qRotationA.z * qRotationB.t;

        return {qResult};
    }

    float MocapToReal::angleBetween(MocapToReal::qRot qRotationA, MocapToReal::qRot qRotationB) {
        MocapToReal::qRot qResult = qMultiply(qInvert(qRotationA), qRotationB);
        return 2 * std::acos(qResult.t);
    }

    float MocapToReal::signedAngleBetween(MocapToReal::qRot qRotationA,
                                          MocapToReal::qRot qRotationB,
                                          const std::array<float, 3>& axis) {
        MocapToReal::qRot qResult = qMultiply(qInvert(qRotationA), qRotationB);
        // Twist (swing-twist decomposition) of the relative rotation about the hinge axis. The
        // projection onto the axis carries the sign that 2*acos(t) throws away.
        float projection = qResult.x * axis[0] + qResult.y * axis[1] + qResult.z * axis[2];
        float angle      = 2.0f * std::atan2(projection, qResult.t);
        // atan2 gives (-2pi, 2pi]; wrap to [-pi, pi] so opposite bend directions keep opposite signs
        constexpr float pi = 3.14159265358979323846f;
        if (angle > pi) {
            angle -= 2.0f * pi;
        }
        else if (angle < -pi) {
            angle += 2.0f * pi;
        }
        return angle;
    }
}  // namespace module::actuation
