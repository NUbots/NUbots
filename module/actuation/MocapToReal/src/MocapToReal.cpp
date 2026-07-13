#include "MocapToReal.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/MotionCapture.hpp"

#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::input::MotionCapture;
    using utility::nusight::graph;

    MocapToReal::MocapToReal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("MocapToReal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MocapToReal.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.output_to_servos = config["output_to_servos"];
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

                float r_elbow_angle_degrees = angleBetween(qRUpperArm, qRForeArm) * 180.0f / 3.14159265358979323846f;


                // Left elbow
                MocapToReal::qRot qLUpperArm = {skeleton.bones[6].rotation(0),
                                                skeleton.bones[6].rotation(1),
                                                skeleton.bones[6].rotation(2),
                                                skeleton.bones[6].rotation(3)};
                MocapToReal::qRot qLForeArm  = {skeleton.bones[7].rotation(0),
                                                skeleton.bones[7].rotation(1),
                                                skeleton.bones[7].rotation(2),
                                                skeleton.bones[7].rotation(3)};

                float l_elbow_angle_degrees = angleBetween(qLUpperArm, qLForeArm) * 180.0f / 3.14159265358979323846f;


                // Right knee
                MocapToReal::qRot qRThigh = {skeleton.bones[17].rotation(0),
                                             skeleton.bones[17].rotation(1),
                                             skeleton.bones[17].rotation(2),
                                             skeleton.bones[17].rotation(3)};
                MocapToReal::qRot qRShin  = {skeleton.bones[18].rotation(0),
                                             skeleton.bones[18].rotation(1),
                                             skeleton.bones[18].rotation(2),
                                             skeleton.bones[18].rotation(3)};

                float r_knee_angle_degrees = angleBetween(qRThigh, qRShin) * 180.0f / 3.14159265358979323846f;


                // Left knee
                MocapToReal::qRot qLThigh = {skeleton.bones[13].rotation(0),
                                             skeleton.bones[13].rotation(1),
                                             skeleton.bones[13].rotation(2),
                                             skeleton.bones[13].rotation(3)};
                MocapToReal::qRot qLShin  = {skeleton.bones[14].rotation(0),
                                             skeleton.bones[14].rotation(1),
                                             skeleton.bones[14].rotation(2),
                                             skeleton.bones[14].rotation(3)};

                float l_knee_angle_degrees = angleBetween(qLThigh, qLShin) * 180.0f / 3.14159265358979323846f;


                // Right ankle
                MocapToReal::qRot qRFoot = {skeleton.bones[19].rotation(0),
                                            skeleton.bones[19].rotation(1),
                                            skeleton.bones[19].rotation(2),
                                            skeleton.bones[19].rotation(3)};

                float r_ankle_angle_degrees = angleBetween(qRShin, qRFoot) * 180.0f / 3.14159265358979323846f;


                // Left ankle
                MocapToReal::qRot qLFoot = {skeleton.bones[15].rotation(0),
                                            skeleton.bones[15].rotation(1),
                                            skeleton.bones[15].rotation(2),
                                            skeleton.bones[15].rotation(3)};

                float l_ankle_angle_degrees = angleBetween(qLShin, qLFoot) * 180.0f / 3.14159265358979323846f;

                emit(graph("MocapToReal",
                           r_elbow_angle_degrees,
                           l_elbow_angle_degrees,
                           r_knee_angle_degrees,
                           l_knee_angle_degrees,
                           r_ankle_angle_degrees,
                           l_ankle_angle_degrees));

                if (cfg.output_to_servos) {
                    auto waypoints = std::make_unique<ServoTargets>();
                    waypoints->targets.push_back(ServoTarget{.id = 5, .angle = r_elbow_angle_degrees});
                    waypoints->targets.push_back(ServoTarget{.id = 6, .angle = l_elbow_angle_degrees});
                    waypoints->targets.push_back(ServoTarget{.id = 13, .angle = r_knee_angle_degrees});
                    waypoints->targets.push_back(ServoTarget{.id = 14, .angle = l_knee_angle_degrees});
                    waypoints->targets.push_back(ServoTarget{.id = 15, .angle = r_ankle_angle_degrees});
                    waypoints->targets.push_back(ServoTarget{.id = 16, .angle = l_ankle_angle_degrees});
                    emit(waypoints);
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
}  // namespace module::actuation
