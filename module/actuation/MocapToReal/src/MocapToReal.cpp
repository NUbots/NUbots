#include "MocapToReal.hpp"

#include "extension/Configuration.hpp"

#include "message/input/MotionCapture.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::input::MotionCapture;

    MocapToReal::MocapToReal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("MocapToReal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MocapToReal.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.output_to_servos = config["output_to_servos"];
        });

        on<Trigger<MotionCapture>>().then(
            [this](const MotionCapture& mocap) { log<DEBUG>("Got NatNet Mocap Message"); });
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
