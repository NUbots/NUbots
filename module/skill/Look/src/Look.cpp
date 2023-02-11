#include "Look.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Look.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::HeadIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::nusight::graph;

    Look::Look(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Look.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Look.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.smoothing_factor = config["smoothing_factor"].as<float>();
            cfg.head_gain        = config["head_gain"].as<float>();
            cfg.head_torque      = config["head_torque"].as<float>();
        });

        on<Startup>().then([this] { emit<Task>(std::make_unique<message::skill::Look>(-0.5, 0.0, true)); });

        on<Provide<message::skill::Look>, Needs<HeadIK>, Trigger<Sensors>>().then(
            [this](const message::skill::Look& look, const Sensors& sensors) {
                log<NUClear::WARN>("looking...");
                emit(graph("Look Goal Angles", look.yaw, look.pitch));

                // Set the requested angles and the current head angles for smoothing
                const Eigen::Vector2d requested_angles(look.yaw, look.pitch);
                Eigen::Vector2d current_angles(sensors.servo[static_cast<int>(ServoID::HEAD_YAW)].present_position,
                                               sensors.servo[static_cast<int>(ServoID::HEAD_PITCH)].present_position);

                // If switching from non-smoothed to smoothed angle command, reset the initial goal angle to help
                // locking on to the target
                if (smooth == false && look.smooth == true) {
                    current_angles = requested_angles;
                }
                smooth = look.smooth;

                // If smoothing requested, smooth requested angles with exponential filter
                current_angles =
                    smooth ? (cfg.smoothing_factor * requested_angles + (1 - cfg.smoothing_factor) * current_angles)
                           : requested_angles;

                // Get look vector from angles
                const Eigen::Vector3d uPCt =
                    sphericalToCartesian(Eigen::Vector3d(1, current_angles.x(), current_angles.y()));

                // Create the HeadIK message
                auto head_ik  = std::make_unique<HeadIK>();
                head_ik->time = NUClear::clock::now();
                head_ik->uPCt = uPCt;

                head_ik->servos[ServoID::HEAD_YAW]   = ServoState(cfg.head_gain, cfg.head_torque);
                head_ik->servos[ServoID::HEAD_PITCH] = ServoState(cfg.head_gain, cfg.head_torque);

                emit<Task>(head_ik);
            });
    }

}  // namespace module::skill
