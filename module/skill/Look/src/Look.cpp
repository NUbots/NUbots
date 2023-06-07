#include "Look.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/skill/Look.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::HeadIK;
    using message::actuation::LimbsSequence;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using message::localisation::FilteredBall;
    using utility::input::ServoID;
    using utility::math::coordinates::screen_angular_from_object_direction;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::nusight::graph;
    using LookTask = message::skill::Look;

    Look::Look(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Look.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Look.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.smoothing_factor = config["smoothing_factor"].as<double>();
            cfg.head_gain        = config["head_gain"].as<double>();
            cfg.head_torque      = config["head_torque"].as<double>();
        });

        on<Provide<LookTask>, Needs<HeadIK>, Every<90, Per<std::chrono::seconds>>>().then([this](const LookTask& look) {
            // Normalise the look vector
            Eigen::Vector3d req_uPCt = look.rPCt.normalized();

            // If switching from non-smoothed to smoothed angle command, reset the initial goal angle to help
            // locking on to the target
            if (smooth == false && look.smooth == true) {
                uPCt = req_uPCt;
            }
            smooth = look.smooth;

            // If smoothing requested, smooth requested angles with exponential filter
            uPCt = smooth ? (cfg.smoothing_factor * req_uPCt + (1 - cfg.smoothing_factor) * uPCt) : req_uPCt;

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
