#include "FootController.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/Behaviour.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"
#include "message/skill/Walk.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"


namespace module::skill {

    using extension::Configuration;

    using message::actuation::LeftArm;
    using message::actuation::LeftLegIK;
    using message::actuation::RightArm;
    using message::actuation::RightLegIK;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::Behaviour;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    using utility::actuation::kinematics::calculateLegJoints;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;

    FootController::FootController(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FootController.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FootController.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.servo_gain  = config["servo_gain"].as<float>();
            cfg.keep_level  = config["keep_level"].as<bool>();
        });

        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                // Construct Leg IK tasks
                auto left_leg  = std::make_unique<LeftLegIK>();
                left_leg->time = left_foot.time;
                if (left_foot.keep_level && cfg.keep_level) {
                    // Calculate the desired foot orientation to keep the foot level with the ground
                    Eigen::Isometry3d Htr = Eigen::Isometry3d(sensors.Htw * sensors.Hrw.inverse());
                    Eigen::Isometry3d Htf = Eigen::Isometry3d(left_foot.Htf.cast<double>());
                    Htf.linear()          = Htr.linear();
                    left_leg->Htl         = Htf.matrix();
                }
                else {
                    left_leg->Htl = left_foot.Htf.cast<double>().matrix();
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_LEG)) {
                    left_leg->servos[id] = ServoState(cfg.servo_gain, 100);
                }

                // Emit IK tasks to achieve the desired pose
                emit<Task>(left_leg, 0, false, "Control left foot");
            });

        on<Provide<ControlRightFoot>, With<Sensors>, Needs<RightLegIK>>().then(
            [this](const ControlRightFoot& right_foot, const Sensors& sensors) {
                auto right_leg  = std::make_unique<RightLegIK>();
                right_leg->time = right_foot.time;
                right_leg->Htr  = right_foot.Htf.cast<double>().matrix();

                if (right_foot.keep_level && cfg.keep_level) {
                    // Calculate the desired foot orientation to keep the foot level with the ground
                    Eigen::Isometry3d Htr = Eigen::Isometry3d(sensors.Htw * sensors.Hrw.inverse());
                    Eigen::Isometry3d Htf = Eigen::Isometry3d(right_foot.Htf.cast<double>());
                    Htf.linear()          = Htr.linear();
                    right_leg->Htr        = Htf.matrix();
                }
                else {
                    right_leg->Htr = right_foot.Htf.cast<double>().matrix();
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                    right_leg->servos[id] = ServoState(cfg.servo_gain, 100);
                }

                // Emit IK tasks to achieve the desired pose
                emit<Task>(right_leg, 0, false, "Control right foot");
            });
    }

}  // namespace module::skill
