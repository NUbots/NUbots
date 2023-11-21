#include "FootController.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {

    using extension::Configuration;

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;
    using utility::input::FrameID;

    using utility::nusight::graph;

    using utility::input::LimbID;

    FootController::FootController(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FootController.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FootController.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.servo_gain  = config["servo_gain"].as<double>();

            cfg.alpha = config["alpha"].as<double>();
            cfg.K_com = config["K_com"].as<double>();
            cfg.K_zmp = config["K_zmp"].as<double>();
            cfg.K_vel = config["K_vel"].as<double>();
            cfg.K_eos = config["K_eos"].as<double>();
            torso_offset.setZero();
        });

        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                // Construct Leg IK tasks
                auto left_leg  = std::make_unique<LeftLegIK>();
                left_leg->time = left_foot.time;

                if (left_foot.is_planted_foot) {
                    auto Hlt_desired = left_foot.Htf.inverse();
                    auto Hlt_actual  = Eigen::Isometry3d(sensors.Htx[FrameID::L_FOOT_BASE]).inverse();
                    emit(graph("COM desired (x,y,z)",
                               Hlt_desired.translation().x(),
                               Hlt_desired.translation().y(),
                               Hlt_desired.translation().z()));
                    emit(graph("COM actual (x,y,z)",
                               Hlt_actual.translation().x(),
                               Hlt_actual.translation().y(),
                               Hlt_actual.translation().z()));

                    // Compute the "COM" error
                    auto COM_error = Hlt_desired.translation() - Hlt_actual.translation();
                    emit(graph("COM error (x,y,z)", COM_error.x(), COM_error.y(), COM_error.z()));

                    // TODO: Compute the "ZMP" error

                    // TODO: Compute the "Velocity" error

                    // TODO: Compute the "End of step foot placement" error

                    // Compute time since the last update
                    auto time_delta = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                                - left_last_update_time)
                                          .count();
                    emit(graph("time_delta", time_delta));
                    left_last_update_time = NUClear::clock::now();


                    // Implement controller
                    torso_offset = cfg.K_com * COM_error.head<2>() * time_delta + (1 - cfg.alpha) * torso_offset;
                    log<NUClear::INFO>("alpha: {}", cfg.alpha);
                    emit(graph("torso_offset", torso_offset.x(), torso_offset.y()));

                    // Update the desired torso x-y position
                    Eigen::Isometry3d Hlt_control = Hlt_desired;
                    Hlt_control.translation().head<2>() += torso_offset;

                    left_leg->Htl = Hlt_control.inverse();
                }
                else {
                    // Use the desired torso level
                    left_leg->Htl = left_foot.Htf;
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

                if (right_foot.is_planted_foot) {
                    auto Hrt_desired = right_foot.Htf.inverse();
                    auto Hrt_actual  = Eigen::Isometry3d(sensors.Htx[FrameID::R_FOOT_BASE]).inverse();
                    emit(graph("COM desired (x,y,z)",
                               Hrt_desired.translation().x(),
                               Hrt_desired.translation().y(),
                               Hrt_desired.translation().z()));
                    emit(graph("COM actual (x,y,z)",
                               Hrt_actual.translation().x(),
                               Hrt_actual.translation().y(),
                               Hrt_actual.translation().z()));

                    // Compute the "COM" error
                    auto COM_error = Hrt_desired.translation() - Hrt_actual.translation();
                    emit(graph("COM error (x,y,z)", COM_error.x(), COM_error.y(), COM_error.z()));

                    // TODO: Compute the "ZMP" error

                    // TODO: Compute the "Velocity" error

                    // TODO: Compute the "End of step foot placement" error

                    // Compute time since the last update
                    auto time_delta = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                                - left_last_update_time)
                                          .count();
                    emit(graph("time_delta", time_delta));
                    left_last_update_time = NUClear::clock::now();


                    // Implement controller
                    torso_offset = cfg.K_com * COM_error.head<2>() * time_delta;  // + (1 - cfg.alpha) * torso_offset;
                    emit(graph("torso_offset", torso_offset.x(), torso_offset.y()));

                    // Update the desired torso x-y position
                    Eigen::Isometry3d Hrt_control = Hrt_desired;
                    Hrt_control.translation().head<2>() += torso_offset;

                    right_leg->Htr = Hrt_control.inverse();
                }
                else {
                    // Use the desired torso level
                    right_leg->Htr = right_foot.Htf;
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                    right_leg->servos[id] = ServoState(cfg.servo_gain, 100);
                }

                // Emit IK tasks to achieve the desired pose
                emit<Task>(right_leg, 0, false, "Control right foot");
            });
    }

}  // namespace module::actuation
