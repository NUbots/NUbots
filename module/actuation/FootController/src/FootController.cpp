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


namespace module::actuation {

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
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.servo_gain         = config["servo_gain"].as<double>();
            cfg.correction_enabled = config["correction_enabled"].as<bool>();
            cfg.roll_p_gain        = config["roll_p_gain"].as<double>();
            cfg.pitch_p_gain       = config["pitch_p_gain"].as<double>();
            cfg.roll_i_gain        = config["roll_i_gain"].as<double>();
            cfg.pitch_i_gain       = config["pitch_i_gain"].as<double>();
            cfg.max_i_error        = config["max_i_error"].as<double>();
            cfg.roll_d_gain        = config["roll_d_gain"].as<double>();
            cfg.pitch_d_gain       = config["pitch_d_gain"].as<double>();
        });

        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                // Construct Leg IK tasks
                auto left_leg  = std::make_unique<LeftLegIK>();
                left_leg->time = left_foot.time;

                if (left_foot.correction_enabled && cfg.correction_enabled) {
                    // Hwt quaternion
                    Eigen::Quaterniond Hwt_quat(sensors.Htw.inverse().linear());

                    // Get fused roll and pitch
                    double fused_roll;
                    double fused_pitch;
                    FusedFromQuat(Hwt_quat, fused_pitch, fused_roll);
                    emit(graph("fused_roll", fused_roll));
                    emit(graph("fused_pitch", fused_pitch));

                    // Get the desired roll and pitch
                    Eigen::Quaterniond Hft_quat(left_foot.Htf.inverse().linear());
                    double desired_roll;
                    double desired_pitch;
                    FusedFromQuat(Hft_quat, desired_pitch, desired_roll);
                    emit(graph("desired_roll", desired_roll));
                    emit(graph("desired_pitch", desired_pitch));


                    // Compute the error between the desired torso orientation and the actual torso orientation
                    auto roll_error  = desired_roll - fused_roll;
                    auto pitch_error = desired_pitch - fused_pitch;
                    emit(graph("roll_error", roll_error));
                    emit(graph("pitch_error", pitch_error));

                    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                        - left_last_update_time)
                                  .count();
                    left_last_update_time = NUClear::clock::now();

                    // P control
                    desired_roll += cfg.roll_p_gain * roll_error;
                    desired_pitch += cfg.pitch_p_gain * pitch_error;

                    // I control
                    left_integral_roll_error += roll_error * dt;
                    left_integral_pitch_error += pitch_error * dt;

                    // Anti windup
                    left_integral_roll_error =
                        std::max(std::min(left_integral_roll_error, cfg.max_i_error), -cfg.max_i_error);
                    left_integral_pitch_error =
                        std::max(std::min(left_integral_pitch_error, cfg.max_i_error), -cfg.max_i_error);

                    desired_roll += cfg.roll_i_gain * left_integral_roll_error;
                    desired_pitch += cfg.pitch_i_gain * left_integral_pitch_error;

                    // D control

                    auto roll_error_rate  = (roll_error - left_prev_roll_error) / dt;
                    auto pitch_error_rate = (pitch_error - left_prev_pitch_error) / dt;
                    left_prev_roll_error  = roll_error;
                    left_prev_pitch_error = pitch_error;

                    desired_roll += cfg.roll_d_gain * roll_error_rate;
                    desired_pitch += cfg.pitch_d_gain * pitch_error_rate;

                    double desired_yaw = MatrixToEulerIntrinsic(Hft_quat.toRotationMatrix()).z();
                    emit(graph("corrected roll", desired_roll));
                    emit(graph("corrected pitch", desired_pitch));

                    // Compute desired orientation: yaw * fused_roll_pitch
                    Eigen::Matrix3d desired_Rft = Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ())
                                                  * QuatFromFused(desired_pitch, desired_roll).toRotationMatrix();
                    Eigen::Isometry3d Htf_corrected = left_foot.Htf;
                    Htf_corrected.linear()          = desired_Rft.transpose();
                    left_leg->Htl                   = Htf_corrected;
                }
                else {
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

                if (right_foot.correction_enabled && cfg.correction_enabled) {
                    // Hwt quaternion
                    Eigen::Quaterniond Hwt_quat(sensors.Htw.inverse().linear());

                    // Get fused roll and pitch
                    double fused_roll;
                    double fused_pitch;
                    FusedFromQuat(Hwt_quat, fused_pitch, fused_roll);
                    emit(graph("fused_roll", fused_roll));
                    emit(graph("fused_pitch", fused_pitch));

                    // Get the desired roll and pitch
                    Eigen::Quaterniond Hft_quat(right_foot.Htf.inverse().linear());
                    double desired_roll;
                    double desired_pitch;
                    FusedFromQuat(Hft_quat, desired_pitch, desired_roll);
                    emit(graph("desired_roll", desired_roll));
                    emit(graph("desired_pitch", desired_pitch));


                    // Compute the error between the desired torso orientation and the actual torso orientation
                    auto roll_error  = desired_roll - fused_roll;
                    auto pitch_error = desired_pitch - fused_pitch;
                    emit(graph("roll_error", roll_error));
                    emit(graph("pitch_error", pitch_error));

                    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                        - right_last_update_time)
                                  .count();

                    // P control
                    desired_roll += cfg.roll_p_gain * roll_error;
                    desired_pitch += cfg.pitch_p_gain * pitch_error;

                    // I control
                    right_integral_roll_error += roll_error * dt;
                    right_integral_pitch_error += pitch_error * dt;

                    // Anti windup
                    right_integral_roll_error =
                        std::max(std::min(right_integral_roll_error, cfg.max_i_error), -cfg.max_i_error);
                    right_integral_pitch_error =
                        std::max(std::min(right_integral_pitch_error, cfg.max_i_error), -cfg.max_i_error);

                    desired_roll += cfg.roll_i_gain * right_integral_roll_error;
                    desired_pitch += cfg.pitch_i_gain * right_integral_pitch_error;

                    // D control
                    right_last_update_time = NUClear::clock::now();
                    auto roll_error_rate   = (roll_error - right_prev_roll_error) / dt;
                    auto pitch_error_rate  = (pitch_error - right_prev_pitch_error) / dt;
                    right_prev_roll_error  = roll_error;
                    right_prev_pitch_error = pitch_error;

                    desired_roll += cfg.roll_d_gain * roll_error_rate;
                    desired_pitch += cfg.pitch_d_gain * pitch_error_rate;

                    double desired_yaw = MatrixToEulerIntrinsic(Hft_quat.toRotationMatrix()).z();
                    emit(graph("corrected roll", desired_roll));
                    emit(graph("corrected pitch", desired_pitch));

                    // Compute desired orientation: yaw * fused_roll_pitch
                    Eigen::Matrix3d desired_Rft = Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ())
                                                  * QuatFromFused(desired_pitch, desired_roll).toRotationMatrix();
                    Eigen::Isometry3d Htf_corrected = right_foot.Htf;
                    Htf_corrected.linear()          = desired_Rft.transpose();
                    right_leg->Htr                  = Htf_corrected;
                }
                else {
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
