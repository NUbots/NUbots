/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#include "DynamicsRecorder.hpp"

#include <array>
#include <cmath>
#include <sstream>

#include "extension/Configuration.hpp"

#include "message/booster/BoosterLowCmd.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/nusim/ground_truth.hpp"

namespace module::support {

    using extension::Configuration;
    using message::booster::BoosterLowCmd;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::nusim::GroundTruthState;

    namespace {
        double seconds(NUClear::clock::time_point t) {
            return std::chrono::duration<double>(t.time_since_epoch()).count();
        }
        // ServoID of each joint, in the Booster JointIndexK1 order the commands use (skill::Walk k1_joints.hpp)
        constexpr std::array<ServoID::Value, 22> SERVO_IDS = {
            ServoID::HEAD_YAW,         ServoID::HEAD_PITCH,      ServoID::L_SHOULDER_PITCH, ServoID::L_SHOULDER_ROLL,
            ServoID::L_ELBOW,          ServoID::L_ELBOW_YAW,     ServoID::R_SHOULDER_PITCH, ServoID::R_SHOULDER_ROLL,
            ServoID::R_ELBOW,          ServoID::R_ELBOW_YAW,     ServoID::L_HIP_PITCH,      ServoID::L_HIP_ROLL,
            ServoID::L_HIP_YAW,        ServoID::L_KNEE,          ServoID::L_ANKLE_PITCH,    ServoID::L_ANKLE_ROLL,
            ServoID::R_HIP_PITCH,      ServoID::R_HIP_ROLL,      ServoID::R_HIP_YAW,        ServoID::R_KNEE,
            ServoID::R_ANKLE_PITCH,    ServoID::R_ANKLE_ROLL};
    }  // namespace

    DynamicsRecorder::DynamicsRecorder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("DynamicsRecorder.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<GroundTruthState>, Sync<DynamicsRecorder>>().then([this](const GroundTruthState& gt) {
            truth.push_back({gt.time, gt.Hst, gt.rBSs, gt.vBs});
            while (truth.size() > 100) {
                truth.pop_front();
            }
        });

        on<Trigger<BoosterLowCmd>, With<Sensors>, Sync<DynamicsRecorder>>().then(
            [this](const BoosterLowCmd& cmd, const Sensors& s) {
                if (cmd.motor_cmd.size() < 22 || truth.size() < 2 || s.servo.size() < ServoID::NUMBER_OF_SERVOS) {
                    return;
                }
                const auto now = NUClear::clock::now();
                const auto ts  = s.timestamp;
                // Ground truth at the sensor sample's time, interpolated (else the nearest)
                Truth g = truth.back();
                for (std::size_t i = 1; i < truth.size(); ++i) {
                    if (truth[i - 1].time <= ts && ts <= truth[i].time) {
                        const double span = seconds(truth[i].time) - seconds(truth[i - 1].time);
                        const double a    = span > 0 ? (seconds(ts) - seconds(truth[i - 1].time)) / span : 0.0;
                        g.Hst             = Eigen::Isometry3d::Identity();
                        g.Hst.translation() =
                            (1 - a) * truth[i - 1].Hst.translation() + a * truth[i].Hst.translation();
                        g.Hst.linear() = Eigen::Quaterniond(truth[i - 1].Hst.linear())
                                             .slerp(a, Eigen::Quaterniond(truth[i].Hst.linear()))
                                             .toRotationMatrix();
                        g.rBSs = (1 - a) * truth[i - 1].rBSs + a * truth[i].rBSs;
                        g.vBs  = (1 - a) * truth[i - 1].vBs + a * truth[i].vBs;
                        g.time = ts;
                        break;
                    }
                }
                const Eigen::Quaterniond q(g.Hst.linear());
                std::ostringstream line;
                line.precision(5);
                line << std::fixed << "DYNREC " << seconds(now) << ' ' << seconds(ts) << ' ' << seconds(g.time);
                for (const auto id : SERVO_IDS) {
                    line << ' ' << s.servo[id].present_position;
                }
                for (const auto id : SERVO_IDS) {
                    line << ' ' << s.servo[id].present_velocity;
                }
                for (int k = 0; k < 3; ++k) {
                    line << ' ' << s.gyroscope[k];
                }
                for (std::size_t j = 0; j < 22; ++j) {
                    line << ' ' << cmd.motor_cmd[j].q;
                }
                for (std::size_t j = 0; j < 22; ++j) {
                    line << ' ' << cmd.motor_cmd[j].kp;
                }
                for (std::size_t j = 0; j < 22; ++j) {
                    line << ' ' << cmd.motor_cmd[j].kd;
                }
                const auto& p = g.Hst.translation();
                line << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.w() << ' ' << q.x() << ' ' << q.y()
                     << ' ' << q.z();
                line << ' ' << g.rBSs.x() << ' ' << g.rBSs.y() << ' ' << g.rBSs.z() << ' ' << g.vBs.x() << ' '
                     << g.vBs.y() << ' ' << g.vBs.z();
                log<INFO>(line.str());
            });
    }

}  // namespace module::support
