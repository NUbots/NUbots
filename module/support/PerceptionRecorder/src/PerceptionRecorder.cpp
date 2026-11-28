/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#include "PerceptionRecorder.hpp"

#include <cmath>
#include <fmt/format.h>
#include <optional>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusim/ground_truth.hpp"

namespace module::support {

    using extension::Configuration;
    using message::input::Sensors;
    using message::vision::Balls;
    using utility::input::ServoID;
    using utility::nusim::GroundTruthState;

    namespace {
        double seconds(NUClear::clock::time_point t) {
            return std::chrono::duration<double>(t.time_since_epoch()).count();
        }
    }  // namespace

    PerceptionRecorder::PerceptionRecorder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("PerceptionRecorder.yaml").then([this](const Configuration& config) {
            log_level   = config["log_level"].as<NUClear::LogLevel>();
            auto o      = config["camera_offset"].as<std::vector<double>>();
            offset      = Eigen::Vector3d(o[0], o[1], o[2]);
            ball_radius = config["ball_radius"].as<double>();
        });

        // The camera's mounting on the head, as skill::Walk takes it
        on<Configuration>("Sensors.yaml").then([this](const Configuration& config) {
            const auto c = config["Hpc"];
            Hpc          = Eigen::Isometry3d::Identity();
            Hpc.translation() << c["translation"][0].as<double>(), c["translation"][1].as<double>(),
                c["translation"][2].as<double>();
            Hpc.linear() = utility::math::euler::rpy_intrinsic_to_mat(Eigen::Vector3d(c["rotation_rpy"][0].as<double>(),
                                                                                      c["rotation_rpy"][1].as<double>(),
                                                                                      c["rotation_rpy"][2].as<double>()));
        });

        on<Trigger<GroundTruthState>, Sync<PerceptionRecorder>>().then([this](const GroundTruthState& gt) {
            truth.push_back({gt.time, gt.Hst, gt.rBSs, gt.vBs});
            while (truth.size() > 200) {
                truth.pop_front();
            }
        });

        on<Trigger<Sensors>, Sync<PerceptionRecorder>>().then([this](const Sensors& s) {
            Motion m;
            m.time           = s.timestamp;
            m.Htw            = Eigen::Isometry3d(s.Htw);
            m.Hcw            = Eigen::Isometry3d(s.Hcw);
            m.gyro           = Eigen::Vector3d(s.gyroscope);
            m.head_yaw_vel   = s.servo[ServoID::HEAD_YAW].present_velocity;
            m.head_pitch_vel = s.servo[ServoID::HEAD_PITCH].present_velocity;
            m.head_yaw       = s.servo[ServoID::HEAD_YAW].present_position;
            m.head_pitch     = s.servo[ServoID::HEAD_PITCH].present_position;
            motion.push_back(m);
            while (motion.size() > 400) {
                motion.pop_front();
            }
        });

        on<Trigger<Balls>, Sync<PerceptionRecorder>>().then([this](const Balls& balls) {
            const auto t_img = balls.timestamp;
            const auto now   = NUClear::clock::now();
            if (truth.size() < 2 || motion.empty()) {
                return;
            }
            // Ground truth at the image time, interpolated between the samples either side
            std::optional<Truth> gt;
            for (std::size_t i = 1; i < truth.size(); ++i) {
                if (truth[i - 1].time <= t_img && t_img <= truth[i].time) {
                    const double span = seconds(truth[i].time) - seconds(truth[i - 1].time);
                    const double a    = span > 0 ? (seconds(t_img) - seconds(truth[i - 1].time)) / span : 0.0;
                    Truth g;
                    g.time = t_img;
                    g.Hst  = Eigen::Isometry3d::Identity();
                    g.Hst.translation() =
                        (1 - a) * truth[i - 1].Hst.translation() + a * truth[i].Hst.translation();
                    g.Hst.linear() = Eigen::Quaterniond(truth[i - 1].Hst.linear())
                                         .slerp(a, Eigen::Quaterniond(truth[i].Hst.linear()))
                                         .toRotationMatrix();
                    g.rBSs = (1 - a) * truth[i - 1].rBSs + a * truth[i].rBSs;
                    g.vBs  = (1 - a) * truth[i - 1].vBs + a * truth[i].vBs;
                    gt     = g;
                    break;
                }
            }
            if (!gt) {
                return;
            }
            // Robot motion nearest the image time
            const Motion* m = &motion.front();
            for (const auto& x : motion) {
                if (std::abs(seconds(x.time) - seconds(t_img)) < std::abs(seconds(m->time) - seconds(t_img))) {
                    m = &x;
                }
            }
            const Eigen::Isometry3d Hcw(balls.Hcw);
            const Eigen::Isometry3d Hct = m->Hcw * m->Htw.inverse();
            // True ball: torso frame from ground truth, then camera and the policies' camera frame
            const Eigen::Vector3d rBTt = gt->Hst.inverse() * gt->rBSs;
            const Eigen::Vector3d tru  = Hpc * (Hct * rBTt) - offset;
            // Detection: nearest to the camera, on the ball centre, as skill::Walk's ball skills take it
            const Eigen::Isometry3d Hwc = Hcw.inverse();
            const Eigen::Vector3d rCWw  = Hwc.translation();
            std::optional<Eigen::Vector3d> best{};
            Eigen::Vector3d best_u = Eigen::Vector3d::Zero();
            double best_r          = 0.0;
            int n                  = 0;
            for (const auto& b : balls.balls) {
                if (b.is_invalid) {
                    continue;
                }
                ++n;
                const Eigen::Vector3d u = Hwc.linear() * Eigen::Vector3d(b.uBCc).normalized();
                if (u.z() > -1e-6) {
                    continue;
                }
                const double t            = (ball_radius - rCWw.z()) / u.z();
                const Eigen::Vector3d rBWw = rCWw + t * u;
                if (!best || (rBWw - rCWw).norm() < (*best - rCWw).norm()) {
                    best   = rBWw;
                    best_u = Eigen::Vector3d(b.uBCc).normalized();
                    best_r = b.radius;
                }
            }
            Eigen::Vector3d meas = Eigen::Vector3d::Constant(NAN);
            if (best) {
                meas = Hpc * (Hcw * *best) - offset;
            }
            const Eigen::Vector3d g = m->Htw.linear() * Eigen::Vector3d(0.0, 0.0, -1.0);
            // True gravity in the torso at the motion sample's time (nearest ground-truth sample)
            const Truth* gm = &truth.front();
            for (const auto& x : truth) {
                if (std::abs(seconds(x.time) - seconds(m->time)) < std::abs(seconds(gm->time) - seconds(m->time))) {
                    gm = &x;
                }
            }
            const Eigen::Vector3d g_true = gm->Hst.linear().transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
            log<INFO>(fmt::format(
                "PAMREC {:.4f} {:.4f} {} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.5f} "
                "{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} "
                "{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}",
                seconds(t_img), seconds(now) - seconds(t_img), n,
                tru.x(), tru.y(), tru.z(), meas.x(), meas.y(), meas.z(),
                best_u.x(), best_u.y(), best_u.z(), best_r,
                m->gyro.x(), m->gyro.y(), m->gyro.z(), m->head_yaw_vel, m->head_pitch_vel, m->head_yaw, m->head_pitch,
                g.x(), g.y(), g.z(), rBTt.x(), rBTt.y(), rBTt.z(),
                gt->vBs.x(), gt->vBs.y(), gt->vBs.z(), seconds(m->time) - seconds(t_img), seconds(now) - seconds(t_img), g_true.x(), g_true.y(), g_true.z(),
                seconds(gm->time) - seconds(m->time)));
        });
    }

}  // namespace module::support
