/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#include "DribbleBench.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/platform/NUSimBallCommand.hpp"
#include "message/skill/BallSkill.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"

#include "utility/nusim/ground_truth.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::platform::NUSimBallCommand;
    using message::skill::BallSkill;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using utility::nusim::GroundTruthState;

    namespace {
        constexpr double DT             = 0.02;
        constexpr int SKILL_PRIORITY    = 2;
        constexpr double FIG8_SPEED     = 0.6;
        constexpr double WALK_DRIB_SPD  = 0.8;
        constexpr double STRAIGHT_S     = 12.0;
        constexpr double STRAIGHT_FROM  = 4.0;
        constexpr double WALK_DRIB_S    = 6.0;
        constexpr double MAX_APPROACH_S = 15.0;

        double wrap(double a) {
            return std::atan2(std::sin(a), std::cos(a));
        }
        Eigen::Vector2d rot(double yaw, const Eigen::Vector2d& v) {
            return Eigen::Rotation2Dd(yaw) * v;
        }
        double uniform(std::mt19937& rng, double lo, double hi) {
            return std::uniform_real_distribution<double>(lo, hi)(rng);
        }
    }  // namespace

    DribbleBench::DribbleBench(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration, Sync<DribbleBench>>("DribbleBench.yaml").then([this](const Configuration& config) {
            log_level                  = config["log_level"].as<NUClear::LogLevel>();
            cfg.speeds                 = config["straight_speeds"].as<std::vector<double>>();
            cfg.straight_repeats       = config["straight_repeats"].as<int>();
            cfg.radii                  = config["figure8_radii"].as<std::vector<double>>();
            cfg.figure8_repeats        = config["figure8_repeats"].as<int>();
            cfg.figure8_circle_timeout = config["figure8_circle_timeout"].as<double>();
            cfg.walk_repeats           = config["walk_to_dribble_repeats"].as<int>();
            cfg.seed                   = config["seed"].as<unsigned>();
            rng.seed(cfg.seed);
            trials.clear();
            for (int r = 0; r < cfg.straight_repeats; ++r) {
                for (double s : cfg.speeds) {
                    trials.push_back({Kind::STRAIGHT, s});
                }
            }
            for (int r = 0; r < cfg.figure8_repeats; ++r) {
                for (double radius : cfg.radii) {
                    trials.push_back({Kind::FIGURE8, radius});
                }
            }
            for (int r = 0; r < cfg.walk_repeats; ++r) {
                trials.push_back({Kind::WALK_TO_DRIBBLE, 0.0});
            }
            trial = 0;
            set_phase(Phase::RECOVER);
            log<INFO>("BENCH", trials.size(), "trials queued");
        });

        on<Startup>().then([this] {
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            emit<Task>(std::make_unique<FallRecovery>(), 5);
        });

        on<Trigger<Stability>, Sync<DribbleBench>>().then([this](const Stability& s) {  //
            fallen = s == Stability::FALLEN;
        });

        on<Trigger<GroundTruthState>, Sync<DribbleBench>>().then([this](const GroundTruthState& gt) {
            robot     = gt.Hst.translation().head<2>();
            robot_z   = gt.Hst.translation().z();
            robot_yaw = std::atan2(gt.Hst.linear()(1, 0), gt.Hst.linear()(0, 0));
            ball      = gt.rBSs.head<2>();
            have_truth = true;
        });

        on<Every<50, Per<std::chrono::seconds>>, Sync<DribbleBench>, Single>().then([this] { tick(); });
    }

    void DribbleBench::set_phase(Phase p) {
        phase   = p;
        phase_t = 0.0;
    }

    void DribbleBench::walk(double vx, double wz) {
        emit<Task>(std::unique_ptr<BallSkill>(nullptr));
        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(vx, 0.0, wz)), SKILL_PRIORITY);
        // Gaze ahead and down at the pitch, as a walk up to the ball would
        emit<Task>(std::make_unique<Look>(Eigen::Vector3d(std::cos(0.4), 0.0, -std::sin(0.4)), false));
    }

    void DribbleBench::dribble(const Eigen::Vector2d& command_world) {
        emit<Task>(std::unique_ptr<Walk>(nullptr));
        auto task      = std::make_unique<BallSkill>();
        task->type     = BallSkill::Type::DRIBBLE;
        task->velocity = rot(-robot_yaw, command_world);
        emit<Task>(std::move(task), SKILL_PRIORITY);
    }

    void DribbleBench::place_ball(double distance, double bearing) {
        auto command      = std::make_unique<NUSimBallCommand>();
        command->frame    = NUSimBallCommand::Frame::ROBOT;
        command->position = Eigen::Vector3d(distance * std::cos(bearing), distance * std::sin(bearing), -1.0);
        command->velocity = Eigen::Vector3d::Zero();
        command->rolling  = true;
        emit(command);
    }

    void DribbleBench::start_trial() {
        lost = fell = false;
        why.clear();
        walk_t       = 0.0;
        side         = 0;
        swept        = 0.0;
        fresh_circle = true;
        circle_t     = 0.0;
        err_max      = 0.0;
        circles_done = 0;
        const Trial& t = trials[trial];
        if (t.kind == Kind::STRAIGHT) {
            direction = rot(robot_yaw + uniform(rng, -M_PI / 6.0, M_PI / 6.0), Eigen::Vector2d::UnitX());
            along_start = 0.0;
        }
        else if (t.kind == Kind::FIGURE8) {
            origin    = ball;
            frame_yaw = robot_yaw;
        }
    }

    void DribbleBench::end_trial() {
        const Trial& t = trials[trial];
        if (t.kind == Kind::STRAIGHT) {
            const double along = (ball.dot(direction) - along_start) / (STRAIGHT_S - STRAIGHT_FROM);
            const bool full    = phase_t >= STRAIGHT_S - 1e-6;
            results["straight"].push_back({t.value, double(fell), double(lost), full ? along : NAN});
            log<INFO>(fmt::format("BENCH RESULT straight {:.1f} m/s: ball speed along {:.2f} m/s, fell {}, lost {}",
                                  t.value,
                                  full ? along : NAN,
                                  int(fell),
                                  int(lost)));
        }
        else if (t.kind == Kind::FIGURE8) {
            results["figure8"].push_back({t.value, double(circles_done), double(fell), err_max});
            log<INFO>(fmt::format("BENCH RESULT figure8 {:.1f} m: {} of 2 circles, max line error {:.2f} m{}",
                                  t.value,
                                  circles_done,
                                  err_max,
                                  why.empty() ? "" : ", " + why));
        }
        else {
            const bool reached = phase == Phase::RUN;
            const double along = reached && !fell ? (ball.dot(direction) - along_start) / WALK_DRIB_S : NAN;
            results["walk_to_dribble"].push_back({double(reached), double(fell), double(lost), along});
            log<INFO>(fmt::format("BENCH RESULT walk->dribble: reached {}, fell {}, lost {}, ball speed {:.2f} m/s{}",
                                  int(reached),
                                  int(fell),
                                  int(lost),
                                  along,
                                  why.empty() ? "" : ", " + why));
        }
        ++trial;
        if (trial >= trials.size()) {
            summary();
            set_phase(Phase::DONE);
        }
        else {
            set_phase(Phase::RECOVER);
        }
    }

    void DribbleBench::tick() {
        if (!have_truth || trials.empty()) {
            return;
        }
        phase_t += DT;
        const bool down = fallen || robot_z < 0.3;
        upright_t       = down ? 0.0 : upright_t + DT;
        const bool live = phase == Phase::HOLD || phase == Phase::APPROACH || phase == Phase::RUN;
        if (live && down) {
            fell = true;
            why  = "fell";
            end_trial();
            return;
        }
        const Trial& t = trials[std::min(trial, trials.size() - 1)];

        switch (phase) {
            case Phase::DONE: walk(0.0, 0.0); return;
            case Phase::RECOVER:
                if (!down) {
                    walk(0.0, 0.0);
                }
                if (upright_t > 2.0) {
                    set_phase(Phase::FACE);
                }
                return;
            case Phase::FACE: {
                // Turn towards the field centre when away from it, to stay on the pitch
                const double target = robot.norm() > 1.0 ? std::atan2(-robot.y(), -robot.x()) : robot_yaw;
                const double err    = wrap(target - robot_yaw);
                walk(0.0, std::clamp(1.5 * err, -0.8, 0.8));
                if (std::abs(err) < 0.15 || phase_t > 8.0) {
                    set_phase(Phase::SETTLE);
                }
                return;
            }
            case Phase::SETTLE:
                walk(0.0, 0.0);
                if (phase_t > 1.5) {
                    if (t.kind == Kind::WALK_TO_DRIBBLE) {
                        place_ball(uniform(rng, 2.0, 3.0), uniform(rng, -0.6, 0.6));
                        switch_at = uniform(rng, 0.45, 0.75);
                    }
                    else {
                        place_ball(0.5, 0.0);
                    }
                    set_phase(Phase::PLACE);
                }
                return;
            case Phase::PLACE:
                walk(0.0, 0.0);
                if (phase_t > 0.7) {
                    start_trial();
                    set_phase(t.kind == Kind::WALK_TO_DRIBBLE ? Phase::APPROACH : Phase::HOLD);
                }
                return;
            case Phase::HOLD:
                dribble(Eigen::Vector2d::Zero());
                if (phase_t > 1.5) {
                    start_trial();
                    set_phase(Phase::RUN);
                }
                return;
            case Phase::APPROACH: {
                const Eigen::Vector2d rel = ball - robot;
                const double bearing      = wrap(std::atan2(rel.y(), rel.x()) - robot_yaw);
                walk(std::abs(bearing) > 0.5 ? 0.2 : 0.5, std::clamp(1.5 * bearing, -0.8, 0.8));
                walk_t += DT;
                if (rel.norm() < switch_at) {
                    direction = rot(robot_yaw + uniform(rng, -M_PI / 3.0, M_PI / 3.0), Eigen::Vector2d::UnitX());
                    along_start = ball.dot(direction);
                    set_phase(Phase::RUN);
                }
                else if (walk_t > MAX_APPROACH_S) {
                    why = "never reached the ball";
                    end_trial();
                }
                return;
            }
            case Phase::RUN: break;
        }

        // Running a trial
        const double gap = (ball - robot).norm();
        if (t.kind == Kind::STRAIGHT) {
            dribble(t.value * direction);
            lost |= gap > 1.0;
            if (phase_t < STRAIGHT_FROM) {
                along_start = ball.dot(direction);
            }
            if (phase_t >= STRAIGHT_S) {
                end_trial();
            }
            return;
        }
        if (t.kind == Kind::WALK_TO_DRIBBLE) {
            dribble(WALK_DRIB_SPD * direction);
            lost |= gap > 1.0;
            if (phase_t >= WALK_DRIB_S) {
                end_trial();
            }
            return;
        }

        // Figure of eight, in the frame of the start: x along the robot's heading, origin at the ball's start
        const double R              = t.value;
        const double sign           = side == 0 ? 1.0 : -1.0;  // upper circle anticlockwise, lower clockwise
        const Eigen::Vector2d centre(0.0, sign * R);
        const Eigen::Vector2d b   = rot(-frame_yaw, ball - origin);
        const Eigen::Vector2d rel = b - centre;
        const double dist         = rel.norm();
        const double angle        = std::atan2(rel.y(), rel.x());
        if (fresh_circle) {
            prev_angle   = -sign * M_PI / 2.0;
            fresh_circle = false;
        }
        swept += sign * wrap(angle - prev_angle);
        prev_angle = angle;
        circle_t += DT;
        const double err = std::abs(dist - R);
        err_max          = std::max(err_max, err);
        if (err > 0.15 + 0.5 * R) {
            why = "ball left the line";
            end_trial();
            return;
        }
        if (gap > 1.0) {
            why = "ball got away";
            end_trial();
            return;
        }
        if (circle_t > cfg.figure8_circle_timeout) {
            why = "stalled";
            end_trial();
            return;
        }
        if (swept >= 2.0 * M_PI) {
            ++circles_done;
            if (side == 1) {
                end_trial();
                return;
            }
            side         = 1;
            swept        = 0.0;
            fresh_circle = true;
            circle_t     = 0.0;
        }
        const double lookahead       = std::clamp(0.5 * R, 0.1, 0.5);
        const double ahead           = angle + sign * lookahead / R;
        const Eigen::Vector2d target = centre + R * Eigen::Vector2d(std::cos(ahead), std::sin(ahead));
        const Eigen::Vector2d to     = target - b;
        dribble(rot(frame_yaw, FIG8_SPEED * to / std::max(to.norm(), 1e-6)));
    }

    void DribbleBench::summary() {
        log<INFO>("BENCH SUMMARY ======================================");
        for (double s : cfg.speeds) {
            int n = 0, fell_n = 0, lost_n = 0, ok = 0;
            double along = 0.0;
            for (const auto& r : results["straight"]) {
                if (r[0] != s) {
                    continue;
                }
                ++n;
                fell_n += int(r[1]);
                lost_n += int(r[2]);
                if (!r[1] && !r[2] && !std::isnan(r[3])) {
                    along += r[3];
                    ++ok;
                }
            }
            log<INFO>(fmt::format("BENCH SUMMARY straight {:.1f} m/s: ball speed {:.2f} m/s, fell {}/{}, lost {}/{}",
                                  s,
                                  ok ? along / ok : NAN,
                                  fell_n,
                                  n,
                                  lost_n,
                                  n));
        }
        for (double R : cfg.radii) {
            int n = 0, circles = 0, full = 0, fell_n = 0;
            for (const auto& r : results["figure8"]) {
                if (r[0] != R) {
                    continue;
                }
                ++n;
                circles += int(r[1]);
                full += r[1] >= 2 ? 1 : 0;
                fell_n += int(r[2]);
            }
            log<INFO>(fmt::format("BENCH SUMMARY figure8 {:.1f} m: circles {}/{} ({:.0f}%), full figures {}/{}, fell {}",
                                  R,
                                  circles,
                                  2 * n,
                                  n ? 100.0 * circles / (2 * n) : NAN,
                                  full,
                                  n,
                                  fell_n));
        }
        int n = 0, reached = 0, fell_n = 0, kept = 0, ok = 0;
        double along = 0.0;
        for (const auto& r : results["walk_to_dribble"]) {
            ++n;
            reached += int(r[0]);
            fell_n += int(r[1]);
            if (r[0] && !r[1] && !r[2]) {
                ++kept;
                along += r[3];
                ++ok;
            }
        }
        log<INFO>(fmt::format("BENCH SUMMARY walk->dribble: reached {}/{}, fell {}, kept {}, ball speed {:.2f} m/s",
                              reached,
                              n,
                              fell_n,
                              kept,
                              ok ? along / ok : NAN));
        log<INFO>("BENCH DONE");
    }

}  // namespace module::purpose
