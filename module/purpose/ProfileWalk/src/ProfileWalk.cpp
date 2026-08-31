/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ProfileWalk.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;

    using utility::nusight::graph;
    using utility::support::Expression;

    ProfileWalk::ProfileWalk(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration, Sync<ProfileWalk>>("ProfileWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ProfileWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.start_delay           = config["start_delay"].as<double>();
            cfg.ramp_time             = config["ramp_time"].as<double>();
            cfg.hold_time             = config["hold_time"].as<double>();
            cfg.rest_time             = config["rest_time"].as<double>();
            cfg.recovery_time         = config["recovery_time"].as<double>();
            cfg.rest_between_segments = config["rest_between_segments"].as<bool>();
            cfg.loop                  = config["loop"].as<bool>();
            cfg.neck_yaw              = config["head"]["yaw"].as<Expression>();
            cfg.head_pitch            = config["head"]["pitch"].as<Expression>();

            // Read the command profile
            cfg.profile.clear();
            for (const auto& stage_config : config["profile"].config) {
                Stage stage{};
                stage.name = stage_config["name"].as<std::string>("");
                for (const auto& segment_config : stage_config["segments"]) {
                    Segment segment{};
                    segment.name     = segment_config["name"].as<std::string>("");
                    segment.velocity = segment_config["velocity"].as<Expression>();
                    stage.segments.emplace_back(segment);
                }
                cfg.profile.emplace_back(stage);
            }

            // Any change to the profile starts it again from the beginning
            reset_profile();
            log<INFO>(fmt::format("Loaded a walk command profile with {} stages", cfg.profile.size()));
        });

        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));

            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 5);
        });

        // Pause the profile while the robot is on the ground, and resume it once it has stood back up
        on<Trigger<Stability>, Sync<ProfileWalk>>().then([this](const Stability& stability) {
            // Falling or fallen, hold a zero command until we are back on our feet
            if (stability <= Stability::FALLING) {
                if (phase != ProfilePhase::FALLEN && phase != ProfilePhase::FINISHED) {
                    log<WARN>(phase == ProfilePhase::WAITING
                                  ? std::string("Fell before the profile started, waiting to get back up")
                                  : fmt::format("Fell during {}, pausing the profile", current_name()));
                    phase_before_fall = phase;
                    velocity          = Eigen::Vector3d::Zero();
                    set_phase(ProfilePhase::FALLEN);
                }
                return;
            }

            // Back on our feet after a fall
            if (phase == ProfilePhase::FALLEN && (stability == Stability::STANDING || stability == Stability::STATIC)) {
                // If we fell before the profile started, just start the delay again
                if (phase_before_fall == ProfilePhase::WAITING) {
                    log<INFO>("Stood back up, restarting the start delay");
                    reset_profile();
                    return;
                }

                // Otherwise abandon the rest of the stage we fell in and pick up from the next one, so a
                // stage that puts the robot on the ground is never reattempted
                if (!profile_complete()) {
                    log<WARN>(fmt::format("Stood back up, abandoning the rest of stage {}/{} '{}'",
                                          stage_index + 1,
                                          cfg.profile.size(),
                                          cfg.profile[stage_index].name));
                }
                skip_stage();

                // The fall itself replaces the usual rest between stages, so this only settles for the
                // (by default zero) recovery time before the next stage starts. skip_stage() has already
                // moved us on, so the rest must not advance again when it ends.
                velocity            = Eigen::Vector3d::Zero();
                rest_after_recovery = true;
                set_phase(ProfilePhase::REST);
            }
        });

        // Main loop - steps the profile and emits the walk command at a fixed rate
        on<Every<UPDATE_RATE, Per<std::chrono::seconds>>, Sync<ProfileWalk>, Single>().then([this] {
            // Nothing to do until the profile has been configured
            if (cfg.profile.empty()) {
                return;
            }

            update_profile();

            // Hold the head still for the whole profile so it doesn't add any disturbance
            Eigen::Vector3d uPCt = (Eigen::AngleAxisd(cfg.neck_yaw, Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(-cfg.head_pitch, Eigen::Vector3d::UnitY()))
                                       .toRotationMatrix()
                                   * Eigen::Vector3d::UnitX();
            emit<Task>(std::make_unique<Look>(uPCt, false));

            emit<Task>(std::make_unique<Walk>(velocity), 2);

            emit(graph("Profile walk command (vx, vy, wz)", velocity.x(), velocity.y(), velocity.z()));
            emit(graph("Profile walk phase", int(phase)));
            emit(graph("Profile walk stage", int(stage_index)));
            emit(graph("Profile walk segment", int(segment_index)));
        });
    }

    double ProfileWalk::phase_elapsed() const {
        return std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now() - phase_start_time)
            .count();
    }

    void ProfileWalk::set_phase(const ProfilePhase& new_phase) {
        phase            = new_phase;
        phase_start_time = NUClear::clock::now();
    }

    void ProfileWalk::reset_profile() {
        stage_index         = 0;
        segment_index       = 0;
        velocity            = Eigen::Vector3d::Zero();
        ramp_from           = Eigen::Vector3d::Zero();
        ramp_to             = Eigen::Vector3d::Zero();
        rest_after_recovery = false;
        phase_before_fall   = ProfilePhase::WAITING;
        // Skip over any stages at the start of the profile that have no segments
        while (!profile_complete() && cfg.profile[stage_index].segments.empty()) {
            stage_index++;
        }
        set_phase(ProfilePhase::WAITING);
    }

    bool ProfileWalk::profile_complete() const {
        return stage_index >= cfg.profile.size();
    }

    const ProfileWalk::Segment& ProfileWalk::current_segment() const {
        return cfg.profile[stage_index].segments[segment_index];
    }

    std::string ProfileWalk::current_name() const {
        if (profile_complete()) {
            return "the end of the profile";
        }
        const Stage& stage = cfg.profile[stage_index];
        return fmt::format("stage {}/{} '{}' segment {}/{} '{}'",
                           stage_index + 1,
                           cfg.profile.size(),
                           stage.name,
                           segment_index + 1,
                           stage.segments.size(),
                           current_segment().name);
    }

    void ProfileWalk::next_segment() {
        segment_index++;
        // Roll over into the next stage when the current one is done, skipping any empty stages
        while (!profile_complete() && segment_index >= cfg.profile[stage_index].segments.size()) {
            stage_index++;
            segment_index = 0;
        }
    }

    void ProfileWalk::skip_stage() {
        stage_index++;
        segment_index = 0;
        // Skip any empty stages
        while (!profile_complete() && cfg.profile[stage_index].segments.empty()) {
            stage_index++;
        }
    }

    void ProfileWalk::start_segment(const Eigen::Vector3d& from) {
        ramp_from = from;
        ramp_to   = current_segment().velocity;
        set_phase(ProfilePhase::RAMP_UP);
        log<INFO>(fmt::format("Starting {} with velocity ({:.3f}, {:.3f}, {:.3f})",
                              current_name(),
                              ramp_to.x(),
                              ramp_to.y(),
                              ramp_to.z()));
    }

    void ProfileWalk::start_or_finish() {
        if (!profile_complete()) {
            start_segment(velocity);
            return;
        }

        if (cfg.loop) {
            log<INFO>("Profile finished, looping back to the first stage");
            stage_index   = 0;
            segment_index = 0;
            // Skip over any stages at the start of the profile that have no segments
            while (!profile_complete() && cfg.profile[stage_index].segments.empty()) {
                stage_index++;
            }
            if (profile_complete()) {
                log<WARN>("The profile has no segments to run, standing still");
                velocity = Eigen::Vector3d::Zero();
                set_phase(ProfilePhase::FINISHED);
                return;
            }
            start_segment(velocity);
            return;
        }

        log<INFO>("Profile finished, standing still");
        velocity = Eigen::Vector3d::Zero();
        set_phase(ProfilePhase::FINISHED);
    }

    void ProfileWalk::update_profile() {
        // Standing still with nothing left to do, or waiting for the robot to get back up
        if (phase == ProfilePhase::FINISHED || phase == ProfilePhase::FALLEN) {
            velocity = Eigen::Vector3d::Zero();
            return;
        }

        const double t = phase_elapsed();

        switch (phase) {
            case ProfilePhase::WAITING: {
                velocity = Eigen::Vector3d::Zero();
                if (t >= cfg.start_delay) {
                    start_or_finish();
                }
            } break;

            case ProfilePhase::RAMP_UP:
            case ProfilePhase::RAMP_DOWN: {
                // Linearly interpolate between the two ends of the ramp
                const double alpha = cfg.ramp_time > 0.0 ? std::clamp(t / cfg.ramp_time, 0.0, 1.0) : 1.0;
                velocity           = ramp_from + alpha * (ramp_to - ramp_from);

                if (t >= cfg.ramp_time) {
                    velocity = ramp_to;
                    // Hold the new velocity, or rest now that we are back at a stop
                    if (phase == ProfilePhase::RAMP_UP) {
                        set_phase(ProfilePhase::HOLD);
                    }
                    else {
                        rest_after_recovery = false;
                        set_phase(ProfilePhase::REST);
                    }
                }
            } break;

            case ProfilePhase::HOLD: {
                velocity = ramp_to;
                if (t >= cfg.hold_time) {
                    // Ramp straight into the next segment of this stage if we aren't resting between them
                    const bool more_in_stage = segment_index + 1 < cfg.profile[stage_index].segments.size();
                    if (!cfg.rest_between_segments && more_in_stage) {
                        segment_index++;
                        start_segment(velocity);
                    }
                    else {
                        // Otherwise ramp back down to a stop before the rest period
                        ramp_from = velocity;
                        ramp_to   = Eigen::Vector3d::Zero();
                        set_phase(ProfilePhase::RAMP_DOWN);
                    }
                }
            } break;

            case ProfilePhase::REST: {
                velocity = Eigen::Vector3d::Zero();
                // A rest after getting up uses the recovery time, since the fall itself was the pause
                const double rest_time = rest_after_recovery ? cfg.recovery_time : cfg.rest_time;
                if (t >= rest_time) {
                    // Getting up already skipped past the stage we fell in, otherwise move on normally
                    if (!rest_after_recovery) {
                        next_segment();
                    }
                    start_or_finish();
                }
            } break;

            default: break;
        }
    }

}  // namespace module::purpose
