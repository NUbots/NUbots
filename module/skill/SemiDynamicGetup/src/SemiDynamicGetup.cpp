#include "SemiDynamicGetup.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/SemiDynamicGetup.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/skill/Script.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using SemiDynamicGetupTask = message::skill::SemiDynamicGetup;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::skill::Frame;
    using utility::support::Expression;

    // -------------------------------------------------------------------------
    // make_sequence
    // -------------------------------------------------------------------------

    std::unique_ptr<BodySequence> SemiDynamicGetup::make_sequence(const std::vector<Frame>& frames,
                                                                  double hip_correction) const {
        auto msg  = std::make_unique<BodySequence>();
        auto time = NUClear::clock::now();
        for (const auto& frame : frames) {
            time += std::chrono::duration_cast<NUClear::clock::time_point::duration>(frame.duration);
            std::map<uint32_t, ServoCommand> servos;
            for (auto target : frame.targets) {
                if (target.id == ServoID::R_HIP_PITCH || target.id == ServoID::L_HIP_PITCH) {
                    target.position += static_cast<float>(hip_correction);
                }
                servos[target.id] = ServoCommand(time,
                                                 target.position,
                                                 ServoState(target.gain, target.torque),
                                                 ServoCommand::DoneType::TIME);
            }
            msg->frames.emplace_back(servos);
        }
        return msg;
    }

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    SemiDynamicGetup::SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("SemiDynamicGetup.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Per-direction getup config (front / back)
            auto load_direction = [](const Configuration& c) {
                Config::DirectionConfig dc{};
                dc.scripts          = c["scripts"].as<std::vector<std::string>>();
                dc.rise_frame_count = c["rise_frame_count"].as<int>();
                dc.pitch_reference  = c["pitch_reference"].as<double>();
                dc.Kp               = c["Kp"].as<double>();
                dc.max_correction   = c["max_correction"].as<double>();
                return dc;
            };
            cfg.back        = load_direction(config["back"]);
            cfg.front       = load_direction(config["front"]);
            cfg.getup_stand = config["getup_stand"].as<std::vector<std::string>>();

            cfg.fall_recovery_tilt = config["fall_recovery_tilt"].as<double>();

            // Falling detector (ported from FallingRelaxPlanner)
            auto load_levels = [](const Configuration& c) {
                Config::Levels levels{};
                levels.mean      = c["mean"].as<Expression>();
                levels.unstable  = c["unstable"].as<Expression>();
                levels.falling   = c["falling"].as<Expression>();
                levels.smoothing = c["smoothing"].as<Expression>();
                return levels;
            };
            cfg.gyro_mag  = load_levels(config["gyroscope_magnitude"]);
            cfg.acc_mag   = load_levels(config["accelerometer_magnitude"]);
            cfg.acc_angle = load_levels(config["accelerometer_angle"]);

            // Load each direction's script(s) and split into early + rise phases.
            auto build_frames = [this](const std::string& label, const Config::DirectionConfig& dc) {
                std::vector<Frame> all_frames;
                for (const auto& name : dc.scripts) {
                    auto parsed = utility::skill::load(name).as<std::vector<Frame>>();
                    all_frames.insert(all_frames.end(), parsed.begin(), parsed.end());
                }

                // Clamp to [1, total] so a bad config value can't index out of bounds or
                // produce an empty rise sequence (which would never signal Done)
                const int total  = static_cast<int>(all_frames.size());
                const int n_rise = total > 0 ? std::clamp(dc.rise_frame_count, 1, total) : 0;
                const int split  = total - n_rise;

                GetupFrames gf{};
                gf.early.assign(all_frames.begin(), all_frames.begin() + split);
                gf.rise.assign(all_frames.begin() + split, all_frames.end());

                log<INFO>("Loaded",
                          all_frames.size(),
                          label.c_str(),
                          "getup frames:",
                          gf.early.size(),
                          "early,",
                          gf.rise.size(),
                          "rise");
                return gf;
            };
            back_frames  = build_frames("back", cfg.back);
            front_frames = build_frames("front", cfg.front);
        });

        // Trigger<Sensors> makes this handler fire on every sensor update, enabling
        // mid-script fall detection (abort the BodySequence immediately if falling).
        on<Provide<SemiDynamicGetupTask>, Needs<BodySequence>, Trigger<Sensors>>().then(
            [this](const SemiDynamicGetupTask& task,
                   const RunReason& run_reason,
                   const Uses<BodySequence>& body,
                   const Sensors& sensors) {
                // Latch which orientation we are getting up from. Read only on NEW_TASK;
                // it must persist across the phase-transition runs that follow.
                if (run_reason == RunReason::NEW_TASK) {
                    start_side =
                        task.direction == SemiDynamicGetupTask::Direction::FRONT ? StartSide::FRONT : StartSide::BACK;
                }
                // -----------------------------------------------------------------
                // Shared sensor helpers
                // -----------------------------------------------------------------

                // Tilt angle
                auto tilt_from_vertical = [&]() -> double {
                    const Eigen::Isometry3d Hwt(sensors.Htw.inverse());
                    const Eigen::Vector3d uZTw = Hwt.rotation().col(2);
                    return std::atan2(uZTw.head<2>().norm(), uZTw.z());
                };

                // Signed forward pitch of the torso (rad): 0 = upright, positive = leaning
                // forward, negative = leaning backward (lying flat on the back = -π/2).
                auto forward_pitch = [&]() -> double {
                    const Eigen::Isometry3d Hwt(sensors.Htw.inverse());
                    return std::atan2(-Hwt.rotation()(2, 0), Hwt.rotation()(2, 2));
                };

                // Dynamic fall detection, ported from FallingRelaxPlanner. Three
                // exponentially smoothed signals are each classified STABLE / UNSTABLE /
                // FALLING, and the robot is declared falling if any two of three (or the
                // accelerometer angle alone) read FALLING. Call once per sensor cycle so
                // the filters stay warm across all phases.
                // Only referenced by the (currently muted) per-tick fall-check log below; keep it
                // paired with that log so re-enabling the log just works.
                [[maybe_unused]] auto state_str = [](State s) {
                    return s == State::FALLING ? "FALLING" : s == State::UNSTABLE ? "UNSTABLE" : "STABLE";
                };
                auto fall_detected = [&]() -> bool {
                    const auto& a = sensors.accelerometer;
                    const auto& g = sensors.gyroscope;

                    // Update the exponential filters
                    gyro_mag_value =
                        smooth(gyro_mag_value,
                               std::abs(std::abs(g.x()) + std::abs(g.y()) + std::abs(g.z()) - cfg.gyro_mag.mean),
                               cfg.gyro_mag.smoothing);
                    acc_mag_value = smooth(acc_mag_value, std::abs(a.norm()), cfg.acc_mag.smoothing);
                    acc_angle_value =
                        smooth(acc_angle_value,
                               std::acos(std::min(1.0, std::abs(a.normalized().z())) - cfg.acc_angle.mean),
                               cfg.acc_angle.smoothing);

                    // Classify each signal. Note acc_mag is inverted: it drops toward
                    // free-fall, so a larger value is more stable.
                    const State gyro_state  = gyro_mag_value < cfg.gyro_mag.unstable  ? State::STABLE
                                              : gyro_mag_value < cfg.gyro_mag.falling ? State::UNSTABLE
                                                                                      : State::FALLING;
                    const State acc_state   = acc_mag_value > cfg.acc_mag.unstable  ? State::STABLE
                                              : acc_mag_value > cfg.acc_mag.falling ? State::UNSTABLE
                                                                                    : State::FALLING;
                    const State angle_state = acc_angle_value < cfg.acc_angle.unstable  ? State::STABLE
                                              : acc_angle_value < cfg.acc_angle.falling ? State::UNSTABLE
                                                                                        : State::FALLING;

                    const bool falling = (gyro_state == State::FALLING && acc_state == State::FALLING)
                                         || (gyro_state == State::FALLING && angle_state == State::FALLING)
                                         || (acc_state == State::FALLING && angle_state == State::FALLING)
                                         || (angle_state == State::FALLING);

                    // NUSight graphs + per-tick log for tuning the thresholds above.
                    emit(graph("SemiDynamicGetup falling signals (gyro, acc, angle deg)",
                               gyro_mag_value,
                               acc_mag_value,
                               acc_angle_value * 180.0 / 3.14159265358979));
                    emit(graph("SemiDynamicGetup falling", falling));
                    // log<DEBUG>("fall check: gyro_mag=",
                    //            gyro_mag_value,
                    //            state_str(gyro_state),
                    //            "acc_mag=",
                    //            acc_mag_value,
                    //            state_str(acc_state),
                    //            "acc_angle=",
                    //            acc_angle_value,
                    //            state_str(angle_state),
                    //            "-> falling=",
                    //            falling);
                    return falling;
                };

                // True if the robot is still lying in the orientation this getup started from.
                // Uses the same cube-face tests as GetUp: torso x mostly world +z = on back,
                // torso x mostly world -z = on front.
                auto in_start_orientation = [&]() -> bool {
                    const Eigen::Isometry3d Hwt(sensors.Htw.inverse());
                    const Eigen::Vector3d uXTw = Hwt.rotation().col(0);
                    if (start_side == StartSide::FRONT) {
                        return uXTw.z() <= uXTw.x() && uXTw.z() <= uXTw.y();
                    }
                    return uXTw.z() >= uXTw.x() && uXTw.z() >= uXTw.y();
                };

                // After a mid-getup fall: if the robot is still in its starting orientation,
                // restart this getup from the early phase; otherwise escalate (Done) so GetUp
                // re-classifies and dispatches the correct getup for the new orientation.
                auto restart_or_escalate = [&]() {
                    // Reset the falling filters so the next attempt starts from a clean slate.
                    gyro_mag_value = acc_mag_value = acc_angle_value = 0.0;
                    if (in_start_orientation()) {
                        log<WARN>("Restarting getup from the early phase");
                        internal_phase = InternalPhase::EARLY;
                        emit<Task>(make_sequence(active_frames().early));
                    }
                    else {
                        log<WARN>("No longer in the start orientation, escalating so the getup side is re-classified");
                        emit<Task>(std::make_unique<Done>());
                    }
                };

                auto reason_str = [](const RunReason& r) {
                    switch (r) {
                        case RunReason::NEW_TASK: return "NEW_TASK";
                        case RunReason::SUBTASK_DONE: return "SUBTASK_DONE";
                        case RunReason::OTHER_TRIGGER: return "OTHER_TRIGGER";
                        case RunReason::STARTED: return "STARTED";
                        case RunReason::STOPPED: return "STOPPED";
                        case RunReason::PUSHED: return "PUSHED";
                        default: return "UNKNOWN";
                    }
                };
                auto phase_str = [](InternalPhase p) {
                    return p == InternalPhase::EARLY ? "EARLY" : p == InternalPhase::RISE ? "RISE" : "STAND";
                };

                // Director-event trace: NEW_TASK / SUBTASK_DONE / PUSHED etc. are rare and each marks
                // real progress, so always log them. The high-rate OTHER_TRIGGER (per sensor tick)
                // runs are NOT logged here — they are summarised by the throttled heartbeat in the
                // OTHER_TRIGGER branch, so a freeze reads as the heartbeat stopping, not as spam.
                if (run_reason != RunReason::OTHER_TRIGGER) {
                    log<DEBUG>("SemiDynamicGetup: ",
                               reason_str(run_reason),
                               " phase=",
                               phase_str(internal_phase),
                               " body.done=",
                               body.done);
                }

                // -----------------------------------------------------------------
                // NEW_TASK — reset and start the early phase
                // -----------------------------------------------------------------
                if (run_reason == RunReason::NEW_TASK) {
                    log<INFO>("SemiDynamicGetup starting from", start_side == StartSide::FRONT ? "front" : "back");
                    gyro_mag_value = acc_mag_value = acc_angle_value = 0.0;
                    internal_phase                                   = InternalPhase::EARLY;
                    emit<Task>(make_sequence(active_frames().early));
                }

                // -----------------------------------------------------------------
                // SUBTASK_DONE — phase transitions (BodySequence completed)
                // -----------------------------------------------------------------
                else if (run_reason == RunReason::SUBTASK_DONE && body.done) {
                    const double tilt = tilt_from_vertical();
                    switch (internal_phase) {

                        case InternalPhase::EARLY: {
                            // One-shot hip-pitch correction based on current signed forward pitch,
                            // using the active direction's tuning.
                            const auto& dcfg   = active_cfg();
                            const double pitch = forward_pitch();
                            const double error = pitch - dcfg.pitch_reference;
                            const double hip_correction =
                                std::clamp(dcfg.Kp * error, -dcfg.max_correction, dcfg.max_correction);
                            log<INFO>("Rise phase: pitch=",
                                      pitch,
                                      "rad  reference=",
                                      dcfg.pitch_reference,
                                      "rad  error=",
                                      error,
                                      "rad  hip_correction=",
                                      hip_correction,
                                      "rad (clamped=",
                                      std::abs(dcfg.Kp * error) > dcfg.max_correction,
                                      ", tilt=",
                                      tilt,
                                      "rad)");
                            internal_phase = InternalPhase::RISE;
                            emit<Task>(make_sequence(active_frames().rise, hip_correction));
                            break;
                        }

                        case InternalPhase::RISE:
                            if (tilt > cfg.fall_recovery_tilt) {
                                log<WARN>("Robot still fallen after rise (tilt=", tilt, " rad)");
                                restart_or_escalate();
                            }
                            else {
                                log<INFO>("Rise complete (tilt=", tilt, "rad), standing up");
                                internal_phase = InternalPhase::STAND;
                                emit<Task>(utility::skill::load_script<BodySequence>(cfg.getup_stand));
                            }
                            break;

                        case InternalPhase::STAND:
                            if (tilt > cfg.fall_recovery_tilt) {
                                log<WARN>("Robot still fallen after stand (tilt=", tilt, " rad)");
                                restart_or_escalate();
                            }
                            else {
                                log<INFO>("Stand complete (tilt=", tilt, "rad), getup done");
                                emit<Task>(std::make_unique<Done>());
                            }
                            break;
                    }
                }

                // -----------------------------------------------------------------
                // OTHER_TRIGGER — sensor update during active motion
                // -----------------------------------------------------------------
                else if (run_reason == RunReason::OTHER_TRIGGER) {

                    // Update the fall-detection filters every sensor cycle (in all phases)
                    // so they stay warm; only the dynamic phases act on the result.
                    const bool falling = fall_detected();

                    // Throttled liveness heartbeat (~1 Hz). OTHER_TRIGGER fires at sensor rate; the
                    // heartbeat collapses that to one line a second carrying the phase, the tick count
                    // since the last beat (so a slowdown shows as the count dropping), and the falling
                    // flag. If the heartbeat STOPS, the Sensors stream or the clock has stalled, and
                    // its last line names the phase the getup froze in.
                    ++ticks_since_heartbeat;
                    const auto now = NUClear::clock::now();
                    if (now - last_heartbeat >= std::chrono::seconds(1)) {
                        log<DEBUG>("SemiDynamicGetup alive: phase=",
                                   phase_str(internal_phase),
                                   " ticks=",
                                   ticks_since_heartbeat,
                                   " falling=",
                                   falling);
                        last_heartbeat        = now;
                        ticks_since_heartbeat = 0;
                    }

                    if (internal_phase == InternalPhase::RISE || internal_phase == InternalPhase::STAND) {
                        // Mid-script fall detection: abort and recover immediately.
                        // This is the core of the semi-dynamic behaviour — the BodySequence
                        // is replaced rather than allowed to play out on the ground.
                        if (falling) {
                            log<WARN>("Fall detected mid-", internal_phase == InternalPhase::RISE ? "rise" : "stand");
                            restart_or_escalate();
                        }
                        else {
                            emit<Task>(std::make_unique<Continue>());
                        }
                    }

                    else {
                        emit<Task>(std::make_unique<Continue>());
                    }
                }

                // -----------------------------------------------------------------
                // Everything else (PUSHED, STOPPED, …)
                // -----------------------------------------------------------------
                else {
                    log<DEBUG>("SemiDynamicGetup: run reason ",
                               reason_str(run_reason),
                               " in phase ",
                               phase_str(internal_phase),
                               ", continue");
                    emit<Task>(std::make_unique<Continue>());
                }
            });
    }

}  // namespace module::skill
