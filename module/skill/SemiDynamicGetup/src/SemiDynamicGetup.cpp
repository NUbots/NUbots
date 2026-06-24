#include "SemiDynamicGetup.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/SemiDynamicGetup.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/skill/Script.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::skill::Walk;
    using SemiDynamicGetupTask = message::skill::SemiDynamicGetup;
    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::skill::Frame;
    using utility::support::Expression;

    // -------------------------------------------------------------------------
    // Support polygon helpers
    // -------------------------------------------------------------------------

    /// 2-D convex hull (Andrew's monotone chain, CCW output).
    static std::vector<Eigen::Vector2d> convex_hull_2d(std::vector<Eigen::Vector2d> pts) {
        const int n = static_cast<int>(pts.size());
        if (n < 2)
            return pts;
        std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
        });

        auto cross = [](const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
            return (A - O).x() * (B - O).y() - (A - O).y() * (B - O).x();
        };

        std::vector<Eigen::Vector2d> hull;
        // Lower hull
        for (int i = 0; i < n; i++) {
            while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), pts[i]) <= 0.0)
                hull.pop_back();
            hull.push_back(pts[i]);
        }
        // Upper hull
        const int lower_size = static_cast<int>(hull.size()) + 1;
        for (int i = n - 2; i >= 0; i--) {
            while (static_cast<int>(hull.size()) >= lower_size
                   && cross(hull[hull.size() - 2], hull.back(), pts[i]) <= 0.0)
                hull.pop_back();
            hull.push_back(pts[i]);
        }
        hull.pop_back();
        return hull;
    }

    /// Minimum signed distance from p to the polygon edges (CCW vertex order).
    /// Positive = p is inside by that many metres; negative = p is outside.
    /// Useful as a tuning signal: it shows the margin by which the capture-point
    /// test passes or fails, not just the pass/fail result.
    static double signed_distance_in_polygon_2d(const Eigen::Vector2d& p,
                                                const std::vector<Eigen::Vector2d>& hull) {
        const int n = static_cast<int>(hull.size());
        if (n == 0)
            return -std::numeric_limits<double>::infinity();
        if (n == 1)
            return -(p - hull[0]).norm();
        double min_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; i++) {
            const Eigen::Vector2d& a = hull[i];
            const Eigen::Vector2d& b = hull[(i + 1) % n];
            // Inward-pointing unit normal for a CCW polygon edge
            Eigen::Vector2d inward(-(b.y() - a.y()), b.x() - a.x());
            const double len = inward.norm();
            if (len < 1e-9)
                continue;
            inward /= len;
            // Signed distance from edge to p (positive = inside)
            min_dist = std::min(min_dist, inward.dot(p - a));
        }
        return min_dist;
    }

    /// Arithmetic centroid of a polygon's vertices.
    static Eigen::Vector2d polygon_centroid_2d(const std::vector<Eigen::Vector2d>& hull) {
        Eigen::Vector2d c = Eigen::Vector2d::Zero();
        for (const auto& v : hull)
            c += v;
        return hull.empty() ? c : c / static_cast<double>(hull.size());
    }

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

            cfg.getup_back       = config["scripts"]["getup_back"].as<std::vector<std::string>>();
            cfg.getup_stand      = config["scripts"]["getup_stand"].as<std::vector<std::string>>();
            cfg.rise_frame_count = config["rise_frame_count"].as<int>();
            cfg.pitch_reference  = config["pitch_reference"].as<double>();
            cfg.Kp               = config["Kp"].as<double>();
            cfg.max_correction   = config["max_correction"].as<double>();

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

            cfg.foot_toe_length  = config["foot_toe_length"].as<double>();
            cfg.foot_heel_length = config["foot_heel_length"].as<double>();
            cfg.foot_width       = config["foot_width"].as<double>();

            cfg.max_stabilisation_time = config["max_stabilisation_time"].as<double>();
            cfg.cp_margin              = config["cp_margin"].as<double>();
            cfg.cp_velocity_gain       = config["cp_velocity_gain"].as<double>();
            cfg.max_step_velocity      = config["max_step_velocity"].as<double>();
            cfg.walk_command_rate      = config["walk_command_rate"].as<double>();

            // Load all back-getup frames and split into early + rise phases
            std::vector<Frame> all_frames;
            for (const auto& name : cfg.getup_back) {
                auto parsed = utility::skill::load(name).as<std::vector<Frame>>();
                all_frames.insert(all_frames.end(), parsed.begin(), parsed.end());
            }

            // Clamp to [1, total] so a bad config value can't index out of bounds or
            // produce an empty rise sequence (which would never signal Done)
            const int total  = static_cast<int>(all_frames.size());
            const int n_rise = total > 0 ? std::clamp(cfg.rise_frame_count, 1, total) : 0;
            const int split  = total - n_rise;

            early_frames.assign(all_frames.begin(), all_frames.begin() + split);
            rise_frames.assign(all_frames.begin() + split, all_frames.end());

            log<INFO>("Loaded",
                      all_frames.size(),
                      "back-getup frames:",
                      early_frames.size(),
                      "early,",
                      rise_frames.size(),
                      "rise");
        });

        // Trigger<Sensors> makes this handler fire on every sensor update, enabling:
        //   - Mid-script fall detection (abort the BodySequence immediately if falling)
        //   - Per-tick capture-point tracking in the STABILISE phase
        on<Provide<SemiDynamicGetupTask>, Needs<BodySequence>, Trigger<Sensors>>().then(
            [this](const RunReason& run_reason, const Uses<BodySequence>& body, const Sensors& sensors) {
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
                auto state_str = [](State s) {
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
                    acc_mag_value   = smooth(acc_mag_value, std::abs(a.norm()), cfg.acc_mag.smoothing);
                    acc_angle_value = smooth(acc_angle_value,
                                             std::acos(std::min(1.0, std::abs(a.normalized().z())) - cfg.acc_angle.mean),
                                             cfg.acc_angle.smoothing);

                    // Classify each signal. Note acc_mag is inverted: it drops toward
                    // free-fall, so a larger value is more stable.
                    const State gyro_state = gyro_mag_value < cfg.gyro_mag.unstable    ? State::STABLE
                                             : gyro_mag_value < cfg.gyro_mag.falling   ? State::UNSTABLE
                                                                                       : State::FALLING;
                    const State acc_state = acc_mag_value > cfg.acc_mag.unstable    ? State::STABLE
                                            : acc_mag_value > cfg.acc_mag.falling   ? State::UNSTABLE
                                                                                    : State::FALLING;
                    const State angle_state = acc_angle_value < cfg.acc_angle.unstable    ? State::STABLE
                                              : acc_angle_value < cfg.acc_angle.falling   ? State::UNSTABLE
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
                    log<DEBUG>("fall check: gyro_mag=",
                               gyro_mag_value,
                               state_str(gyro_state),
                               "acc_mag=",
                               acc_mag_value,
                               state_str(acc_state),
                               "acc_angle=",
                               acc_angle_value,
                               state_str(angle_state),
                               "-> falling=",
                               falling);
                    return falling;
                };

                // After a fall, restart the getup only if the robot is still on its back // TODO
                auto restart_or_escalate = [&]() {
                    // Reset the falling filters so the next attempt starts from a clean slate.
                    gyro_mag_value = acc_mag_value = acc_angle_value = 0.0;
                    const Eigen::Isometry3d Hwt(sensors.Htw.inverse());
                    const Eigen::Vector3d uXTw = Hwt.rotation().col(0);
                    // Same cube-face test as GetUp: on the back when torso x is mostly world z
                    if (uXTw.z() >= uXTw.x() && uXTw.z() >= uXTw.y()) {
                        log<WARN>("Restarting getup from the early phase");
                        internal_phase = InternalPhase::EARLY;
                        emit<Task>(make_sequence(early_frames));
                    }
                    else {
                        log<WARN>("No longer on back, escalating so the getup side is re-classified");
                        emit<Task>(std::make_unique<Done>());
                    }
                };

                // Compute support polygon from both feet and the current capture point.
                // Returns {capture_point_world_xy, support_polygon_hull}.
                auto compute_cp_and_polygon = [&]() -> std::pair<Eigen::Vector2d, std::vector<Eigen::Vector2d>> {
                    Eigen::Isometry3d Hwt(sensors.Htw.inverse());

                    // CoM position in world frame
                    const Eigen::Vector3d com_world = Hwt.translation() + Hwt.rotation() * sensors.rMTt.cast<double>();

                    // LIPM natural frequency: ω = sqrt(g / h_com)
                    const double h_com = std::max(com_world.z(), 0.05);
                    const double omega = std::sqrt(9.81 / h_com);

                    // Capture point (Hof 2008): CP = CoM_xy + CoM_vel_xy / ω
                    // vTw is the torso translational velocity in world frame — a good
                    // approximation for CoM velocity when the robot is nearly stationary.
                    const Eigen::Vector2d cp = com_world.head<2>() + sensors.vTw.cast<double>().head<2>() / omega;

                    log<DEBUG>("CP calc: com_world=",
                               com_world.transpose(),
                               "h_com=",
                               h_com,
                               "omega=",
                               omega,
                               "vTw=",
                               sensors.vTw.transpose(),
                               "cp_xy=",
                               cp.transpose());

                    // Support polygon: convex hull of the four corners of each foot.
                    // Foot frame: x-forward, y-left, origin at ankle joint centre.
                    std::vector<Eigen::Vector2d> corners;
                    corners.reserve(8);
                    for (const auto& foot : sensors.feet) {
                        // TODO: If Hwf origin is not at the ankle joint centre, adjust
                        //       foot_toe_length / foot_heel_length offsets accordingly.
                        //       Check against the KinematicsConfiguration.yaml values.
                        const Eigen::Isometry3d Hwf(foot.Hwf);
                        // Log foot origin so the support polygon geometry can be sanity-checked
                        // against the real stance (and the ankle-frame assumption above verified).
                        log<DEBUG>("CP calc: foot origin (world)=", Hwf.translation().transpose());
                        for (double fx : {cfg.foot_toe_length, -cfg.foot_heel_length}) {
                            for (double fy : {cfg.foot_width / 2.0, -cfg.foot_width / 2.0}) {
                                const Eigen::Vector3d c = Hwf * Eigen::Vector3d(fx, fy, 0.0);
                                corners.push_back(c.head<2>());
                            }
                        }
                    }
                    return {cp, convex_hull_2d(std::move(corners))};
                };

                // Compute a corrective Walk velocity (robot frame) that steps toward the
                // capture point.  Returns {velocity_target, is_stable}.
                auto stabilisation_command = [&]() -> std::pair<Eigen::Vector3d, bool> {
                    auto [cp, hull] = compute_cp_and_polygon();

                    // Signed distance is the tuning signal for cp_margin: positive means the
                    // CP is inside by that many metres, negative means it is outside.
                    const double signed_dist     = signed_distance_in_polygon_2d(cp, hull);
                    const Eigen::Vector2d centroid = polygon_centroid_2d(hull);
                    const bool stable            = signed_dist >= cfg.cp_margin;
                    log<DEBUG>("Stabilise: hull_pts=",
                               hull.size(),
                               "centroid=",
                               centroid.transpose(),
                               "signed_dist=",
                               signed_dist,
                               "m (margin",
                               cfg.cp_margin,
                               ") stable=",
                               stable);

                    if (stable) {
                        return {Eigen::Vector3d::Zero(), true};
                    }

                    // Step toward the capture point.
                    // cp_err_world points from the support polygon centroid to the CP;
                    // the robot needs to move in that direction.
                    const Eigen::Vector2d cp_err_world = cp - centroid;

                    // Transform into the robot (torso) frame so the Walk task can use it.
                    // Hwt maps torso→world, so its transpose maps world→torso.
                    Eigen::Isometry3d Hwt(sensors.Htw.inverse());
                    const Eigen::Vector2d cp_err_robot =
                        Hwt.rotation().topLeftCorner<2, 2>().transpose() * cp_err_world;

                    Eigen::Vector2d vel = cfg.cp_velocity_gain * cp_err_robot;
                    // Clamp magnitude, preserve direction
                    const double speed = vel.norm();
                    if (speed > cfg.max_step_velocity)
                        vel *= cfg.max_step_velocity / speed;

                    log<DEBUG>("Stabilise: cp_err_world=",
                               cp_err_world.transpose(),
                               "cp_err_robot=",
                               cp_err_robot.transpose(),
                               "raw_speed=",
                               cfg.cp_velocity_gain * cp_err_robot.norm(),
                               "clamped=",
                               speed > cfg.max_step_velocity,
                               "-> vx=",
                               vel.x(),
                               "vy=",
                               vel.y());

                    return {Eigen::Vector3d(vel.x(), vel.y(), 0.0), false};
                };

                // -----------------------------------------------------------------
                // NEW_TASK — reset and start the early phase
                // -----------------------------------------------------------------
                if (run_reason == RunReason::NEW_TASK) {
                    log<INFO>("SemiDynamicGetup starting");
                    gyro_mag_value = acc_mag_value = acc_angle_value = 0.0;
                    internal_phase = InternalPhase::EARLY;
                    emit<Task>(make_sequence(early_frames));
                }

                // -----------------------------------------------------------------
                // SUBTASK_DONE — phase transitions (BodySequence completed)
                // -----------------------------------------------------------------
                else if (run_reason == RunReason::SUBTASK_DONE && body.done) {
                    const double tilt = tilt_from_vertical();
                    switch (internal_phase) {

                        case InternalPhase::EARLY: {
                            // One-shot hip-pitch correction based on current signed forward pitch.
                            const double pitch = forward_pitch();
                            const double error = pitch - cfg.pitch_reference;
                            const double hip_correction =
                                std::clamp(cfg.Kp * error, -cfg.max_correction, cfg.max_correction);
                            log<INFO>("Rise phase: pitch=",
                                      pitch,
                                      "rad  reference=",
                                      cfg.pitch_reference,
                                      "rad  error=",
                                      error,
                                      "rad  hip_correction=",
                                      hip_correction,
                                      "rad (clamped=",
                                      std::abs(cfg.Kp * error) > cfg.max_correction,
                                      ", tilt=",
                                      tilt,
                                      "rad)");
                            internal_phase = InternalPhase::RISE;
                            emit<Task>(make_sequence(rise_frames, hip_correction));
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
                                log<INFO>("Stand complete (tilt=", tilt, "rad), entering capture-point stabilisation");
                                internal_phase      = InternalPhase::STABILISE;
                                stabilisation_start = NUClear::clock::now();
                                // The walk engine does not update its generator while Stability
                                // is FALLEN, so mark the robot standing before issuing any Walk
                                // commands. Safe: FallRecovery is not gated on Stability and
                                // GetUpPlanner re-triggers on tilt, so a later fall is still caught.
                                emit(std::make_unique<Stability>(Stability::STANDING));
                                // Evaluate immediately — may already be stable.
                                auto [vel, stable] = stabilisation_command();
                                if (stable) {
                                    log<INFO>("Already stable on entry, done");
                                    emit<Task>(std::make_unique<Done>());
                                }
                                else {
                                    log<INFO>("CP outside polygon, corrective walk: vx=", vel.x(), " vy=", vel.y());
                                    last_walk_command = NUClear::clock::now();
                                    emit<Task>(std::make_unique<Walk>(vel));
                                }
                            }
                            break;

                        case InternalPhase::STABILISE:
                            // Walk does not self-complete, so this fires only if Walk
                            // emits Done unexpectedly. Treat it as stable and finish.
                            log<INFO>("Walk subtask finished unexpectedly, declaring done");
                            emit<Task>(std::make_unique<Done>());
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

                    else if (internal_phase == InternalPhase::STABILISE) {
                        // Fall check
                        if (falling) {
                            log<WARN>("Fall during stabilisation");
                            restart_or_escalate();
                            return;
                        }

                        // Timeout guard
                        const double elapsed =
                            std::chrono::duration<double>(NUClear::clock::now() - stabilisation_start).count();
                        if (elapsed > cfg.max_stabilisation_time) {
                            log<WARN>("Stabilisation timeout after ", elapsed, " s, declaring done");
                            emit<Task>(std::make_unique<Done>());
                            return;
                        }

                        // Capture-point check and corrective walk
                        auto [vel, stable] = stabilisation_command();
                        if (stable) {
                            log<INFO>("Capture point inside support polygon — stable, done");
                            emit<Task>(std::make_unique<Done>());
                        }
                        else {
                            // Rate-limit Walk task re-emission; Continue keeps the previous
                            // command active in the meantime.
                            const auto now = NUClear::clock::now();
                            if (std::chrono::duration<double>(now - last_walk_command).count()
                                >= 1.0 / cfg.walk_command_rate) {
                                last_walk_command = now;
                                emit<Task>(std::make_unique<Walk>(vel));
                            }
                            else {
                                emit<Task>(std::make_unique<Continue>());
                            }
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
                    emit<Task>(std::make_unique<Continue>());
                }
            });
    }

}  // namespace module::skill
