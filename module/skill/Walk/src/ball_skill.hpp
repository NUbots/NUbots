/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
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
#ifndef MODULE_SKILL_WALK_BALL_SKILL_HPP
#define MODULE_SKILL_WALK_BALL_SKILL_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

/// The observation, command and hand-over rules of the mjlab K1 ball-skill policies (mjlab
/// policies/CONTROLLER_INTERFACE.md), kept free of NUClear so they can be unit tested.
namespace module::skill::ball_skill {

    /// All K1 joints, JointIndexK1 order, which is also the policies' order
    inline constexpr std::size_t JOINT_COUNT = 22;
    /// Every term is observed over the last three control steps, oldest first
    inline constexpr std::size_t HISTORY = 3;
    /// Ball velocity command (vx, vy)
    inline constexpr std::size_t COMMAND_DIM = 2;
    /// Ball in the head camera (x, y, z, visible)
    inline constexpr std::size_t BALL_DIM = 4;
    /// One step of the observation
    inline constexpr std::size_t FRAME_DIM = 3 + 3 + 3 * JOINT_COUNT + COMMAND_DIM + BALL_DIM;
    /// The ONNX input, 234 numbers
    inline constexpr std::size_t INPUT_SIZE = HISTORY * FRAME_DIM;

    /// The ball observation before the first sighting
    inline constexpr std::array<float, BALL_DIM> NO_BALL = {0.5f, 0.0f, -0.8f, 0.0f};

    /// One control step of the seven observation terms
    struct Frame {
        std::array<float, 3> gyro{};
        std::array<float, 3> gravity{};
        std::array<float, JOINT_COUNT> q{};  // joint angle minus the ball skills' default pose
        std::array<float, JOINT_COUNT> dq{};
        std::array<float, JOINT_COUNT> action{};  // previous raw policy output
        std::array<float, COMMAND_DIM> command{};
        std::array<float, BALL_DIM> ball{};
    };

    /// The observation history, oldest step first
    struct History {
        std::array<Frame, HISTORY> frames{};
        bool primed = false;

        /// Adds the newest step. The first step after a reset fills the whole history, as in training.
        void push(const Frame& frame) {
            if (!primed) {
                frames.fill(frame);
                primed = true;
                return;
            }
            std::rotate(frames.begin(), frames.begin() + 1, frames.end());
            frames.back() = frame;
        }

        void reset() {
            primed = false;
        }

        /// The ONNX input: term by term, each term as its three steps oldest first
        [[nodiscard]] std::vector<float> input() const {
            std::vector<float> obs;
            obs.reserve(INPUT_SIZE);
            const auto put = [&](auto member) {
                for (const Frame& step : frames) {
                    obs.insert(obs.end(), (step.*member).begin(), (step.*member).end());
                }
            };
            put(&Frame::gyro);
            put(&Frame::gravity);
            put(&Frame::q);
            put(&Frame::dq);
            put(&Frame::action);
            put(&Frame::command);
            put(&Frame::ball);
            return obs;
        }
    };

    /// The head camera the policies' ball observation models: the head pitch link frame (Head_2, x forward,
    /// y left, z up) with its origin moved by `offset`, cut to a symmetric field of view and a range
    struct Camera {
        /// NUbots camera {c} to head pitch link {p}, as Sensors' Hpc
        Eigen::Isometry3d Hpc = Eigen::Isometry3d::Identity();
        /// Camera position in {p}, as trained
        Eigen::Vector3d offset = Eigen::Vector3d(0.06, 0.0, 0.07);
        double hfov            = 106.2 * M_PI / 180.0;
        double vfov            = 95.0 * M_PI / 180.0;
        double range           = 9.0;
    };

    /// A point in the world {w} as the policies' camera sees it
    [[nodiscard]] inline Eigen::Vector3d to_policy_camera(const Eigen::Vector3d& rBWw,
                                                          const Eigen::Isometry3d& Hcw,
                                                          const Camera& camera) {
        const Eigen::Vector3d rBCc = Hcw * rBWw;
        const Eigen::Vector3d rBPp = camera.Hpc * rBCc;
        return rBPp - camera.offset;
    }

    /// Whether a point in the policies' camera frame is in its field of view and range
    [[nodiscard]] inline bool in_view(const Eigen::Vector3d& r, const Camera& camera) {
        if (r.x() <= 0.0) {
            return false;
        }
        const double azimuth   = std::atan2(r.y(), r.x());
        const double elevation = std::atan2(r.z(), r.x());
        return std::abs(azimuth) < camera.hfov / 2.0 && std::abs(elevation) < camera.vfov / 2.0
               && r.norm() < camera.range;
    }

    /// The ball centre in the world {w} from a detection's unit ray uBCc in the camera {c}: where the ray
    /// crosses the plane at the ball's radius above the ground. Empty if the ray does not reach that plane.
    [[nodiscard]] inline std::optional<Eigen::Vector3d> ray_to_ball_centre(const Eigen::Vector3d& uBCc,
                                                                           const Eigen::Isometry3d& Hcw,
                                                                           double ball_radius) {
        const Eigen::Isometry3d Hwc = Hcw.inverse();
        const Eigen::Vector3d uBCw  = Hwc.linear() * uBCc.normalized();
        const Eigen::Vector3d rCWw  = Hwc.translation();
        if (uBCw.z() > -1e-6) {
            return std::nullopt;
        }
        const double t = (ball_radius - rCWw.z()) / uBCw.z();
        if (t <= 0.0) {
            return std::nullopt;
        }
        return rCWw + t * uBCw;
    }

    /// The trained command ranges, with the hard limits the controller interface asks for
    struct CommandLimits {
        double dribble_min = 0.4;
        double dribble_max = 1.8;
        double kick_min    = 2.0;
        double kick_max    = 7.0;
        /// Kicks within this angle of the heading, narrowing to shot_angle above shot_speed
        double kick_angle      = 135.0 * M_PI / 180.0;
        double shot_speed      = 4.5;
        double shot_angle      = 45.0 * M_PI / 180.0;
        double first_touch_min = 0.5;
        double first_touch_max = 1.2;
        double first_pass_min  = 2.0;
        double first_pass_max  = 4.0;
    };

    /// The skills, in BallSkill::Type's order
    enum class Kind { RECEIVE = 0, DRIBBLE = 1, KICK = 2, KICK_AIR = 3, FIRST_TOUCH = 4, FIRST_PASS = 5 };
    inline constexpr std::size_t KIND_COUNT = 6;

    /// Clamps a ball velocity command (robot yaw frame) to what the skill was trained on. A dribble command
    /// under half the minimum speed is a hold, (0, 0). Receive is always (0, 0).
    [[nodiscard]] inline Eigen::Vector2d clamp_command(Kind kind,
                                                       const Eigen::Vector2d& command,
                                                       const CommandLimits& limits) {
        const double speed = command.norm();
        double angle       = std::atan2(command.y(), command.x());
        const auto polar   = [](double s, double a) { return Eigen::Vector2d(s * std::cos(a), s * std::sin(a)); };
        switch (kind) {
            case Kind::RECEIVE: return Eigen::Vector2d::Zero();
            case Kind::DRIBBLE:
                if (speed < limits.dribble_min / 2.0) {
                    return Eigen::Vector2d::Zero();
                }
                return polar(std::clamp(speed, limits.dribble_min, limits.dribble_max), angle);
            case Kind::KICK:
            case Kind::KICK_AIR: {
                if (speed < 1e-6) {
                    angle = 0.0;  // no direction given: straight ahead
                }
                const double s     = std::clamp(speed, limits.kick_min, limits.kick_max);
                const double limit = s > limits.shot_speed ? limits.shot_angle : limits.kick_angle;
                return polar(s, std::clamp(angle, -limit, limit));
            }
            case Kind::FIRST_TOUCH:
                if (speed < 1e-6) {
                    return Eigen::Vector2d::Zero();
                }
                return polar(std::clamp(speed, limits.first_touch_min, limits.first_touch_max), angle);
            case Kind::FIRST_PASS:
                if (speed < 1e-6) {
                    return Eigen::Vector2d::Zero();
                }
                return polar(std::clamp(speed, limits.first_pass_min, limits.first_pass_max), angle);
        }
        return Eigen::Vector2d::Zero();
    }

    /// A joint target expressed as a policy's raw action: (target - default) / scale. Used at a hand-over
    /// between the walk and a ball skill, so the new policy's "previous action" is the target last sent.
    [[nodiscard]] inline float to_action(double target, double default_pose, double scale) {
        return scale != 0.0 ? static_cast<float>((target - default_pose) / scale) : 0.0f;
    }

    /// Moves `target` towards `goal` by at most `max_step`
    [[nodiscard]] inline double rate_limit(double target, double goal, double max_step) {
        return target + std::clamp(goal - target, -max_step, max_step);
    }

}  // namespace module::skill::ball_skill

#endif  // MODULE_SKILL_WALK_BALL_SKILL_HPP
