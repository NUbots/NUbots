/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
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

#ifndef MODULES_MOTION_IKKICKCONTROLLERS_HPP
#define MODULES_MOTION_IKKICKCONTROLLERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/matrix/transform.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::motion {

    using utility::math::transform::interpolate;
    using utility::support::Expression;

    enum MotionStage { READY = 0, RUNNING = 1, STOPPING = 2, FINISHED = 3 };

    class SixDOFFrame {
        // enum InterpolationType {
        //  LINEAR = 0,
        //  SERVO = 1
        // TODO:
        // };
        // InterpolationType interpolation = LINEAR;
    public:
        Eigen::Isometry3d pose;
        float duration;
        SixDOFFrame() : pose(), duration(0.0f) {}
        SixDOFFrame(Eigen::Isometry3d pose_, float duration_) : pose(pose_), duration(duration_) {}
        SixDOFFrame(const YAML::Node& config) : SixDOFFrame() {
            duration                    = config["duration"].as<float>();
            Eigen::Vector3d pos         = config["pos"].as<Expression>();
            Eigen::Vector3d orientation = config["orientation"].as<Expression>();
            pose                        = Eigen::Isometry3d::Identity();
            pose.rotate(Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX()));
            pose.rotate(Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY()));
            pose.rotate(Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ()));
            pose.translation() = pos;
        };
        // TODO:
        // std::map<message::input::ServoID, float> jointGains;
    };

    class Animator {
    public:
        std::vector<SixDOFFrame> frames;
        int i = 0;
        Animator() : frames() {
            frames.push_back(SixDOFFrame{Eigen::Isometry3d::Identity(), 0});
        }
        Animator(const std::vector<SixDOFFrame>& frames_) : frames(frames_) {}
        int clampPrev(int k) const {
            return std::max(std::min(k, int(frames.size() - 2)), 0);
        }
        int clampCurrent(int k) const {
            return std::max(std::min(k, int(frames.size() - 1)), 0);
        }
        void next() {
            i = clampPrev(i + 1);
        }
        void reset() {
            i = 0;
        }
        const SixDOFFrame& currentFrame() const {
            return frames[clampCurrent(i + 1)];
        }
        const SixDOFFrame& previousFrame() const {
            return frames[i];
        }
        const SixDOFFrame& startFrame() const {
            return frames[0];
        }
        bool stable() {
            return i >= int(frames.size() - 2);
        }
    };

    class SixDOFFootController {
    protected:
        MotionStage stage = MotionStage::READY;
        bool stable       = false;

        // State variables
        utility::input::LimbID supportFoot;

        float forward_duration;
        float return_duration;
        Animator anim;
        float servo_angle_threshold = 0.1;

        Eigen::Vector3d ballPosition;
        Eigen::Vector3d goalPosition;

        NUClear::clock::time_point motionStartTime;

    public:
        SixDOFFootController()
            : supportFoot()
            , forward_duration(0.0f)
            , return_duration(0.0f)
            , anim()
            , ballPosition(Eigen::Vector3d::Zero())
            , goalPosition(Eigen::Vector3d::Zero())
            , motionStartTime() {}
        virtual ~SixDOFFootController() = default;

        virtual void computeStartMotion(const message::actuation::KinematicsModel& kinematicsModel,
                                        const message::input::Sensors& sensors) = 0;
        virtual void computeStopMotion(const message::input::Sensors& sensors)  = 0;

        void start(const message::actuation::KinematicsModel& kinematicsModel, const message::input::Sensors& sensors) {
            if (stage == MotionStage::READY) {
                anim.reset();
                stage           = MotionStage::RUNNING;
                stable          = false;
                motionStartTime = sensors.timestamp;
                computeStartMotion(kinematicsModel, sensors);
            }
        }

        void stop(const message::input::Sensors& sensors) {
            if (stage == MotionStage::RUNNING) {
                anim.reset();
                stage           = MotionStage::STOPPING;
                stable          = false;
                motionStartTime = sensors.timestamp;
                computeStopMotion(sensors);
            }
        }

        bool isRunning() {
            return stage == MotionStage::RUNNING || stage == MotionStage::STOPPING;
        }
        bool isStable() {
            return stable;
        }
        bool isFinished() {
            return stage == MotionStage::FINISHED;
        }
        void reset() {
            stage  = MotionStage::READY;
            stable = false;
            anim.reset();
        }


        void setKickParameters(utility::input::LimbID supportFoot_,
                               Eigen::Vector3d ballPosition_,
                               Eigen::Vector3d goalPosition_) {
            supportFoot  = supportFoot_;
            ballPosition = ballPosition_;
            goalPosition = goalPosition_;
            reset();
        }

        Eigen::Isometry3d getTorsoPose(const message::input::Sensors& sensors) {
            // Find position vector from support foot to torso in support foot coordinates.
            return ((supportFoot == utility::input::LimbID::LEFT_LEG)
                        ? Eigen::Isometry3d(sensors.Htx[utility::input::ServoID::L_ANKLE_ROLL])
                        : Eigen::Isometry3d(sensors.Htx[utility::input::ServoID::R_ANKLE_ROLL]));
        }

        Eigen::Isometry3d getFootPose(const message::input::Sensors& sensors) {
            auto result = Eigen::Isometry3d::Identity();
            if (stage == MotionStage::RUNNING || stage == MotionStage::STOPPING) {

                double elapsedTime =
                    std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count()
                    * 1e-6;
                double alpha = (anim.currentFrame().duration != 0)
                                   ? std::fmax(0, std::fmin(elapsedTime / anim.currentFrame().duration, 1))
                                   : 1;

                result = interpolate(anim.previousFrame().pose, anim.currentFrame().pose, alpha);

                bool servosAtGoal = true;

                // Check all the servos between R_HIP_YAW and L_ANKLE_ROLL are within the angle threshold
                for (int id = utility::input::ServoID::R_HIP_YAW; id <= utility::input::ServoID::L_ANKLE_ROLL; ++id) {
                    if (std::fabs(sensors.servo[id].goal_position - sensors.servo[id].present_position)
                        > servo_angle_threshold) {
                        servosAtGoal = false;
                        break;
                    }
                }

                if (alpha >= 1.0 && servosAtGoal) {
                    stable = anim.stable();
                    if (!stable) {
                        anim.next();
                        motionStartTime = sensors.timestamp;
                    }
                }
                if (stable && stage == MotionStage::STOPPING) {
                    stage = MotionStage::FINISHED;
                }
            }
            return result;
        }

        virtual void configure(const ::extension::Configuration& config) = 0;
    };

    class KickBalancer : public SixDOFFootController {
    private:
        // Config
        float stand_height    = 0.18;
        float foot_separation = 0.074;
        float forward_lean    = 0.01;
        float adjustment      = 0.011;

    public:
        virtual void configure(const ::extension::Configuration& config);
        virtual void computeStartMotion(const message::actuation::KinematicsModel& kinematicsModel,
                                        const message::input::Sensors& sensors);
        virtual void computeStopMotion(const message::input::Sensors& sensors);
    };

    class Kicker : public SixDOFFootController {
    private:
        SixDOFFrame lift_foot;
        SixDOFFrame kick;
        SixDOFFrame place_foot;

        float kick_velocity;
        float follow_through;
        float kick_height;
        float wind_up;
        float foot_separation_margin;

        float return_before_place_duration;
        float lift_before_windup_duration;

    public:
        Kicker()
            : lift_foot()
            , kick()
            , place_foot()
            , kick_velocity(0.0f)
            , follow_through(0.0f)
            , kick_height(0.0f)
            , wind_up(0.0f)
            , foot_separation_margin(0.0f)
            , return_before_place_duration(0.0f)
            , lift_before_windup_duration(0.0f) {}

        virtual void configure(const ::extension::Configuration& config);
        virtual void computeStartMotion(const message::actuation::KinematicsModel& kinematicsModel,
                                        const message::input::Sensors& sensors);
        virtual void computeStopMotion(const message::input::Sensors& sensors);
    };
}  // namespace module::motion

#endif
