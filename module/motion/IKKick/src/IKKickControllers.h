/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_IKKICKCONTROLLERS_H
#define MODULES_MOTION_IKKICKCONTROLLERS_H

#include <armadillo>
#include <nuclear>

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace motion {

    enum MotionStage { READY = 0, RUNNING = 1, STOPPING = 2, FINISHED = 3 };

    class SixDOFFrame {
        // enum InterpolationType {
        //  LINEAR = 0,
        //  SERVO = 1
        // TODO:
        // };
        // InterpolationType interpolation = LINEAR;
    public:
        utility::math::matrix::Transform3D pose;
        float duration;
        SixDOFFrame() : pose(), duration(0.0f) {}
        SixDOFFrame(utility::math::matrix::Transform3D pose_, float duration_) : pose(pose_), duration(duration_) {}
        SixDOFFrame(const YAML::Node& config) : SixDOFFrame() {
            duration               = config["duration"].as<float>();
            arma::vec3 pos         = config["pos"].as<arma::vec>();
            arma::vec3 orientation = (180.0 / M_PI) * config["orientation"].as<arma::vec>();
            pose                   = utility::math::matrix::Transform3D();
            pose.rotateX(orientation[0]);
            pose.rotateY(orientation[1]);
            pose.rotateZ(orientation[2]);
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
            frames.push_back(SixDOFFrame{utility::math::matrix::Transform3D(), 0});
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

        arma::vec3 ballPosition;
        arma::vec3 goalPosition;

        NUClear::clock::time_point motionStartTime;

    public:
        SixDOFFootController()
            : supportFoot()
            , forward_duration(0.0f)
            , return_duration(0.0f)
            , anim()
            , ballPosition(arma::fill::zeros)
            , goalPosition(arma::fill::zeros)
            , motionStartTime() {}
        virtual ~SixDOFFootController() = default;

        virtual void computeStartMotion(const message::motion::KinematicsModel& kinematicsModel,
                                        const message::input::Sensors& sensors) = 0;
        virtual void computeStopMotion(const message::input::Sensors& sensors)  = 0;

        void start(const message::motion::KinematicsModel& kinematicsModel, const message::input::Sensors& sensors) {
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
                               arma::vec3 ballPosition_,
                               arma::vec3 goalPosition_) {
            supportFoot  = supportFoot_;
            ballPosition = ballPosition_;
            goalPosition = goalPosition_;
            reset();
        }

        utility::math::matrix::Transform3D getTorsoPose(const message::input::Sensors& sensors) {
            // Find position vector from support foot to torso in support foot coordinates.
            return ((supportFoot == utility::input::LimbID::LEFT_LEG)
                        ? convert(sensors.forward_kinematics[utility::input::ServoID::L_ANKLE_ROLL])
                        : convert(sensors.forward_kinematics[utility::input::ServoID::R_ANKLE_ROLL]));
        }

        utility::math::matrix::Transform3D getFootPose(const message::input::Sensors& sensors) {
            auto result = utility::math::matrix::Transform3D();
            if (stage == MotionStage::RUNNING || stage == MotionStage::STOPPING) {

                double elapsedTime =
                    std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count()
                    * 1e-6;
                float alpha = (anim.currentFrame().duration != 0)
                                  ? std::fmax(0, std::fmin(elapsedTime / anim.currentFrame().duration, 1))
                                  : 1;
                result = utility::math::matrix::Transform3D::interpolate(
                    anim.previousFrame().pose, anim.currentFrame().pose, alpha);

                bool servosAtGoal = true;
                for (auto& servo : sensors.servo) {
                    if (int(servo.id) >= 6         // R_HIP_YAW
                        && int(servo.id) <= 17) {  // L_ANKLE_ROLL
                        servosAtGoal =
                            servosAtGoal
                            && std::fabs(servo.goal_position - servo.present_position) < servo_angle_threshold;
                    }
                }

                if (alpha >= 1 && servosAtGoal) {
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
        virtual void computeStartMotion(const message::motion::KinematicsModel& kinematicsModel,
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
        virtual void computeStartMotion(const message::motion::KinematicsModel& kinematicsModel,
                                        const message::input::Sensors& sensors);
        virtual void computeStopMotion(const message::input::Sensors& sensors);
    };
}  // namespace motion
}  // namespace module

#endif
