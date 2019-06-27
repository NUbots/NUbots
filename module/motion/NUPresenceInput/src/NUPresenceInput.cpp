/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "NUPresenceInput.h"

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/MotionCapture.h"
#include "message/input/PresenceUserState.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace motion {

    using extension::Configuration;

    using message::behaviour::ServoCommand;
    using message::input::MotionCapture;
    using message::input::PresenceUserState;
    using message::input::Sensors;
    using message::motion::BodySide;
    using message::motion::KinematicsModel;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using utility::behaviour::ActionPriorites;
    using utility::behaviour::RegisterAction;
    using ServoSide = utility::input::ServoSide;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using utility::support::Expression;

    NUPresenceInput::NUPresenceInput(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , l_arm(arma::fill::zeros)
        , r_arm(arma::fill::zeros)
        , mocap_head_pos(arma::fill::zeros)
        , head_id(0)
        , l_arm_id(0)
        , r_arm_id(0)
        , camera_to_robot()
        , robot_to_head()
        , mocap_to_robot()
        , oculus_to_robot_scale(0.0f)
        , eulerLimits()
        , goalCamPose()
        , currentCamPose()
        , jointLimiter()
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Configuration>("NUPresenceInput.yaml").then("Head6DoF config", [this](const Configuration& config) {
            // Use configuration here from file NUPresenceInput.yaml
            foot_separation = config["foot_separation"].as<Expression>();
            body_angle      = config["body_angle"].as<Expression>();

            float yaw      = config["robot_to_head"]["yaw"].as<Expression>();
            float pitch    = config["robot_to_head"]["pitch"].as<Expression>();
            arma::vec3 pos = config["robot_to_head"]["pos"].as<arma::vec>();

            oculus_to_robot_scale = config["robot_to_head"]["scale"].as<Expression>();
            robot_to_head         = Transform3D::createTranslation(pos) * Transform3D::createRotationZ(yaw)
                            * Transform3D::createRotationY(pitch);

            // goalCamPose = robot_to_head;
            // currentCamPose = robot_to_head;

            smoothing_alpha = config["smoothing_alpha"].as<Expression>();

            l_arm = config["l_arm"].as<arma::vec>();
            r_arm = config["r_arm"].as<arma::vec>();

            arma::vec oculus_x_axis = config["oculus"]["x_axis"].as<arma::vec>();
            arma::vec oculus_y_axis = config["oculus"]["y_axis"].as<arma::vec>();
            arma::vec oculus_z_axis = config["oculus"]["z_axis"].as<arma::vec>();

            arma::mat33 camera_to_robot_rot =
                arma::join_rows(oculus_x_axis, arma::join_rows(oculus_y_axis, oculus_z_axis));
            camera_to_robot.rotation() = camera_to_robot_rot;

            // Kinematic limits:
            distance_limit        = config["limits"]["distance"].as<Expression>();
            eulerLimits.roll.min  = config["limits"]["roll"][0].as<Expression>();
            eulerLimits.roll.max  = config["limits"]["roll"][1].as<Expression>();
            eulerLimits.pitch.min = config["limits"]["pitch"][0].as<Expression>();
            eulerLimits.pitch.max = config["limits"]["pitch"][1].as<Expression>();
            eulerLimits.yaw.min   = config["limits"]["yaw"][0].as<Expression>();
            eulerLimits.yaw.max   = config["limits"]["yaw"][1].as<Expression>();

            // Servo Limits:
            for (auto& servo : config["limits"]["servos"].config) {
                ServoID id(servo[0].as<std::string>());
                float min = servo[1].as<Expression>();
                float max = servo[2].as<Expression>();
                jointLimiter.addLimit(id, min, max);
            }
            for (auto& servo : config["limits"]["smoothing"].config) {
                ServoID id(servo[0].as<std::string>());
                float alpha = servo[1].as<Expression>();
                jointLimiter.addSmoothing(id, alpha);
            }

            updatePriority(100);

            // Mocap
            head_id  = config["mocap_rigidbody_ids"]["head"].as<int>();
            l_arm_id = config["mocap_rigidbody_ids"]["l_arm"].as<int>();
            r_arm_id = config["mocap_rigidbody_ids"]["r_arm"].as<int>();

            arma::vec mocap_x_axis = config["mocap"]["x_axis"].as<arma::vec>();
            arma::vec mocap_y_axis = config["mocap"]["y_axis"].as<arma::vec>();
            arma::vec mocap_z_axis = config["mocap"]["z_axis"].as<arma::vec>();
            mocap_to_robot         = arma::join_rows(mocap_x_axis, arma::join_rows(mocap_y_axis, mocap_z_axis));

            gyro_compensation = config["gyro_compensation"].as<bool>();
        });

        on<Network<PresenceUserState>, Sync<NUPresenceInput>>().then(
            "NUPresenceInput Network Input", [this](const PresenceUserState& user) {
                // Rotate to robot coordinate system
                goalCamPose = arma::conv_to<arma::mat>::from(convert(user.head_pose));
                goalCamPose = camera_to_robot * goalCamPose.i() * camera_to_robot.t();
                goalCamPose.translation() *= oculus_to_robot_scale;

                limitPose(goalCamPose);

                // pose << user.head_pose();
                // goalCamPose = Transform3D(arma::conv_to<arma::mat>::from(pose));
                // std::cout << "goalCamPose = \n" << goalCamPose << std::endl;
                // std::cout << "robotCamPos = " << user.head_pose().t().x() << " "<<  user.head_pose().t().y() << " "<<
                // user.head_pose().t().z() << std::endl;
            });

        on<Trigger<MotionCapture>, Sync<NUPresenceInput>>().then([this](const MotionCapture& mocap) {
            arma::vec3 l_arm_raw, r_arm_raw;
            int marker_count = 0;
            for (auto& rigidBody : mocap.rigidBodies) {

                int id  = rigidBody.id;
                float x = rigidBody.position[0];
                float y = rigidBody.position[1];
                float z = rigidBody.position[2];
                // std::cout << "Rigid body " << id << " " << arma::vec({x,y,z}).t();
                if (id == head_id) {
                    mocap_head_pos = oculus_to_robot_scale * mocap_to_robot * arma::vec3({x, y, z});
                    marker_count++;
                }
                else if (id == l_arm_id) {
                    l_arm_raw = oculus_to_robot_scale * mocap_to_robot * arma::vec3({x, y, z});
                    marker_count++;
                }
                else if (id == r_arm_id) {
                    r_arm_raw = oculus_to_robot_scale * mocap_to_robot * arma::vec3({x, y, z});
                    marker_count++;
                }
            }
            if (marker_count == 3) {
                // std::cout << "calculating arms" << std::endl;
                l_arm = (l_arm_raw - mocap_head_pos);
                r_arm = (r_arm_raw - mocap_head_pos);
            }
        });

        on<Every<60, Per<std::chrono::seconds>>, With<Sensors, KinematicsModel>, Sync<NUPresenceInput>>().then(
            [this](const Sensors& sensors, const KinematicsModel& kinematicsModel) {
                // Record current arm position:
                // arma::vec3 prevArmJointsL = {
                //                          sensors.servos[int(ServoID::L_SHOULDER_PITCH)].presentPosition,
                //                          sensors.servos[int(ServoID::L_SHOULDER_ROLL)].presentPosition,
                //                          sensors.servos[int(ServoID::L_ELBOW)].presentPosition,
                //                          };
                // arma::vec3 prevArmJointsR = {
                //                          sensors.servos[int(ServoID::R_SHOULDER_PITCH)].presentPosition,
                //                          sensors.servos[int(ServoID::R_SHOULDER_ROLL)].presentPosition,
                //                          sensors.servos[int(ServoID::R_ELBOW)].presentPosition,
                //                          };


                currentCamPose = Transform3D::interpolate(currentCamPose, robot_to_head * goalCamPose, smoothing_alpha);
                // currentCamPose.rotation() = Rotation3D();

                // 3DoF
                arma::vec3 gaze = currentCamPose.rotation().col(0);
                Rotation3D yawlessOrientation =
                    Rotation3D::createRotationZ(Rotation3D(Transform3D(-convert(sensors.Htw)).rotation()).yaw())
                    * Transform3D(convert(sensors.Htw)).rotation();

                if (gyro_compensation) {
                    gaze = yawlessOrientation * gaze;
                }
                auto joints = utility::motion::kinematics::calculateCameraLookJoints(kinematicsModel, gaze);

                // TODO: 6DOF needs fixing
                // auto joints = utility::motion::kinematics::setHeadPoseFromFeet(kinematicsModel, currentCamPose,
                // foot_separation, body_angle);

                // Adjust arm position
                // int max_number_of_iterations = 20;
                Transform3D camToBody = convert(sensors.forward_kinematics[ServoID::HEAD_PITCH]);
                arma::vec3 kneckPos   = {kinematicsModel.head.NECK_BASE_POS_FROM_ORIGIN_X,
                                       kinematicsModel.head.NECK_BASE_POS_FROM_ORIGIN_Y,
                                       kinematicsModel.head.NECK_BASE_POS_FROM_ORIGIN_Z};
                auto arm_jointsL = utility::motion::kinematics::setArmApprox(kinematicsModel, kneckPos + l_arm, true);
                auto arm_jointsR = utility::motion::kinematics::setArmApprox(kinematicsModel, kneckPos + r_arm, false);
                // joints.insert(joints.end(), arm_jointsL.begin(), arm_jointsL.end());
                // joints.insert(joints.end(), arm_jointsR.begin(), arm_jointsR.end());

                auto waypoints = std::make_unique<std::vector<ServoCommand>>();
                waypoints->reserve(16);

                NUClear::clock::time_point time = NUClear::clock::now();

                for (auto& joint : joints) {
                    waypoints->push_back(ServoCommand(id,
                                                      time,
                                                      joint.first,
                                                      jointLimiter.clampAndSmooth(joint.first, joint.second),
                                                      30,
                                                      100));  // TODO: support separate gains for each leg
                }
                emit(waypoints);

                // Transform3D R_shoulder_pitch = sensors.forward_kinematics[ServoID::R_SHOULDER_PITCH];
                // Transform3D R_shoulder_roll = sensors.forward_kinematics[ServoID::R_SHOULDER_ROLL];
                // Transform3D R_arm = sensors.forward_kinematics[ServoID::R_ELBOW];
                // Transform3D L_shoulder_pitch = sensors.forward_kinematics[ServoID::L_SHOULDER_PITCH];
                // Transform3D L_shoulder_roll = sensors.forward_kinematics[ServoID::L_SHOULDER_ROLL];
                // Transform3D L_arm = sensors.forward_kinematics[ServoID::L_ELBOW];

                // arma::vec3 zeros = arma::zeros(3);
                // arma::vec3 zero_pos = utility::motion::kinematics::calculateArmPosition(kinematicsModel, zeros,
                // true);

                // std::cout << "New zero pos = \n" << zero_pos << std::endl;
                // std::cout << "Traditional FK R_shoulder_pitch = \n" << R_shoulder_pitch << std::endl;
                // std::cout << "Traditional FK R_shoulder_roll = \n" << R_shoulder_roll << std::endl;
                // std::cout << "Traditional FK R_arm = \n" << R_arm << std::endl;
                // std::cout << "Traditional FK L_shoulder_pitch = \n" << L_shoulder_pitch << std::endl;
                // std::cout << "Traditional FK L_shoulder_roll = \n" << L_shoulder_roll << std::endl;
                // std::cout << "Traditional FK L_arm = \n" << L_arm << std::endl;
            });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "Head 6DoF Controller",
            {std::pair<float, std::set<LimbID>>(
                0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::HEAD, LimbID::RIGHT_ARM, LimbID::LEFT_ARM})},
            [this](const std::set<LimbID>&) {
                // emit(std::make_unique<ExecuteGetup>());
            },
            [this](const std::set<LimbID>&) {
                // emit(std::make_unique<KillGetup>());
            },
            [this](const std::set<ServoID>& /*servoSet*/) {
                // HACK 2014 Jake Fountain, Trent Houliston
                // TODO track set limbs and wait for all to finish
                // if(servoSet.find(ServoID::L_ANKLE_PITCH) != servoSet.end() ||
                //    servoSet.find(ServoID::R_ANKLE_PITCH) != servoSet.end()) {
                //     emit(std::make_unique<KillGetup>());
                // }
            }}));
    }


    void NUPresenceInput::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {priority}}));
    }

    void NUPresenceInput::limitPose(Transform3D& pose) {
        float norm = arma::norm(pose.translation());
        if (norm > distance_limit) {
            pose.translation() = distance_limit * pose.translation() / norm;
        }

        arma::vec3 eulerAngles = pose.eulerAngles();
        // std::cout << "eulerAngles = " << eulerAngles.t();
        // std::cout << "eulerLimits = " << ", " << eulerLimits.roll.max << ", " << eulerLimits.roll.min << "; " <<
        // eulerLimits.pitch.max << ", " << eulerLimits.pitch.min << "; " << eulerLimits.yaw.max << ", " <<
        // eulerLimits.yaw.min << std::endl;
        eulerAngles[0] = std::fmax(std::fmin(eulerAngles[0], eulerLimits.roll.max), eulerLimits.roll.min);
        eulerAngles[1] = std::fmax(std::fmin(eulerAngles[1], eulerLimits.pitch.max), eulerLimits.pitch.min);
        eulerAngles[2] = std::fmax(std::fmin(eulerAngles[2], eulerLimits.yaw.max), eulerLimits.yaw.min);
        // std::cout << "eulerAngles = " << eulerAngles.t();
        pose.rotation() = Rotation3D::createFromEulerAngles(eulerAngles);
        // std::cout << "check = " << pose.rotation() - R << std::endl;
    }
}  // namespace motion
}  // namespace module
