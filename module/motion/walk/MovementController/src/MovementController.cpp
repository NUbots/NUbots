#include "MovementController.h"

#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/TorsoTarget.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;

        using message::behaviour::ServoCommand;
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::KinematicsModel;
        using message::motion::TorsoTarget;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::motion::kinematics::legPoseValid;
        using utility::nusight::graph;

        double MovementController::f_x(const Eigen::Vector3d& pos) {
            return -pos.x() / std::abs(pos.x()) * std::exp(-std::abs(std::pow(c * pos.x(), -step_steep)));
        }

        double MovementController::f_z(const Eigen::Vector3d& pos) {
            return std::exp(-std::abs(std::pow(c * pos.x(), -step_steep))) - pos.z() / step_height;
        }

        Eigen::Affine3d MovementController::plan_torso(const NUClear::clock::time_point& now,
                                                       const Sensors& sensors,
                                                       const TorsoTarget& target) {
            // Get support foot coordinate system
            const Eigen::Affine3d Htf(target.isRightFootSupport
                                          ? Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL])
                                          : Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]));

            // Target given in support foot space
            Eigen::Affine3d Haf(target.Haf);

            // Get orientation for world (Rotation of world->torso)
            // Eigen::Matrix3d Rtw(Eigen::Affine3d(sensors.world).rotation());

            // Construct a torso to foot ground space (support foot centric world oriented space)
            // Eigen::Affine3d Htg;
            // Htg.linear()      = Rtw;                // Rotation from Rtg
            // Htg.translation() = Htf.translation();  // Translation is the same as to the support foot


            // Position of the target in torso space
            Eigen::Vector3d rATt(Htf * Haf.inverse().translation());

            // Find scale to reach target at specified time based on distance from target, time left, and the time
            // horizon
            double time_left(
                std::chrono::duration_cast<std::chrono::duration<double>>(target.timestamp - now - offset_time)
                    .count());
            // If we have less time left than our time horizon, assume we have until time horizon to finish
            time_left = time_left < time_horizon ? time_horizon : time_left;

            double horizon_distance((rATt.norm() / time_left) * time_horizon);

            // Create next torso target in torso space
            Eigen::Vector3d rT_tTt(rATt.normalized() * (horizon_distance));

            // Create vector from foot target to torso
            Eigen::Vector3d rT_tFf(Htf.inverse() * rT_tTt);


            // Torso to target transform
            Eigen::Affine3d Hat(Haf * Htf.inverse());
            // Get torso to swing foot rotation as quaternion
            Eigen::Quaterniond Rtf(Htf.rotation());
            // Create rotation of torso to target as a quaternion
            Eigen::Quaterniond Raf((Hat * Htf.inverse()).linear());
            // Create rotation matrix for foot target
            // Slerp the above two Quaternions and switch to rotation matrix to get the rotation
            Eigen::Matrix3d Rt_tf(Rtf.inverse().slerp(time_horizon / time_left, Raf).toRotationMatrix());

            // Create the final position matrix to return
            Eigen::Affine3d Htf_t;
            // Htf_t.linear() = Htf.linear();
            // Htf_t.linear() = Eigen::Matrix3d::Identity();
            Htf_t.linear()      = Rt_tf;    // Rotation as above from slerp
            Htf_t.translation() = -rT_tFf;  // Translation to target
            // log("\nHtf\n", convert<double, 4, 4>(Htf.matrix()));
            log("\nHtf_t\n", convert<double, 4, 4>(Htf_t.matrix()));
            return Htf_t;
        }

        Eigen::Affine3d MovementController::plan_swing(const NUClear::clock::time_point& now,
                                                       const Sensors& sensors,
                                                       const FootTarget& target,
                                                       const Eigen::Affine3d& Htf_s) {
            // Get swing foot coordinate system from support foot coordinate system given
            // And get the new torso space compared to old
            Eigen::Affine3d Htt_t =
                target.isRightFootSwing
                    ? Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]) * Htf_s.inverse()
                    : Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]) * Htf_s.inverse();
            Eigen::Affine3d Htf_w =
                target.isRightFootSwing
                    ? Htt_t.inverse() * Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL])
                    : Htt_t.inverse() * Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);

            Eigen::Affine3d Haf_s(target.Haf_s);

            // Get orientation for world (Rotation of world->torso) relative to new torso space
            Eigen::Matrix3d Rtw((Htt_t.inverse() * Eigen::Affine3d(sensors.world)).rotation());

            // Convert Rtw and Htf_s rotation into euler angles to get pitch and roll from Rtw and yaw from
            // Htf_s to create a new rotation matrix
            Eigen::Vector3d ea_Rtw(Rtw.eulerAngles(0, 1, 2));
            Eigen::Vector3d ea_Htf_s(Htf_s.rotation().eulerAngles(0, 1, 2));

            // Rtg is the yawless world rotation
            Eigen::Matrix3d Rtg(Eigen::AngleAxisd(ea_Rtw.x(), Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(ea_Rtw.y(), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(ea_Rtw.y() - ea_Htf_s.z(), Eigen::Vector3d::UnitZ()));

            // Construct a torso to foot ground space (support foot centric world oriented space)
            Eigen::Affine3d Htg;
            Htg.linear()      = Rtg;                  // Rotation from Rtg
            Htg.translation() = Htf_s.translation();  // Translation is the same as to the support foot

            // Vector to the swing foot in ground space
            // Hgt * rF_wTt = rF_wGg
            Eigen::Vector3d rF_wGg = Htg.inverse() * Htf_w.translation();
            // Vector from ground to target
            // Hgf_s * rAF_sf_s
            Eigen::Vector3d rAGg = (Htg.inverse() * Htf_s) * -Haf_s.translation();

            // Direction of the target from the swing foot
            Eigen::Vector3d rAF_wg = rAGg - rF_wGg;
            // Create a rotation to the plane that cuts through the two positions
            Eigen::Matrix3d Rgp;
            // X axis is the direction towards the target. Make the z=0 so that it is at a right angle to the y-axis.
            // This makes the vector field straight rather than on a slope.
            rAF_wg.z() = 0;
            Rgp.col(0) = rAF_wg.normalized();
            // Z axis is straight up
            Rgp.col(2) = Eigen::Vector3d::UnitZ();
            // Y axis is the cross product of X and Z. This makes the y-axis at a right angle to both the x-axis and
            // z-axis
            Rgp.col(1) = Rgp.col(0).cross(Rgp.col(2)).normalized();
            // Rgp.leftCols<1>()  = Rgp.middleCols<1>(1).cross(Rgp.rightCols<1>()).normalized();

            // Create transform based on above rotation
            Eigen::Affine3d Hgp;       // plane to ground transform
            Hgp.linear()      = Rgp;   // Rotation from above
            Hgp.translation() = rAGg;  // Translation to target

            // Make a transformation matrix that goes the whole way
            Eigen::Affine3d Htp(Htg * Hgp);

            // Swing foot's position on plane
            Eigen::Vector3d rF_wPp(Hgp.inverse() * rF_wGg);

            // Find scale to reach target at specified time based on distance from target, time left, and the
            // time horizon
            double time_left(
                std::chrono::duration_cast<std::chrono::duration<double>>(target.timestamp - now - offset_time)
                    .count());
            // If we have less time left than our time horizon, assume we have until time horizon to finish
            time_left               = time_left < time_horizon ? time_horizon : time_left;
            double distance         = rF_wPp.norm();  // TODO line integral goes here!
            double horizon_distance = (distance / time_left) * time_horizon;

            // Swing foot's new target position on the plane, scaled for time based on the horizon
            Eigen::Vector3d rF_tPp(rF_wPp
                                   + Eigen::Vector3d(f_x(rF_wPp), 0, f_z(rF_wPp)).normalized() * horizon_distance);

            // If no lift is wanted, then set the y (vertical axis in plane space) to 0
            if (!target.lift) {
                rF_tPp.z() = 0;
            }

            // If small distance, go to target
            if (distance < well_width) {
                rF_tPp = Eigen::Vector3d::Zero();
            }

            // Foot target's position relative to torso
            Eigen::Vector3d rF_tTt(Htp * rF_tPp);

            // Torso to target transform
            Eigen::Affine3d Hat(Haf_s * Htf_s.inverse());
            // Get torso to swing foot rotation as quaternion
            Eigen::Quaterniond Rgt(Rtg.inverse());
            // Create rotation of torso to target as a quaternion
            Eigen::Quaterniond Rat(Hat.linear());
            // Create rotation matrix for foot target
            // Slerp the above two Quaternions and switch to rotation matrix to get the rotation
            Eigen::Matrix3d Rf_tt(Rgt.inverse().slerp(time_horizon / time_left, Rat).toRotationMatrix());

            Eigen::Affine3d Htf_t;
            // Htf_t.linear() = Rtg;
            Htf_t.linear()      = Rf_tt.inverse();  // Rotation from above
            Htf_t.translation() = rF_tTt;           // Translation to foot target

            return Htf_t;
        }


        MovementController::MovementController(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Configuration>("MovementController.yaml").then([this](const Configuration& config) {
                // Use configuration here from file MovementController.yaml
                step_height  = config["step_height"].as<double>();
                well_width   = config["well_width"].as<double>();
                step_steep   = config["step_steep"].as<double>();
                time_horizon = config["time_horizon"].as<double>();
                gain         = config["gain"].as<double>();
                offset_time  = std::chrono::milliseconds(config["offset_time"].as<int>());

                // Constant for f_x and f_y
                c = (std::pow(step_steep, 2 / step_steep) * std::pow(step_height, 1 / step_steep)
                     * std::pow(step_steep * step_height + (step_steep * step_steep * step_height), -1 / step_steep))
                    / well_width;
            });

            on<Trigger<Sensors>, With<KinematicsModel>, With<FootTarget>, With<TorsoTarget>>().then(
                [this](const Sensors& sensors,
                       const KinematicsModel& model,
                       const FootTarget& foot_target,
                       const TorsoTarget& torso_target) {
                    using namespace std::chrono;

                    // Set the time now so the calculations are consistent across the methods
                    auto now = NUClear::clock::now();

                    // Determine where the torso will move to (ie where the support foot will move)
                    Eigen::Affine3d torso_Htf_t = plan_torso(now, sensors, torso_target);

                    // Determine where the swing foot will move, using the torso's result
                    Eigen::Affine3d swing_Htf_t = plan_swing(now, sensors, foot_target, torso_Htf_t);

                    // Create transforms for inverse kinematics
                    Transform3D t_t               = convert<double, 4, 4>(torso_Htf_t.matrix());
                    Transform3D t_f               = convert<double, 4, 4>(swing_Htf_t.matrix());
                    const Transform3D& left_foot  = torso_target.isRightFootSupport ? t_f : t_t;
                    const Transform3D& right_foot = torso_target.isRightFootSupport ? t_t : t_f;
                    bool left_valid               = legPoseValid(model, left_foot, LimbID::LEFT_LEG);
                    bool right_valid              = legPoseValid(model, right_foot, LimbID::RIGHT_LEG);
                    if (!left_valid) {
                        log("invalid pose on left leg\n", left_foot);
                    }
                    if (!right_valid) {
                        log("invalid pose on right leg\n", right_foot);
                    }
                    if (left_valid && right_valid) {
                        log("both leg poses are valid\n", left_foot, "\n", right_foot);

                        // Use inverse kinematics to calculate joint positions
                        auto joints    = calculateLegJoints(model, left_foot, right_foot);
                        auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                        // Create the target time based on the time horizon
                        NUClear::clock::time_point target_time =
                            time_point_cast<NUClear::clock::duration>(now + duration<double>(time_horizon));

                        // HACK: By putting these waypoints first, we trick the Controller into dumping future commands
                        // (previous horizon)
                        for (const auto& joint : joints) {
                            waypoints->emplace_back(
                                foot_target.subsumption_id, now, joint.first, joint.second, gain, 100);
                        }

                        for (const auto& joint : joints) {
                            waypoints->emplace_back(
                                foot_target.subsumption_id, target_time, joint.first, joint.second, gain, 100);
                        }
                        emit(waypoints);
                    }
                    else {
                    }
                });
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
