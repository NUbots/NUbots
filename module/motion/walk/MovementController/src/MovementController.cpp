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
        using utility::nusight::graph;

        double MovementController::f_x(const Eigen::Vector3d& pos) {
            return -pos.x() / std::abs(pos.x()) * std::exp(-std::abs(std::pow(c * pos.x(), -step_steep)));
        }

        double MovementController::f_y(const Eigen::Vector3d& pos) {
            return std::exp(-std::abs(std::pow(c * pos.x(), -step_steep))) - pos.y() / step_height;
        }

        Eigen::Affine3d MovementController::plan_torso(const Sensors& sensors, const TorsoTarget& target) {
            // Get support foot coordinate system
            Eigen::Affine3d Htf;

            // Right foot is the support foot
            if (target.isRightFootSupport) {
                Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
            }
            // Left foot is the support foot
            else {
                Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
            }

            // Target given in support foot space
            Eigen::Affine3d Haf;
            Haf = target.Haf;

            // Position of the target in torso space
            Eigen::Vector3d rATt = Htf * -Haf.translation();

            // Find scale to reach target at specified time based on distance from target, time left, and the
            // time horizon
            std::chrono::duration<double> time_left = target.timestamp - offset_time - NUClear::clock::now();
            double distance                         = rATt.norm();
            double scale =
                time_left > std::chrono::duration<double>::zero() ? (distance / time_left.count()) * time_horizon : 1;
            // If the torso is close enough to the position, stop running
            if (distance < 0.001) {
                scale = 1;
            }

            // Create next torso target in torso space
            Eigen::Vector3d rT_tTt = rATt.normalized() * scale;

            // If our scale would move us past the target, then just go to the target
            if ((scale > distance) || (scale == 1)) {
                rT_tTt = rATt;
            }

            // Create vector from torso to foot target
            // rT_tTt ~ rF_tFf
            Eigen::Vector3d rF_tTt = Htf * -rT_tTt;

            // Target rotation from torso space
            Eigen::Affine3d Hat;
            Hat = Haf * Htf.inverse();
            // Get support foot rotation from torso as quaternion
            Eigen::Quaterniond Rft;
            Rft = Htf.inverse().linear();
            // Get target rotation from torso as quaternion
            Eigen::Quaterniond Rat;
            Rat = Hat.linear();
            // Create rotation of torso to torso target
            Eigen::Matrix3d Rf_tt;
            // Slerp the above two Quaternions and switch to rotation matrix to get the rotation
            // Scale as above to rotate in specified time
            Rf_tt = Rft.slerp(1, Rat).toRotationMatrix();

            Eigen::Affine3d Htf_t;
            // Htf_t.linear() = Htf.linear();  // No rotation
            Htf_t.linear()      = Rf_tt.inverse();  // Rotation as above from slerp
            Htf_t.translation() = rF_tTt;           // Translation to foot target
            return Htf_t;
        }

        Eigen::Affine3d MovementController::plan_swing(const Sensors& sensors,
                                                       const FootTarget& target,
                                                       const Eigen::Affine3d& Htf_s) {
            // Get swing foot coordinate system from support foot coordinate system given
            Eigen::Affine3d Htf_w;  // swing foot
            Eigen::Affine3d Htt_t;

            // Right foot is the swing foot
            if (target.isRightFootSwing) {
                // Htf_s * Ht_tf_s.inverse() = Htt_t
                Htt_t = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]) * Htf_s.inverse();
                // Htt_t.inverse() * Htf_w = Ht_tt * Htf_w = Ht_tf_w
                Htf_w = Htt_t.inverse() * Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
            }
            // Left foot is the swing foot
            else {
                // Htf_s * Ht_tf_s.inverse() = Htt_t
                Htt_t = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]) * Htf_s.inverse();
                // Htt_t.inverse() * Htf_w = Ht_tt * Htf_w = Ht_tf_w
                Htf_w = Htt_t.inverse() * Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
            }

            Eigen::Affine3d Haf_s;
            Haf_s = target.Haf_s;

            // Get orientation for world (Rotation of world->torso)
            Eigen::Matrix3d Rtw;
            // Ht_tt * Htw = Ht_tw
            Rtw = (Htt_t.inverse() * Eigen::Affine3d(sensors.world)).rotation();

            // Convert Rtw and Htf_s rotation into euler angles to get pitch and roll from Rtw and yaw from
            // Htf_s to create a new rotation matrix
            Eigen::Vector3d ea_Rtw;
            ea_Rtw = Rtw.eulerAngles(0, 1, 2);
            Eigen::Vector3d ea_Htf_s;
            ea_Htf_s = Htf_s.rotation().eulerAngles(0, 1, 2);

            // Rtg is the yawless world rotation
            Eigen::Matrix3d Rtg;
            Rtg = Eigen::AngleAxisd(ea_Rtw.x(), Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(ea_Rtw.y(), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(ea_Htf_s.z(), Eigen::Vector3d::UnitZ());

            // Construct a torso to foot ground space (support foot centric world oriented space)
            Eigen::Affine3d Htg;
            Htg.linear()      = Rtg;                  // Rotation from Rtg
            Htg.translation() = Htf_s.translation();  // Translation is the same as to the support foot

            // Vector to the swing foot in ground space
            Eigen::Vector3d rF_wGg = Htg.inverse() * Htf_w.translation();
            // Vector from ground to target
            // Hgf_s * rAF_sf_s
            Eigen::Vector3d rAGg = (Htg.inverse() * Htf_s) * -Haf_s.translation();

            // Direction of the target from the swing foot
            Eigen::Vector3d rAF_wg = rAGg - rF_wGg;

            // Create a rotation to the plane that cuts through the two positions
            Eigen::Matrix3d Rgp;
            // X axis is the direction towards the target
            Rgp.leftCols<1>() = rAF_wg.normalized();
            // Y axis is straight up
            Rgp.middleCols<1>(1) = Eigen::Vector3d::UnitZ();
            // Z axis is the cross product of X and Y
            Rgp.rightCols<1>() = Rgp.leftCols<1>().cross(Rgp.middleCols<1>(1)).normalized();
            // Fix X axis by finding the cross product Y and Z
            Rgp.leftCols<1>() = Rgp.middleCols<1>(1).cross(Rgp.rightCols<1>()).normalized();

            // Create transform based on above rotation
            Eigen::Affine3d Hgp;       // plane to ground transform
            Hgp.linear()      = Rgp;   // Rotation from above
            Hgp.translation() = rAGg;  // Translation to target

            // Make a transformation matrix that goes the whole way
            Eigen::Affine3d Htp = Htg * Hgp;

            // Swing foot's position on plane
            Eigen::Vector3d rF_wPp = Hgp.inverse() * rF_wGg;

            // Find scale to reach target at specified time
            std::chrono::duration<double> time_left = target.timestamp - offset_time - NUClear::clock::now();
            double distance                         = rF_wPp.norm();
            double scale =
                time_left > std::chrono::duration<double>::zero() ? (distance / time_left.count()) * time_horizon : 1;

            // Swing foot's new target position on the plane, scaled for time
            Eigen::Vector3d rF_tPp = rF_wPp + Eigen::Vector3d(f_x(rF_wPp), f_y(rF_wPp), 0).normalized() * scale;

            // If no lift is wanted, then set the y (vertical axis in plane space) to 0
            if (!target.lift) {
                rF_tPp.y() = 0;
            }

            // If the scale is more than the distance, go to the target to avoid overshooting
            if (((scale > distance) || (distance < 0.001)) || (scale == 1)) {
                rF_tPp = Eigen::Vector3d(0, 0, 0);
            }

            // Foot target's position relative to torso
            Eigen::Vector3d rF_tTt = Htp * rF_tPp;

            // Torso to target transform
            Eigen::Affine3d Hat;
            Hat = Haf_s * Htf_s.inverse();
            // Get torso to swing foot rotation as quaternion
            Eigen::Quaterniond Rf_wt;
            Rf_wt = Htf_w.inverse().linear();
            // Create rotation of torso to target as a quaternion
            Eigen::Quaterniond Rat;
            Rat = Hat.linear();
            // Create rotation matrix for foot target
            Eigen::Matrix3d Rf_tt;
            // Slerp the above two Quaternions and switch to rotation matrix to get the rotation
            Rf_tt = Rf_wt.slerp(1, Rat).toRotationMatrix();

            Eigen::Affine3d Htf_t;
            // Htf_t.linear() = Htf_w.linear(); // No rotation
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
                    // Determine where the torso will move to (ie where the support foot will move)
                    Eigen::Affine3d torso_Htf_t;
                    torso_Htf_t = plan_torso(sensors, torso_target);

                    // Determine where the swing foot will move, using the torso's result
                    Eigen::Affine3d swing_Htf_t;
                    swing_Htf_t = plan_swing(sensors, foot_target, torso_Htf_t);

                    // Create transforms for inverse kinematics
                    Transform3D t_t = convert<double, 4, 4>(torso_Htf_t.matrix());
                    Transform3D t_f = convert<double, 4, 4>(swing_Htf_t.matrix());

                    // Use inverse kinematics to calculate joint positions
                    auto joints = torso_target.isRightFootSupport ? calculateLegJoints(model, t_f, t_t)
                                                                  : calculateLegJoints(model, t_t, t_f);


                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                    for (const auto& joint : joints) {
                        waypoints->push_back(
                            {foot_target.subsumption_id,
                             NUClear::clock::now()
                                 + NUClear::clock::duration(int(time_horizon * NUClear::clock::period::den)),
                             joint.first,
                             joint.second,
                             20,
                             100});
                    }
                    emit(waypoints);
                });
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
