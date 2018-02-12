#include "TorsoMovement.h"

#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/TorsoTarget.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;

        using message::behaviour::ServoCommand;
        using message::input::Sensors;
        using message::motion::KinematicsModel;
        using message::motion::TorsoTarget;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::nubugger::graph;


        TorsoMovement::TorsoMovement(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("TorsoMovement.yaml").then([this](const Configuration& config) {
                double x = config["test"]["x"].as<double>();
                double y = config["test"]["y"].as<double>();
                double z = config["test"]["z"].as<double>();
                int foot = config["foot"].as<int>();
                Eigen::Affine3d Haf_s;
                Haf_s.linear()      = Eigen::Matrix3d::Identity();
                Haf_s.translation() = -Eigen::Vector3d(x, y, z);
                emit(std::make_unique<TorsoTarget>(
                    NUClear::clock::now() + std::chrono::seconds(1), foot, Haf_s.matrix()));
            });


            on<Trigger<Sensors>, With<KinematicsModel>, With<TorsoTarget>>().then(
                [this](const Sensors& sensors, const KinematicsModel& model, const TorsoTarget& target) {
                    // Get support foot and swing foot coordinate systems

                    Eigen::Affine3d Htf_s;  // support foot

                    // Right foot is the swing foot
                    if (target.isRightFootSupport) {
                        // Transform of left foot to torso
                        Htf_s = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                    }
                    // Left foot is the swing foot
                    else {
                        // Transform of right foot to torso
                        Htf_s = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                    }

                    Eigen::Affine3d Haf_s;
                    Haf_s = target.Haf_s;

                    // Target position is in foot space
                    Eigen::Vector3d rAF_sf_s = -Haf_s.translation();
                    // Position of target in torso space
                    Eigen::Vector3d rATt = Htf_s * rAF_sf_s;
                    // Next torso target in torso space
                    Eigen::Vector3d rT_tTt = rATt * 0.01;
                    // Create vector from torso to foot target
                    Eigen::Vector3d rF_tF_st = -rT_tTt;
                    Eigen::Vector3d rF_tTt   = Htf_s * rF_tF_st;

                    // Target rotation from torso space
                    Eigen::Affine3d Hat;
                    Hat = Haf_s * Htf_s.inverse();
                    // Get support foot rotation from torso as quaternion
                    Eigen::Quaterniond Rf_st;
                    Rf_st = Htf_s.inverse().linear();
                    // Get target rotation from torso as quaternion
                    Eigen::Quaterniond Rat;
                    Rat = Hat.linear();
                    // Create rotation of torso to torso target
                    Eigen::Matrix3d Rf_tt;
                    // Slerp the above two Quaternions and switch to rotation matrix to get the rotation
                    // TODO: determine t
                    Rf_tt = Rf_st.slerp(0.1, Rat).toRotationMatrix();
                    Eigen::Affine3d Htf_t;
                    Htf_t.linear()      = Rf_tt.inverse();  // Rotation as above from slerp
                    Htf_t.translation() = rF_tTt;           // Translation to foot target

                    Transform3D t = convert<double, 4, 4>(Htf_t.matrix());
                    auto joints =
                        calculateLegJoints(model, t, target.isRightFootSupport ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG);

                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                    for (const auto& joint : joints) {
                        waypoints->push_back({subsumptionId,
                                              NUClear::clock::now() + std::chrono::milliseconds(10),
                                              joint.first,
                                              joint.second,
                                              20,
                                              100});  // TODO: support separate gains for each leg
                    }

                    emit(waypoints);
                });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{subsumptionId,
                               "TorsoMovement",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
