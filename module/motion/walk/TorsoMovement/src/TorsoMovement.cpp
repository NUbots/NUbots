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


        TorsoMovement::TorsoMovement(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            // Left for future use
            // on<Configuration>("TorsoMovement.yaml").then([this](const Configuration& config) {

            // });


            on<Trigger<Sensors>, With<KinematicsModel>, With<TorsoTarget>>().then(
                [this](const Sensors& sensors, const KinematicsModel& model, const TorsoTarget& target) {
                    // Get support foot coordinate system

                    Eigen::Affine3d Htf;  // support foot

                    // Right foot is the support foot
                    if (target.isRightFootSupport) {
                        // Transform of right foot to torso
                        Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                    }
                    // Left foot is the support foot
                    else {
                        // Transform of left foot to torso
                        Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                    }

                    // Target given in support foot space
                    Eigen::Affine3d Haf;
                    Haf = target.Haf;

                    // Create vector to target from support foot
                    Eigen::Vector3d rAFf = -Haf.translation();
                    // Position of target in torso space
                    Eigen::Vector3d rATt = Htf * rAFf;
                    rATt.normalize();

                    // Find scale to reach target at specified time
                    std::chrono::duration<double> time_left = target.timestamp - NUClear::clock::now();
                    double distance = std::sqrt(std::pow(rATt.x(), 2) + std::pow(rATt.y(), 2) + std::pow(rATt.z(), 2));
                    double scale;
                    // 0.01 refers to 10 milliseconds as below
                    if (time_left <= std::chrono::duration<double>::zero()) {  // Time has elapsed
                        scale = 0.01;
                    }
                    else {  // There is time left to complete
                        scale = (distance / time_left.count()) * 0.01;
                    }

                    // Create next torso target in torso space
                    Eigen::Vector3d rT_tTt = rATt * scale;
                    // Create vector from torso to foot target
                    Eigen::Vector3d rF_tFt = -rT_tTt;
                    Eigen::Vector3d rF_tTt = Htf * rF_tFt;

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
                    Rf_tt = Rft.slerp(scale, Rat).toRotationMatrix();
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
