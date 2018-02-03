#include "TorsoMovement.h"

#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

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
        using message::motion::TorsoMovement;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::nubugger::graph;


        TorsoMovement::TorsoMovement(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("TorsoMovement.yaml").then([this](const Configuration& config) {

                emit(std::make_unique<TorsoMovement>(
                    NUClear::clock::now() + std::chrono::seconds(1), foot, Eigen::Vector3d(x, y, z)));
            });


            on<Trigger<Sensors>, With<KinematicsModel>, With<TorsoMovement>>().then(
                [this](const Sensors& sensors, const KinematicsModel& model, const FootTarget& target) {
                    // Get support foot and swing foot coordinate systems

                    Eigen::Affine3d Htf_s;  // support foot
                    Eigen::Affine3d Htf_w;  // swing foot

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


                    // Get orientation for world (Rotation of world->torso)
                    Eigen::Affine3d Rtw;
                    Rtw.linear() = Eigen::Affine3d(sensors.world).rotation();

                    // Get the yawless world rotation
                    Eigen::Affine3d Rtg =
                        Rtw * Eigen::AngleAxisd(-Rtw.rotation().eulerAngles(0, 2, 1).y(), Eigen::Vector3d::UnitZ());

                    // Apply the foot yaw to the yawless world rotation
                    Rtg = Rtg * Eigen::AngleAxisd(Htf_s.rotation().eulerAngles(0, 2, 1).y(), Eigen::Vector3d::UnitZ());

                    // Construct a torso to foot ground space (support foot centric world oriented space)
                    Eigen::Affine3d Htg;
                    Htg.linear()      = Rtg.linear();         // Rotation from Rtg
                    Htg.translation() = Htf_s.translation();  // Translation is the same as to the support foot

                    // Target position is in ground space
                    Eigen::Vector3d rAGg = target.position;
                    // Position of target in torso space
                    Eigen::Vector3d rATt = Htg * rAGg;
                    // Next torso target in torso space
                    Eigen::Vector3d rT_tTt = rATt.normalized() * 0.01;
                    // Get target
                    Eigen::Vector3d rF_tTt = ;
                    Eigen::Affine3d Htf_t;
                    Htf_t.linear()      = Has.inverse().linear();  // Support foot rotation
                    Htf_t.translation() = rF_tTt;                  // Translation to foot target

                    Transform3D t = convert<double, 4, 4>(Htf_t.matrix());
                    auto joints =
                        calculateLegJoints(model, t, target.isRightFootSupport ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG);


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
