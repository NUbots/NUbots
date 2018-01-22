#include "FootStep.h"

#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.h"

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
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
        using message::motion::FootTarget;
        using message::motion::KinematicsModel;

        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::nubugger::graph;

        double FootStep::f_x(const Eigen::Vector3d& pos) {
            if (pos.x() > 0) {
                return std::pow(2, std::pow(-pos.x() / d, -step_steep));
            }
            else {
                return -1 * std::pow(2, std::pow(-pos.x() / d, -step_steep));
            }
            // return -std::tanh(pos.x()) * step_height * std::pow(2, (step_steep / -std::abs(std::pow(pos.x(), d))));
        }

        double FootStep::f_y(const Eigen::Vector3d& pos) {
            return std::pow(2, std::pow(-pos.x(), -step_steep));
            // return step_height * pow(2, (step_steep / -std::abs(std::pow(pos.x(), d))))
            //     - pos.y() * abs((std::tanh(10 * pos.y())));
        }

        FootStep::FootStep(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("FootStep.yaml").then([this](const Configuration& config) {
                // Use configuration here from file FootStep.yaml
                step_height = config["step_height"];
                d           = config["d"];
                step_steep  = config["step_steep"];

                double x = config["test"]["x"].as<double>();
                double y = config["test"]["y"].as<double>();
                double z = config["test"]["z"].as<double>();
                log("Making new foot target at", x, y);
                emit(std::make_unique<FootTarget>(
                    NUClear::clock::now() + std::chrono::seconds(1), 0, Eigen::Vector3d(x, y, z)));
            });


            on<Trigger<Sensors>, With<KinematicsModel>, With<FootTarget>>().then(
                [this](const Sensors& sensors, const KinematicsModel& model, const FootTarget& target) {

                    // Get support foot coordinate system

                    Eigen::Affine3d Htf_s;  // support foot
                    Eigen::Affine3d Htf_w;  // swing foot

                    // Right foot is the swing foot
                    if (target.isRightFootSwing) {
                        // Transform of left foot to torso
                        Htf_s = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                        // Transform of right foot to torso
                        Htf_w = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                    }
                    // Left foot is the swing foot
                    else {
                        // Transform of right foot to torso
                        Htf_s = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                        // Transform of left foot to torso
                        Htf_w = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                    }

                    // Get orientation for world (Rotation of world->torso)
                    Eigen::Affine3d Rtw;
                    Rtw.linear() = Eigen::Affine3d(sensors.world).rotation();

                    // Get the yawless world rotation
                    // TODO: Euler angle decomposition may not be in the correct order?
                    Eigen::Affine3d Rtg =
                        Rtw * Eigen::AngleAxisd(-Rtw.rotation().eulerAngles(2, 1, 0).x(), Eigen::Vector3d::UnitZ());

                    // Apply the foot yaw to the yawless world rotation
                    // TODO: Euler angle decomposition may not be in the correct order?
                    Rtg = Rtg * Eigen::AngleAxisd(Htf_s.rotation().eulerAngles(2, 1, 0).x(), Eigen::Vector3d::UnitZ());

                    // Construct a torso to foot ground space (support foot centric world oriented space)
                    Eigen::Affine3d Htg;
                    Htg.linear()      = Rtg.linear();          // Rotation from Rtg
                    Htg.translation() = -Htf_s.translation();  // Translation is the same as to the support foot

                    // Vector to the swing foot in ground space
                    Eigen::Vector3d rF_wGg = Htg.inverse() * Htf_w.translation();
                    // The target position is already measured in ground space
                    Eigen::Vector3d rAGg = target.position;

                    // Direction of the target from the swing foot
                    Eigen::Vector3d rAF_wg = rAGg - rF_wGg;
                    // Create a rotation to the plane that cuts through the two positions
                    Eigen::Matrix3d Rgp;
                    // X axis is the direction towards the target
                    Rgp.leftCols<1>() = rAF_wg.normalized();
                    // Y axis is straight up
                    Rgp.middleCols<1>(1) = Eigen::Vector3d::UnitZ();
                    // Z axis is the cross product
                    Rgp.rightCols<1>() = Rgp.leftCols<1>().cross(Rgp.middleCols<1>(1)).normalized();
                    // Fix X axis
                    Rgp.leftCols<1>() = Rgp.middleCols<1>(1).cross(Rgp.rightCols<1>());

                    // Upgrade this to a transform
                    Eigen::Affine3d Hgp;       // plane to ground transform
                    Hgp.linear()      = Rgp;   // Rotation from above
                    Hgp.translation() = rAGg;  // Translation to target

                    // Eigen::Vector3d p1 = Hgp.inverse() * rF_wGg;
                    // Eigen::Vector3d p2 = Hgp.inverse() * rAGg;

                    // Make a transformation matrix that goes the whole way
                    Eigen::Affine3d Htp = Htg * Hgp;

                    // Swing foots position on plane
                    Eigen::Vector3d rF_wPp = Hgp.inverse() * rF_wGg;

                    // Swing foots new target position on the plane
                    Eigen::Vector3d rF_tPp = rF_wPp + Eigen::Vector3d(f_x(rF_wPp), f_y(rF_wPp), 0).normalized() * 0.001;

                    auto g = Eigen::Vector3d(f_x(rF_wPp), f_y(rF_wPp), 0).normalized() * 1000;
                    emit(graph("FootMove", g.x(), g.y()));

                    Eigen::Vector3d rF_tTt = Htp * rF_tPp;

                    Eigen::Affine3d Htf_t;
                    Htf_t.linear()      = Eigen::Matrix3d::Identity();
                    Htf_t.translation() = rF_tTt;
                    // Htf_t.translation().z() += 0.01;
                    // Apply IK
                    /*log(rF_wGg.transpose(),
                        "\n",
                        rAGg.transpose(),
                        "\n",
                        rAF_wg.transpose(),
                        "\n",
                        rF_wPp.transpose(),
                        "\n",
                        rF_tPp.transpose(),
                        "\n",
                        rF_tTt.transpose());*/
                    Transform3D t = convert<double, 4, 4>(Htf_t.matrix());
                    auto joints =
                        calculateLegJoints(model, t, target.isRightFootSwing ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG);

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
                               "FootStep",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
