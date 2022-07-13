#include "DeadReckonLocalisation.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/SimpleBall.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/nusight/DataPoint.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/GoalPair.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using SimpleBall  = message::localisation::SimpleBall;
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;

    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::GoalPair;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;
    using utility::support::Expression;

    DeadReckonLocalisation::DeadReckonLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        using message::localisation::SimpleBall;
        using utility::math::coordinates::reciprocalSphericalToCartesian;

        on<Configuration>("DeadReckonLocalisation.yaml")
            .then("DeadReckonLocalisation Config", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();

                // Set our initial estimate for theta
                theta                = config["initial_theta"].as<float>();
                filtered_theta       = config["initial_theta"].as<float>();
                cfg.smoothing_factor = config["smoothing_factor"].as<float>();
            });

        /* Runs on sensors updates */
        // on<Last<10, Trigger<GoalPair>, With<Image>, With<Sensors>, With<FieldDescription>>>().then(
        //     [this](const std::vector<std::shared_ptr<const GoalPair>>& goal_pairs,
        //            const std::vector<std::shared_ptr<const Image>>& images,
        //            const std::vector<std::shared_ptr<const Sensors>>& sensors,
        //            const std::vector<std::shared_ptr<const FieldDescription>>& field_description) {
        //         /******************************** Goal Based Measurement ********************************/
        //         float goal_theta_estimate = 0;
        //         Eigen::Vector3f rGrCc     = Eigen::Vector3f::Zero();
        //         Eigen::Vector3f rGlCC     = Eigen::Vector3f::Zero();


        //         if (goal_pairs[0]) {
        //             for (int i = 0; i < goal_pairs.size(); i++) {

        //                 // Get the transform from camera {c} to torso space {t}
        //                 Eigen::Affine3f Htc(sensors[i]->Htw.cast<float>() * images[i]->Hcw.inverse().cast<float>());

        //                 // Get left and right goal post positions in torso space {t}
        //                 rGrCc += goal_pairs[i]->rGrCc;
        //                 rGlCC += goal_pairs[i]->rGlCc;
        //             }
        //             // Average theta estimate
        //             rGrCc /= goal_pairs.size();
        //             rGlCc /= goal_pairs.size();


        //             // Goal width
        //             float goal_width = field_description[0]->dimensions.goal_width;

        //             rGrCc                 = Eigen::Vector3f(goal_width / 2, 4.5, 0);
        //             rGlCc                 = Eigen::Vector3f(-goal_width / 2, 4.5, 0);
        //             Eigen::Vector3f rGcCc = 0.5 * (rGrCc + rGlCc);

        //             // Distance to right goal in 2d plane
        //             float distance_r = rGrCc.head(2).norm();
        //             // DIstance to left goal in 2d plane
        //             float distance_l = rGlCc.head(2).norm();
        //             // Distance to centre of goal
        //             float distance_c = rGcCc.head(2).norm();

        //             float A = std::acos((distance_r * distance_r + goal_width * goal_width - distance_l * distance_l)
        //                                 / (2 * distance_r * goal_width));

        //             float B = std::acos((distance_l * distance_l + goal_width * goal_width - distance_r * distance_r)
        //                                 / (2 * distance_l * goal_width));

        //             log<NUClear::DEBUG>("A: ", A);
        //             log<NUClear::DEBUG>("B: ", B);

        //             if (!std::isnan(A)) {


        //                 // Get theta from the goal post positions
        //                 rGrCc.z() = 0;
        //                 rGlCc.z() = 0;

        //                 float angle_l = std::acos((Eigen::Vector3f::UnitX().dot(rGlCc) / (rGlCc.norm())));
        //                 float angle_r = std::acos((Eigen::Vector3f::UnitX().dot(rGrCc) / (rGrCc.norm())));

        //                 float alpha = M_PI - M_PI_2 - A;
        //                 float beta  = M_PI - M_PI_2 - B;

        //                 float C = M_PI - A - B;

        //                 log<NUClear::DEBUG>("angle_l: ", angle_l);
        //                 log<NUClear::DEBUG>("angle_r: ", angle_r);
        //                 log<NUClear::DEBUG>("alpha: ", alpha);
        //                 log<NUClear::DEBUG>("beta: ", beta);
        //                 log<NUClear::DEBUG>("alpha  + beta: ", alpha + beta);

        //                 log<NUClear::DEBUG>("C: ", C);
        //                 log<NUClear::DEBUG>("A+B+C: ", A + B + C);

        //                 goal_theta_estimate = std::acos((Eigen::Vector3f::UnitX().dot(rGcCc) / (rGlCc.norm())));
        //                 log<NUClear::DEBUG>("Goal Theta Estimate: ", goal_theta_estimate);
        //                 log<NUClear::DEBUG>("rGrCc ", rGrCc.transpose());
        //                 log<NUClear::DEBUG>("rGlCc ", rGlCc.transpose());
        //                 emit(graph("goal_theta_estimate: ", goal_theta_estimate));
        //             }
        //             // If we have a single set of goals and it is currently within correct 180 degree region of our
        //             // estimated theta, run an opponent goal based update, assuming we can't see 2 sets of goal posts
        //             at
        //             // a time, if we can ignore
        //         }
        //     });

        on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
            // Dead reckon integral of gyro reading and kinematics based dtheta

            /******************************** Kinematics Based Measurement ********************************/

            float kinematics_theta_estimate = 0;

            /******************************** Gyro Based Measurement ********************************/
            emit(graph("Gyro (x,y,z): ", sensors.gyroscope.x(), sensors.gyroscope.y(), sensors.gyroscope.z()));
            // Get the current time
            NUClear::clock::time_point current_time = NUClear::clock::now();
            // Get the delta time
            float dt =
                std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - last_update).count();
            // Log dt
            log<NUClear::DEBUG>("dt: ", dt);
            float gyro_dtheta = dt * sensors.gyroscope.z();

            theta += gyro_dtheta;
            if (theta > 2 * M_PI) {
                theta -= 2 * M_PI;
            }
            else if (theta < -2 * M_PI) {
                theta += 2 * M_PI;
            }
            filtered_theta = cfg.smoothing_factor * theta + (1 - cfg.smoothing_factor) * filtered_theta;

            // Store the current timestamp for next time update
            last_update = current_time;

            emit(graph("deltaT: ", dt));
            emit(graph("Theta: ", theta));
            emit(graph("filtered_theta: ", filtered_theta));
        });
    }

}  // namespace module::localisation
