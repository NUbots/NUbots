#include "BallLocalisation.h"
#include <chrono>
#include "extension/Configuration.h"
#include "utility/time/time.h"

#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisionObjects.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using utility::time::TimeDifferenceSeconds;
    using message::localisation::Ball;
    using message::support::FieldDescription;
    using message::input::Sensors;

    using utility::math::matrix::Transform3D;
    using utility::math::coordinates::cartesianToSpherical;
    using ServoID = utility::input::ServoID;
    using utility::nubugger::graph;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), filter() {

        on<Startup>().then([this] {
            last_measurement_update_time = NUClear::clock::now();
            last_time_update_time        = NUClear::clock::now();
        });

        on<Configuration, Sync<BallLocalisation>>("BallLocalisation.yaml").then([this](const Configuration& config) {
            auto message = std::make_unique<std::vector<Ball>>();
            emit(message);
            emit(std::make_unique<Ball>());

            ball_pos_log = config["ball_pos_log"].as<bool>();
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<arma::vec>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<arma::vec>();
            int n_particles                   = config["n_particles"].as<int>();

            arma::vec2 start_state    = config["start_state"].as<arma::vec>();
            arma::vec2 start_variance = config["start_variance"].as<arma::vec>();

            filter.reset(start_state, arma::diagmat(start_variance), n_particles);

            // Use configuration here from file BallLocalisation.yaml
        });

        /* Run Time Update */
        on<Every<15, Per<std::chrono::seconds>>, Sync<BallLocalisation>, With<FieldDescription>, With<Sensors>>().then(
            "BallLocalisation Time", [this](const FieldDescription& field, const Sensors& sensors) {
                /* Perform time update */
                auto curr_time        = NUClear::clock::now();
                double seconds        = TimeDifferenceSeconds(curr_time, last_time_update_time);
                last_time_update_time = curr_time;
                filter.timeUpdate(seconds);

                /* Creating ball state vector and covariance matrix for emission */
                // std::unique_ptr ball;
                auto ball                             = std::make_unique<Ball>();
                ball->locObject.position              = convert<double, 2>(filter.get());
                ball->locObject.position_cov          = convert<double, 2, 2>(filter.getCovariance());
                ball->locObject.last_measurement_time = last_measurement_update_time;

                /* For graphing purposes */
                arma::vec3 rBWw = {ball->locObject.position[0], ball->locObject.position[1], field.ball_radius};
                // Get our transform to world coordinates
                const Transform3D& Htw = convert<double, 4, 4>(sensors.world);
                const Transform3D& Htc = convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH));
                Transform3D Hcw        = Htc.i() * Htw;
                arma::vec3 rBCc_cart   = Hcw.transformPoint(rBWw);
                arma::vec3 rBCc_sph1   = cartesianToSpherical(rBCc_cart);  // in r,theta,phi
                arma::vec3 rBCc_sph2   = {
                    rBCc_sph1[0], rBCc_sph1[1], rBCc_sph1[2]};  // in roe, theta, phi, where roe is 1/r
                if (ball_pos_log) {
                    emit(graph("localisation ball pos", filter.get()[0], filter.get()[1]));
                    log("localisation ball pos = ", filter.get()[0], filter.get()[1]);
                    log("localisation seconds elapsed = ", seconds);
                }
                emit(ball);
            });

        /* To run whenever a ball has been detected */
        on<Trigger<std::vector<message::vision::Ball>>, With<FieldDescription>>().then(
            [this](const std::vector<message::vision::Ball>& balls, const FieldDescription& field) {

                if (balls.size() > 0) {
                    const auto& sensors = *balls[0].visObject.sensors;

                    /* Call Time Update first */
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = TimeDifferenceSeconds(curr_time, last_time_update_time);
                    last_time_update_time = curr_time;
                    filter.timeUpdate(seconds);

                    /* Now call Measurement Update. Supports multiple measurement methods
                     * and will treat them as
                     * separate measurements */
                    for (auto& measurement : balls[0].measurements) {
                        filter.measurementUpdate(cartesianToSpherical(convert<double, 3, 1>(measurement.rBCc)),
                                                 convert<double, 3, 3>(measurement.covariance),
                                                 field,
                                                 sensors);
                    }
                    last_measurement_update_time = curr_time;
                }
            });
    }
}  // namespace localisation
}  // namespace module
