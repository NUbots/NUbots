#include "BallLocalisation.h"

#include <chrono>

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/Ball.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Ball.h"
#include "utility/input/ServoID.h"
#include "utility/math/coordinates.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::support::FieldDescription;

    using utility::math::coordinates::cartesianToSpherical;
    using ServoID = utility::input::ServoID;
    using utility::nusight::graph;
    using utility::support::Expression;

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
            filter.model.processNoiseDiagonal = config["process_noise_diagonal"].as<Expression>();
            filter.model.n_rogues             = config["n_rogues"].as<int>();
            filter.model.resetRange           = config["reset_range"].as<Expression>();
            int n_particles                   = config["n_particles"].as<int>();

            Eigen::Vector2d start_state    = config["start_state"].as<Expression>();
            Eigen::Vector2d start_variance = config["start_variance"].as<Expression>();

            filter.set_state(start_state, Eigen::DiagonalMatrix<double, 2, 2>{start_variance}, n_particles);

            // Use configuration here from file BallLocalisation.yaml
        });

        /* Run Time Update */
        on<Every<15, Per<std::chrono::seconds>>, Sync<BallLocalisation>, With<FieldDescription>, With<Sensors>>().then(
            "BallLocalisation Time", [this](const FieldDescription& field, const Sensors& sensors) {
                /* Perform time update */
                using namespace std::chrono;
                auto curr_time        = NUClear::clock::now();
                double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                last_time_update_time = curr_time;
                filter.time(seconds);

                /* Creating ball state vector and covariance matrix for emission */
                auto ball        = std::make_unique<Ball>();
                ball->position   = filter.get();
                ball->covariance = filter.getCovariance();

                if (ball_pos_log) {
                    emit(graph("localisation ball pos", filter.get()[0], filter.get()[1]));
                    log("localisation ball pos = ", filter.get()[0], filter.get()[1]);
                    log("localisation seconds elapsed = ", seconds);
                }
                emit(ball);
            });

        /* To run whenever a ball has been detected */
        on<Trigger<message::vision::Balls>, With<FieldDescription>>().then(
            [this](const message::vision::Balls& balls, const FieldDescription& field) {
                if (balls.balls.size() > 0) {
                    /* Call Time Update first */
                    using namespace std::chrono;
                    auto curr_time        = NUClear::clock::now();
                    double seconds        = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                    last_time_update_time = curr_time;
                    filter.time(seconds);

                    /* Now call Measurement Update. Supports multiple measurement methods
                     * and will treat them as
                     * separate measurements */
                    for (auto& measurement : balls.balls[0].measurements) {
                        /* These parameters must be cast because Eigen doesn't do implicit conversion of float to
                           double. They must also be wrapped, because Eigen converts to an intermediate type after the
                           cast and that intermediate type screws up the function call */
                        filter.measure(Eigen::VectorXd{measurement.rBCc.cast<double>()},
                                       Eigen::MatrixXd{measurement.covariance.cast<double>()},
                                       field,
                                       balls.Hcw);
                    }
                    last_measurement_update_time = curr_time;
                }
            });
    }
}  // namespace localisation
}  // namespace module
