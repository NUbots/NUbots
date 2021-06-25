#include "BallLocalisation.hpp"

#include <chrono>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/ResetBallHypotheses.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::ResetBallHypotheses;
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

        on<Configuration, Sync<BallLocalisation>>("BallLocalisation.yaml").then([this](const Configuration& cfg) {
            auto message = std::make_unique<std::vector<Ball>>();
            emit(message);
            emit(std::make_unique<Ball>());

            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            ball_pos_log = cfg["ball_pos_log"].as<bool>();
            // Use configuration here from file RobotParticleLocalisation.yaml
            filter.model.n_rogues    = cfg["n_rogues"].as<int>();
            filter.model.resetRange  = cfg["reset_range"].as<Expression>();
            filter.model.n_particles = cfg["n_particles"].as<int>();

            config.start_variance = cfg["start_variance"].as<Expression>();
        });

        on<Startup, With<FieldDescription>>().then([this](const FieldDescription& fd) {
            // Left side penalty mark
            config.start_state.emplace_back((fd.dimensions.field_width / 2.0),
                                            (fd.dimensions.field_length / 2.0) + fd.dimensions.penalty_mark_distance);

            // Right side penalty mark
            config.start_state.emplace_back((fd.dimensions.field_width / 2.0),
                                            -(fd.dimensions.field_length / 2.0) + fd.dimensions.penalty_mark_distance);

            // Infront of our feet (Penalty shootout)
            config.start_state.emplace_back(0.2, 0);

            filter.set_state(config.start_state,
                             std::vector<Eigen::Vector2d>(config.start_state.size(), config.start_variance));
        });

        /* Run Time Update */
        on<Every<15, Per<std::chrono::seconds>>, Sync<BallLocalisation>>().then("BallLocalisation Time", [this] {
            /* Perform time update */
            using namespace std::chrono;
            const auto curr_time  = NUClear::clock::now();
            const double seconds  = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
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
                // Call Time Update first
                using namespace std::chrono;
                const auto curr_time  = NUClear::clock::now();
                const double seconds  = duration_cast<duration<double>>(curr_time - last_time_update_time).count();
                last_time_update_time = curr_time;
                filter.time(seconds);
                for (const auto& ball : balls.balls) {
                    if (!ball.measurements.empty()) {

                        // Now call Measurement Update. Supports multiple measurement methods and will treat them as
                        // separate measurements
                        for (auto& measurement : ball.measurements) {
                            filter.measure(Eigen::Vector3d(measurement.srBCc.cast<double>()),
                                           Eigen::Matrix3d(measurement.covariance.cast<double>()),
                                           field,
                                           balls.Hcw);
                        }
                        last_measurement_update_time = curr_time;
                    }
                }
            });

        on<Trigger<ResetBallHypotheses>, With<Sensors>, With<Field>, With<FieldDescription>, Sync<BallLocalisation>>()
            .then("Reset Ball Hypotheses",
                  [this](const ResetBallHypotheses& locReset,
                         const Sensors& sensors,
                         const Field& field,
                         const FieldDescription& fd) {
                      if (locReset.self_reset) {
                          filter.set_state(
                              config.start_state,
                              std::vector<Eigen::Vector2d>(config.start_state.size(), config.start_variance));
                      }
                      else {

                          // Set the filter state to the field origin relative to us
                          filter.set_state(Eigen::Affine2d(field.position).translation(),
                                           config.start_variance.asDiagonal());
                      }
                  });
    }
}  // namespace module::localisation
