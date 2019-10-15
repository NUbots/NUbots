#include "PFTest.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "extension/Configuration.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace tools {

    struct MeasurementFinished {};

    using extension::Configuration;

    using utility::support::Expression;

    PFTest::PFTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), time_count(0) {

        on<Configuration>("PFTest.yaml").then([this](const Configuration& config) {
            model_filter.model.process_noise = config["process_noise"].as<Expression>();
            measurement_noise.x()            = config["measurement_noise"].as<double>();
            deltaT                           = config["deltaT"].as<double>();

            for (const auto& data : config["true_state"].config) {
                true_state.push_back(data.as<Expression>());
            }
            for (const auto& data : config["measurements"].config) {
                measurements.push_back(data.as<Expression>());
            }

            model_filter.set_state(config["initial_state"].as<Expression>(),
                                   config["initial_covariance"].as<Expression>(),
                                   config["num_particles"].as<int>());
        });

        on<Startup>().then([this] { filter_handle.enable(); });

        filter_handle =
            on<Every<20, Per<std::chrono::seconds>>>()
                .then([this] {
                    model_filter.measure(Eigen::Matrix<double, 1, 1>(measurements[time_count]), measurement_noise);
                    model_filter.time(deltaT);
                    innovations.push_back(measurements[time_count] - model_filter.get().x());
                    actual_state.push_back(std::make_pair(model_filter.get(), model_filter.getCovariance()));
                    time_count++;

                    if (time_count == measurements.size()) {
                        filter_handle.disable();
                        emit(std::make_unique<MeasurementFinished>());
                    }
                })
                .disable();

        on<Trigger<MeasurementFinished>>().then([this] {
            double count_x1                  = 0.0;
            double count_x2                  = 0.0;
            double mean_innovations          = 0.0;
            double mean_x1_boundary          = 0.0;
            double mean_x2_boundary          = 0.0;
            Eigen::Vector2d mean_state_error = Eigen::Vector2d::Zero();
            for (size_t i = 0; i < actual_state.size(); ++i) {
                Eigen::Vector2d state_error = true_state[i] - actual_state[i].first;
                double covariance_bounds_x1 = std::sqrt(actual_state[i].second(0, 0));
                double covariance_bounds_x2 = std::sqrt(actual_state[i].second(1, 1));
                mean_x1_boundary += covariance_bounds_x1;
                mean_x2_boundary += covariance_bounds_x2;

                count_x1 += (std::abs(state_error.x()) - covariance_bounds_x1) > 0.0 ? 1.0 : 0.0;
                count_x2 += (std::abs(state_error.y()) - covariance_bounds_x2) > 0.0 ? 1.0 : 0.0;

                mean_innovations += innovations[i];
                mean_state_error += state_error;
            }

            mean_innovations /= innovations.size();
            mean_state_error /= actual_state.size();
            mean_x1_boundary /= actual_state.size();
            mean_x2_boundary /= actual_state.size();

            double percentage_x1 = 100.0 * count_x1 / actual_state.size();
            double percentage_x2 = 100.0 * count_x2 / actual_state.size();

            NUClear::log<NUClear::INFO>(
                fmt::format("The mean of the innovations is: {}. This should be small.", mean_innovations));
            NUClear::log<NUClear::INFO>(fmt::format("The mean of the state errors is: {}. This should be small.",
                                                    mean_state_error.transpose()));
            NUClear::log<NUClear::INFO>(
                fmt::format("{}% of state 1 estimates exceed the 1\u03C3 boundary", percentage_x1));
            NUClear::log<NUClear::INFO>(
                fmt::format("{}% of state 2 estimates exceed the 1\u03C3 boundary", percentage_x2));
            NUClear::log<NUClear::INFO>(
                fmt::format("The mean 1\u03C3 boundary for state 1 is [{}, {}]", -mean_x1_boundary, mean_x1_boundary));
            NUClear::log<NUClear::INFO>(
                fmt::format("The mean 1\u03C3 boundary for state 2 is [{}, {}]", -mean_x2_boundary, mean_x2_boundary));
            if (percentage_x1 < 30.0) {
                NUClear::log<NUClear::INFO>(fmt::format(
                    "Less than 30% of state 1 estimates exceed the 1\u03C3 boundary. This is good estimation."));
            }
            else {
                NUClear::log<NUClear::INFO>(
                    fmt::format("More than 30% of state 1 estimates exceed the 1\u03C3 boundary."));
            }

            powerplant.shutdown();
        });
    }
}  // namespace tools
}  // namespace module
