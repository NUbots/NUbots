#include "FilterTester.hpp"

#include <filesystem>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/math/matrix/transform.hpp"
#include "utility/nbs/Decoder.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/ProgressBar.hpp"

namespace module::tools {

    using extension::Configuration;
    using message::input::Sensors;
    using message::platform::RawSensors;
    using NUClear::message::CommandLineArguments;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::math::transform::error;
    using utility::nusight::graph;

    FilterTester::FilterTester(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("FilterTester.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FilterTester.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // This is the information that will come back from the sensor filter
        on<Trigger<Sensors>, Sync<FilterTester>>().then([this](const Sensors& sensors) {
            Eigen::Isometry3d Hwt              = Eigen::Isometry3d(sensors.Htw).inverse();
            Eigen::Isometry3d Hwt_ground_truth = Eigen::Isometry3d(sensors.Htw_ground_truth).inverse();
            total_error += error<double>(Hwt, Hwt_ground_truth).squaredNorm();
            log<NUClear::INFO>("Total Error: {}", total_error);
        });

        on<Trigger<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            std::vector<std::filesystem::path> paths(std::next(args.begin()), args.end());

            log<NUClear::INFO>("Opening NBS files: ");
            for (const auto& a : paths) {
                log<NUClear::INFO>(a.string());
            }

            utility::support::ProgressBar index_progress("B");
            utility::nbs::Decoder decoder(
                paths,
                [&](const std::filesystem::path& path, const uint64_t& current, const uint64_t& total) {
                    index_progress.update(current, total, fmt::format("Building index for file {}", path.string()));
                });

            decoder.on<RawSensors>([&](const RawSensors& msg) {
                // Emit with direct scope so we can be sure that only one message will process at a time
                emit<Scope::DIRECT>(std::make_unique<RawSensors>(msg));
            });

            utility::support::ProgressBar progress("B");
            decoder.process([&progress](const uint64_t& /*current_message*/,
                                        const uint64_t& /*total_message*/,
                                        const uint64_t& current_bytes,
                                        const uint64_t& total_bytes) {
                progress.update(current_bytes, total_bytes, "Running Filter");
            });
        });
    }

}  // namespace module::tools
