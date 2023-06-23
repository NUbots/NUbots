#include "Benchmarking.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>

#include "extension/Configuration.hpp"

#include "message/vision/GoalPost.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/vision/Vision.hpp"

namespace module::vision {

    using extension::Configuration;

    using message::vision::GoalPost;
    using utility::nusight::graph;


    Benchmarking::Benchmarking(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Benchmarking.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Benchmarking.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<GoalPost>>().then("Benchmarking", [this](const GoalPost& post) {
            // Convenience variables
            const int id                = post.id;
            const Eigen::Isometry3f Hcw = Eigen::Isometry3f(post.Hcw.cast<float>());

            std::vector<Eigen::Vector3f> points;
            std::vector<Eigen::Vector3f> rPWw;
            int i = 0;

            /*
            for (auto point : post.points) {

                i++;
                // points.insert(point);
                // Eigen::Vector3f p = point;
                points.push_back(point);

                const float mag = sqrt((point.x() * point.x()) + (point.y() * point.y()));

                log<NUClear::DEBUG>(fmt::format("Point {}: x:{} y:{} mag:{}", i, point.x(), point.y(), mag));
                emit(graph("Goal Point", point.x(), point.y(), mag));
            }
            i = 0; */
            for (auto p : post.rPWw) {

                i++;

                const float mag = sqrt((p.x() * p.x()) + (p.y() * p.y()));

                log<NUClear::DEBUG>(fmt::format("Point {}: x:{} y:{} mag:{}", i, p.x(), p.y(), mag));
                emit(graph("Goal rPWw", p.x(), p.y(), mag));
            }

            i = 0;

            for (auto p : post.raw_best) {

                i++;

                const float best_mag = sqrt((p.x() * p.x()) + (p.y() * p.y()));

                log<NUClear::DEBUG>(fmt::format("Cluster {} Best: x:{} y:{} mag:{}", i, p.x(), p.y(), best_mag));
                emit(graph("Raw Best", p.x(), p.y(), best_mag));
            }

            i = 0;

            for (auto p : post.normal_best) {

                i++;

                const float best_mag = sqrt((p.x() * p.x()) + (p.y() * p.y()));

                log<NUClear::DEBUG>(fmt::format("Cluster {} Normal Best: x:{} y:{}", i, p.x(), p.y()));
                emit(graph("Normal Best", p.x(), p.y()));
            }

            // log<NUClear::DEBUG>(
            //     fmt::format("Raw Best: x:{} y:{} mag:{}", post.raw_best.x(), post.raw_best.y(), best_mag));
            // emit(graph("Raw Best", post.raw_best.x(), post.raw_best.y(), best_mag));

            // log<NUClear::DEBUG>(fmt::format("Normal Best: x:{} y:{}", post.normal_best.x(), post.normal_best.y()));
            // emit(graph("Normal Best", post.normal_best.x(), post.normal_best.y()));
        });
    }

}  // namespace module::vision
