#include "ObstacleDetector.h"

#include "extension/Configuration.h"

#include "message/support/FieldDescription.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/ClassifiedImage.h"

namespace module {
namespace vision {

    using extension::Configuration;
    using message::support::FieldDescription;
    using message::input::CameraParameters;
    using message::vision::ClassifiedImage;

    using utility::nubugger::drawVisionLines;

    ObstacleDetector::ObstacleDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("ObstacleDetector.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ObstacleDetector.yaml
            kmeansClusterer.configure(config["clustering"]);
        });

        // Our segments that may be a part of a goal

        on<Trigger<ClassifiedImage>, With<CameraParameters>, With<FieldDescription>, Single>().then(
            "Obstacle Detector",
            [this](const ClassifiedImage classImage, const CameraParameters& cam, const FieldDescription& fd) {

                // Get points
                int chunk_size   = 50;
                arma::mat points = arma::zeros(2, chunk_size);
                int i            = 0;
                // Get our obstacle segments
                for (const auto& segment : classImage.horizontalSegments) {
                    arma::vec2 start = arma::conv_to<double>::from(convert<int, 2>(segment.start));
                    arma::vec2 end   = arma::conv_to<double>::from(convert<int, 2>(segment.end));
                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    bool colourMatches = segment.segmentClass == SegmentClass::UNKNOWN
                                         || segment.segmentClass == SegmentClass::CYAN_TEAM
                                         || segment.segmentClass == SegmentClass::MAGENTA_TEAM;
                    bool belowVisualHorizon = start > utility::vision::visualHorizonAtPoint(classifiedImage, start[0])
                                              || end > utility::vision::visualHorizonAtPoint(classifiedImage, end[0]);
                    if (belowVisualHorizon && colourMatches && (segment.subsample == 1) && (segment.previous > -1)
                        && (segment.next > -1)) {
                        points.col(i++) = points.col(i++) =
                    }
                    if (i >= points.n_cols) {
                        points = arma::join_cols(points, arma::zeros(2, chunk_size));
                    }
                }

                // Get our obstacle segments
                for (const auto& segment : classImage.verticalSegments) {
                    arma::vec2 start = arma::conv_to<double>::from(convert<int, 2>(segment.start));
                    arma::vec2 end   = arma::conv_to<double>::from(convert<int, 2>(segment.end));
                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    bool colourMatches = segment.segmentClass == SegmentClass::UNKNOWN
                                         || segment.segmentClass == SegmentClass::CYAN_TEAM
                                         || segment.segmentClass == SegmentClass::MAGENTA_TEAM;
                    bool belowVisualHorizon = start > utility::vision::visualHorizonAtPoint(classifiedImage, start[0])
                                              || end > utility::vision::visualHorizonAtPoint(classifiedImage, end[0]);
                    if (belowVisualHorizon && colourMatches && (segment.subsample == 1) && (segment.previous > -1)
                        && (segment.next > -1)) {
                        points.col(i++) = points.col(i++) =
                    }
                    if (i >= points.n_cols) {
                        points = arma::join_cols(points, arma::zeros(2, chunk_size));
                    }
                }

                // Kmeans
                kmeansClusterer.learn(points);
                std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> edges =
                    kmeansClusterer.getDebugRectangles();


                std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> debug;
                for (auto& p : edges) {
                    debug.push_back(convert<double, 2>(p.first), convert<double, 2>(p.second));
                }
                emit(drawVisionLines(debug));

                // TODO: something with this info

            });
    }  // namespace vision
}  // namespace vision
}  // namespace module
