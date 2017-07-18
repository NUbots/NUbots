#include "ObstacleDetector.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
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
    using SegmentClass = message::vision::ClassifiedImage::SegmentClass::Value;

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
            [this](const ClassifiedImage classifiedImage, const CameraParameters& cam, const FieldDescription& fd) {

                // Get points
                int chunk_size   = 50;  // Chunk size must be divisible by two
                arma::mat points = arma::zeros(2, chunk_size);
                int i            = 0;
                // Get our obstacle segments
                for (const auto& segment : classifiedImage.horizontalSegments) {
                    arma::vec2 start = arma::conv_to<arma::vec>::from(convert<int, 2>(segment.start));
                    arma::vec2 end   = arma::conv_to<arma::vec>::from(convert<int, 2>(segment.end));
                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    bool colourMatches = segment.segmentClass == SegmentClass::UNKNOWN_CLASS
                                         || segment.segmentClass == SegmentClass::CYAN_TEAM
                                         || segment.segmentClass == SegmentClass::MAGENTA_TEAM;
                    bool belowVisualHorizon =
                        start[1] > utility::vision::visualHorizonAtPoint(classifiedImage, start[0])
                        || end[1] > utility::vision::visualHorizonAtPoint(classifiedImage, end[0]);
                    if (belowVisualHorizon && colourMatches && (segment.subsample == 1) && (segment.previous > -1)
                        && (segment.next > -1)) {
                        points.col(i++) = start;
                        points.col(i++) = end;
                    }
                    if (i >= points.n_cols) {
                        points = arma::join_cols(points, arma::zeros(2, chunk_size));
                    }
                }

                // Get our obstacle segments
                for (const auto& segment : classifiedImage.verticalSegments) {
                    arma::vec2 start = arma::conv_to<arma::vec>::from(convert<int, 2>(segment.start));
                    arma::vec2 end   = arma::conv_to<arma::vec>::from(convert<int, 2>(segment.end));
                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    bool colourMatches = segment.segmentClass == SegmentClass::UNKNOWN_CLASS
                                         || segment.segmentClass == SegmentClass::CYAN_TEAM
                                         || segment.segmentClass == SegmentClass::MAGENTA_TEAM;
                    bool belowVisualHorizon =
                        start[1] > utility::vision::visualHorizonAtPoint(classifiedImage, start[0])
                        || end[1] > utility::vision::visualHorizonAtPoint(classifiedImage, end[0]);
                    if (belowVisualHorizon && colourMatches && (segment.subsample == 1) && (segment.previous > -1)
                        && (segment.next > -1)) {
                        points.col(i++) = start;
                        points.col(i++) = end;
                    }
                    if (i >= points.n_cols) {
                        points = arma::join_cols(points, arma::zeros(2, chunk_size));
                    }
                }

                // Kmeans
                kmeansClusterer.learn(points);

                emit(drawVisionLines(kmeansClusterer.getDebugRectangles()));

                // TODO: something with this info

            });
    }
}  // namespace vision
}  // namespace module
