#ifndef MODULE_VISION_FIELDLINEDETECTOR_H
#define MODULE_VISION_FIELDLINEDETECTOR_H

#include <nuclear>

namespace module {
namespace vision {

    class FieldLineDetector : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the FieldLineDetector reactor.
        explicit FieldLineDetector(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            uint min_points_for_consensus;
            uint max_iterations_per_fitting;
            uint max_fitted_models;
            double consensus_error_threshold;
            double max_angle_difference;
            double max_line_distance;
        } config;
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_FIELDLINEDETECTOR_H
