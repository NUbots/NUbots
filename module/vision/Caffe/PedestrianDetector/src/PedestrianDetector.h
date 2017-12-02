#ifndef MODULE_VISION_CAFFE_PEDESTRIANDETECTOR_H
#define MODULE_VISION_CAFFE_PEDESTRIANDETECTOR_H

#include <Eigen/Core>
#include <caffe/caffe.hpp>
#include <nuclear>
#include <string>
#include <vector>

#include <viennacl/ocl/backend.hpp>
#include <viennacl/ocl/device.hpp>
#include <viennacl/ocl/platform.hpp>

namespace module {
namespace vision {
    namespace Caffe {

        class PedestrianDetector : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the PedestrianDetector reactor.
            explicit PedestrianDetector(std::unique_ptr<NUClear::Environment> environment);

        private:
            bool use_gpu;
            float detection_threshold;
            std::string network_input;
            std::string network_cvg;
            std::string network_boxes;

            std::unique_ptr<caffe::Net<float>> pednet;
            Eigen::Matrix<unsigned int, 3, 1> input_dimensions;
            Eigen::Matrix<unsigned int, 3, 1> cvg_dimensions, boxes_dimensions;

            bool rectOverlap(const std::array<float, 6>& r1, const std::array<float, 6>& r2) const;
            void mergeRect(std::vector<std::array<float, 6>>& rects, const std::array<float, 6>& rect) const;
        };

    }  // namespace Caffe
}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_CAFFE_PEDESTRIANDETECTOR_H
