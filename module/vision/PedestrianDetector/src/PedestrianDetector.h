#ifndef MODULE_VISION_PEDESTRIANDETECTOR_H
#define MODULE_VISION_PEDESTRIANDETECTOR_H

#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <nuclear>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "message/vision/BakedImage.h"

namespace module {
namespace vision {

    class PedestrianDetector : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PedestrianDetector reactor.
        explicit PedestrianDetector(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool dump_images;
        std::chrono::duration<double, std::milli> avg_fp_ms;
        size_t avg_count;

        std::map<int, std::string> category_index;
        std::string graph_file;
        std::string checkpoint_path;

        std::unique_ptr<tensorflow::Session> session;
        tensorflow::GraphDef graph_def;
        tensorflow::Tensor checkpointPathTensor;

        bool loadLabelMap(const std::string& label_file);
        void visualizeBoxesAndLabelsOnImageArray(message::vision::BakedImage& image,
                                                 const std::vector<std::array<float, 4>>& boxes,
                                                 const std::vector<int>& classes,
                                                 const std::vector<float>& scores,
                                                 bool use_normalized_coordinates = false,
                                                 size_t max_boxes_to_draw        = 20,
                                                 float min_score_thresh          = 0.5f,
                                                 bool agnostic_mode              = false,
                                                 int line_thickness              = 4);
        void drawBoundingBoxOnImage(message::vision::BakedImage& image,
                                    const std::array<float, 4>& box,
                                    const cv::Scalar& color                          = cv::Scalar(255, 0, 0),
                                    int thickness                                    = 4,
                                    const std::vector<std::string>& display_str_list = {},
                                    bool use_normalized_coordinates                  = true);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_PEDESTRIANDETECTOR_H
