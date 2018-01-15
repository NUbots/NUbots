#include "PedestrianDetector.h"

#include <fmt/format.h>

#include "extension/Configuration.h"

#include "message/vision/BakedImage.h"
#include "message/vision/ReprojectedImage.h"

#include "utility/math/comparison.h"

namespace module {
namespace vision {
    namespace Caffe {

        using extension::Configuration;

        using message::vision::BakedImage;
        using message::vision::ReprojectedImage;

        PedestrianDetector::PedestrianDetector(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , use_gpu(false)
            , detection_threshold(0.5f)
            , network_input()
            , network_cvg()
            , network_boxes()
            , pednet(nullptr) {

            on<Configuration>("CaffePedestrianDetector.yaml").then([this](const Configuration& config) {
                log(fmt::format("{}:{}", __FILE__, __LINE__));
                use_gpu             = config["use_gpu"].as<bool>();
                detection_threshold = config["detection_threshold"].as<float>();
                network_input       = config["network"]["input_layer"].as<std::string>();
                network_cvg         = config["network"]["output_cvg"].as<std::string>();
                network_boxes       = config["network"]["output_boxes"].as<std::string>();

                // Find OpenCL device.
                std::vector<viennacl::ocl::platform> platforms = viennacl::ocl::get_platforms();
                bool device_found                              = false;
                viennacl::ocl::platform ocl_platform;
                viennacl::ocl::device ocl_device;

                for (auto& platform_iter : platforms) {
                    std::vector<viennacl::ocl::device> devices =
                        platform_iter.devices((use_gpu ? CL_DEVICE_TYPE_GPU : CL_DEVICE_TYPE_CPU));

                    for (auto& device_iter : devices) {
                        if (device_iter.available() && device_iter.compiler_available()) {
                            ocl_platform = platform_iter;
                            ocl_device   = device_iter;
                            device_found = true;
                            break;
                        }
                    }
                }

                if (!device_found) {
                    log<NUClear::ERROR>(fmt::format("Failed to find a {} device.", (use_gpu ? "GPU" : "CPU")));
                    return;
                }

                log<NUClear::INFO>(fmt::format("Using OpenCL platform: {}", ocl_platform.info()));
                log<NUClear::INFO>(fmt::format("Using OpenCL device: {}", ocl_device.name()));

                // Make a context for this device
                viennacl::ocl::setup_context(0, {ocl_device});

                caffe::Caffe::set_mode(use_gpu ? caffe::Caffe::GPU : caffe::Caffe::CPU);

                // Delete old network,
                if (pednet) {
                    pednet.reset(nullptr);
                }

                // Load new network.
                caffe::device dev;
                dev.Init();
                pednet = std::make_unique<caffe::Net<float>>(
                    config["network"]["model"].as<std::string>(), caffe::TEST, &dev);
                pednet->CopyTrainedLayersFrom(config["network"]["weights"].as<std::string>());

                // Verify network inputs and outputs.
                if (pednet->num_inputs() != 1) {
                    log<NUClear::ERROR>(
                        fmt::format("Caffe network should have exactly 1 input. Found {}", pednet->num_inputs()));
                    return;
                }

                if (pednet->num_outputs() < 2) {
                    log<NUClear::ERROR>(
                        fmt::format("Caffe network has {} outputs, expected (atleast) 2", pednet->num_inputs()));
                    return;
                }

                // Get the input and output geometries of the network.
                auto input_blob = pednet->blob_by_name(network_input);
                input_dimensions << input_blob->width(), input_blob->height(), input_blob->channels();

                auto boxes_blob = pednet->blob_by_name(network_boxes);
                auto cvg_blob   = pednet->blob_by_name(network_cvg);
                cvg_dimensions << cvg_blob->width(), cvg_blob->height(), cvg_blob->channels();
                boxes_dimensions << boxes_blob->width(), boxes_blob->height(), boxes_blob->channels();

                // Set up layer sizes.
                input_blob->Reshape(1, input_dimensions.z(), input_dimensions.y(), input_dimensions.x());

                // Forward dimension change to all layers.
                pednet->Reshape();
            });

            on<Trigger<ReprojectedImage>>().then([this](const ReprojectedImage& image) {
                if (pednet) {
                    // Check the dimensions of the input image. Ensure they are consistent with the networks input
                    // layer.
                    if ((input_dimensions.x() != image.dimensions.x())
                        || (input_dimensions.y() != image.dimensions.y())) {

                        log<NUClear::WARN>(
                            fmt::format("Pedestrian detection network requires input images of size {}x{}, but "
                                        "ReprojectedImage images are currently {}x{}. Please correct this",
                                        input_dimensions.x(),
                                        input_dimensions.y(),
                                        image.dimensions.x(),
                                        image.dimensions.y()));

                        return;
                    }

                    // Convert image to floating point (0.0-255.0) BGR image and write it to the input layer of the
                    // network.
                    auto input_blob = pednet->blob_by_name(network_input);
                    auto data       = input_blob->mutable_cpu_data();
                    for (size_t row = 0; row < input_dimensions.y(); row++) {
                        for (size_t col = 0; col < input_dimensions.x(); col++) {
                            for (size_t channel = 0; channel < 3; channel++) {
                                // RGB -> BGR
                                // 012 -> 210
                                const size_t in_index  = (row * input_dimensions.x() + col) * 3 + channel;
                                const size_t out_index = (row * input_dimensions.x() + col) * 3 + (2 - channel);

                                data[out_index] = float(image.data[in_index]);
                            }
                        }
                    }

                    // Process the image through the network.
                    pednet->Forward();

                    // Finally, extract the results from the network.
                    auto cvg_blob    = pednet->blob_by_name(network_cvg);
                    auto bboxes_blob = pednet->blob_by_name(network_boxes);

                    std::vector<float> coverage(cvg_blob->cpu_data(), cvg_blob->cpu_data() + cvg_dimensions.prod());
                    std::vector<float> bboxes(bboxes_blob->cpu_data(),
                                              bboxes_blob->cpu_data() + boxes_dimensions.prod());


                    const size_t ow  = boxes_dimensions.x();  // number of columns in bbox grid in X dimension
                    const size_t oh  = boxes_dimensions.y();  // number of rows in bbox grid in Y dimension
                    const size_t owh = ow * oh;               // total number of bbox in grid
                    const size_t cls = cvg_dimensions.z();    // number of object classes in coverage map

                    const float cell_width  = input_dimensions.x() / ow;
                    const float cell_height = input_dimensions.y() / oh;

                    const float scale_x = float(image.dimensions.x()) / float(input_dimensions.x());
                    const float scale_y = float(image.dimensions.y()) / float(input_dimensions.y());

                    std::vector<std::vector<std::array<float, 6>>> rects;
                    rects.resize(cls);

                    // extract and cluster the raw bounding boxes that meet the coverage threshold
                    for (size_t z = 0; z < cls; z++) {
                        rects[z].reserve(owh);

                        for (size_t y = 0; y < oh; y++) {
                            for (size_t x = 0; x < ow; x++) {
                                const float coverage = cvg_blob->cpu_data()[z * owh + y * ow + x];

                                if (coverage > detection_threshold) {
                                    const float mx = x * cell_width;
                                    const float my = y * cell_height;

                                    const float x1 = (bboxes[0 * owh + y * ow + x] + mx) * scale_x;  // left
                                    const float y1 = (bboxes[1 * owh + y * ow + x] + my) * scale_y;  // top
                                    const float x2 = (bboxes[2 * owh + y * ow + x] + mx) * scale_x;  // right
                                    const float y2 = (bboxes[3 * owh + y * ow + x] + my) * scale_y;  // bottom

                                    mergeRect(rects[z], std::array<float, 6>{x1, y1, x2, y2, coverage, float(z)});
                                }
                            }
                        }
                    }

                    // Draw boxes on image.
                    auto msg           = std::unique_ptr<BakedImage>();
                    msg->format        = image.format;
                    msg->data          = std::vector<uint8_t>(image.data);
                    msg->dimensions    = image.dimensions;
                    msg->camera_id     = image.camera_id;
                    msg->serial_number = "PedestrianDetector";
                    msg->timestamp     = NUClear::clock::now();

                    for (size_t z = 0; z < cls; z++) {
                        const size_t numBox = rects[z].size();

                        for (size_t b = 0; b < numBox; b++) {
                            const std::array<float, 6> r = rects[z][b];

                            for (size_t y = std::max(0.0f, r[1]); y <= std::min(r[3], float(input_dimensions.y()));
                                 y++) {
                                for (size_t x = std::max(0.0f, r[0]); x <= std::min(r[2], float(input_dimensions.x()));
                                     x++) {
                                    const int origin = (y * image.dimensions.x() + x) * 3;
                                    msg->data[origin + 0] =
                                        utility::math::clamp(uint8_t(0),
                                                             uint8_t(msg->data[origin + 0] * 0.75f + 0.25f * 255.0f),
                                                             uint8_t(255));
                                    msg->data[origin + 1] =
                                        utility::math::clamp(uint8_t(0),
                                                             uint8_t(msg->data[origin + 1] * 0.75f + 0.25f * 0.0f),
                                                             uint8_t(255));
                                    msg->data[origin + 2] =
                                        utility::math::clamp(uint8_t(0),
                                                             uint8_t(msg->data[origin + 2] * 0.75f + 0.25f * 0.0f),
                                                             uint8_t(255));
                                }
                            }
                        }
                    }

                    emit(msg);
                }
            });
        }  // namespace Caffe

        bool PedestrianDetector::rectOverlap(const std::array<float, 6>& r1, const std::array<float, 6>& r2) const {
            return !((r2[0] > r1[2]) || (r2[2] < r1[0]) || (r2[1] > r1[3]) || (r2[3] < r1[1]));
        }

        void PedestrianDetector::mergeRect(std::vector<std::array<float, 6>>& rects,
                                           const std::array<float, 6>& rect) const {
            bool intersects = false;

            for (size_t r = 0; r < rects.size(); r++) {
                if (rectOverlap(rects[r], rect)) {
                    intersects = true;

                    if (rect[0] < rects[r][0]) {
                        rects[r][0] = rect[0];
                    }
                    if (rect[1] < rects[r][1]) {
                        rects[r][1] = rect[1];
                    }
                    if (rect[2] > rects[r][2]) {
                        rects[r][2] = rect[2];
                    }
                    if (rect[3] > rects[r][3]) {
                        rects[r][3] = rect[3];
                    }

                    break;
                }
            }

            if (!intersects) {
                rects.push_back(rect);
            }
        }


    }  // namespace Caffe
}  // namespace vision
}  // namespace module
