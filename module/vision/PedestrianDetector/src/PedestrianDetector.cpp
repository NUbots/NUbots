#include "PedestrianDetector.h"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/motion/FixedMotionCommand.h"
#include "message/motion/HeadCommand.h"
#include "message/vision/ReprojectedImage.h"

#include "utility/input/ServoID.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::Sensors;
    using message::motion::HeadCommand;
    using message::motion::WaveRightCommand;
    using message::vision::BakedImage;
    using message::vision::ReprojectedImage;

    using utility::input::ServoID;

    const static std::map<std::string, cv::Scalar> STANDARD_COLORS = {
        {"AliceBlue", cv::Scalar(240, 248, 255)},
        {"AntiqueWhite", cv::Scalar(250, 235, 215)},
        {"Aqua", cv::Scalar(0, 255, 255)},
        {"Aquamarine", cv::Scalar(127, 255, 212)},
        {"Azure", cv::Scalar(240, 255, 255)},
        {"Beige", cv::Scalar(245, 245, 220)},
        {"Bisque", cv::Scalar(255, 228, 196)},
        {"Black", cv::Scalar(0, 0, 0)},
        {"BlanchedAlmond", cv::Scalar(255, 235, 205)},
        {"Blue", cv::Scalar(0, 0, 255)},
        {"BlueViolet", cv::Scalar(138, 43, 226)},
        {"Brown", cv::Scalar(165, 42, 42)},
        {"BurlyWood", cv::Scalar(222, 184, 135)},
        {"CadetBlue", cv::Scalar(95, 158, 160)},
        {"Chartreuse", cv::Scalar(127, 255, 0)},
        {"Chocolate", cv::Scalar(210, 105, 30)},
        {"Coral", cv::Scalar(255, 127, 80)},
        {"CornflowerBlue", cv::Scalar(100, 149, 237)},
        {"Cornsilk", cv::Scalar(255, 248, 220)},
        {"Crimson", cv::Scalar(220, 20, 60)},
        {"Cyan", cv::Scalar(0, 255, 255)},
        {"DarkBlue", cv::Scalar(0, 0, 139)},
        {"DarkCyan", cv::Scalar(0, 139, 139)},
        {"DarkGoldenRod", cv::Scalar(184, 134, 11)},
        {"DarkGreen", cv::Scalar(0, 100, 0)},
        {"DarkGrey", cv::Scalar(169, 169, 169)},
        {"DarkKhaki", cv::Scalar(189, 183, 107)},
        {"DarkMagenta", cv::Scalar(139, 0, 139)},
        {"DarkOliveGreen", cv::Scalar(85, 107, 47)},
        {"DarkOrange", cv::Scalar(255, 140, 0)},
        {"DarkOrchid", cv::Scalar(153, 50, 204)},
        {"DarkRed", cv::Scalar(139, 0, 0)},
        {"DarkSalmon", cv::Scalar(233, 150, 122)},
        {"DarkSeaGreen", cv::Scalar(143, 188, 139)},
        {"DarkSlateBlue", cv::Scalar(72, 61, 139)},
        {"DarkSlateGray", cv::Scalar(47, 79, 79)},
        {"DarkTurquoise", cv::Scalar(0, 206, 209)},
        {"DarkViolet", cv::Scalar(148, 0, 211)},
        {"DeepPink", cv::Scalar(255, 20, 147)},
        {"DeepSkyBlue", cv::Scalar(0, 191, 255)},
        {"DimGray", cv::Scalar(105, 105, 105)},
        {"DodgerBlue", cv::Scalar(30, 144, 255)},
        {"FireBrick", cv::Scalar(178, 34, 34)},
        {"FloralWhite", cv::Scalar(255, 250, 240)},
        {"ForestGreen", cv::Scalar(34, 139, 34)},
        {"Fuchsia", cv::Scalar(255, 0, 255)},
        {"Gainsboro", cv::Scalar(220, 220, 220)},
        {"GhostWhite", cv::Scalar(248, 248, 255)},
        {"Gold", cv::Scalar(255, 215, 0)},
        {"GoldenRod", cv::Scalar(218, 165, 32)},
        {"Gray", cv::Scalar(128, 128, 128)},
        {"Green", cv::Scalar(0, 128, 0)},
        {"GreenYellow", cv::Scalar(173, 255, 47)},
        {"HoneyDew", cv::Scalar(240, 255, 240)},
        {"HotPink", cv::Scalar(255, 105, 180)},
        {"IndianRed", cv::Scalar(205, 92, 92)},
        {"Indigo", cv::Scalar(75, 0, 130)},
        {"Ivory", cv::Scalar(255, 255, 240)},
        {"Khaki", cv::Scalar(240, 230, 140)},
        {"LavenderBlush", cv::Scalar(255, 240, 245)},
        {"Lavender", cv::Scalar(230, 230, 250)},
        {"LawnGreen", cv::Scalar(124, 252, 0)},
        {"LemonChiffon", cv::Scalar(255, 250, 205)},
        {"LightBlue", cv::Scalar(173, 216, 230)},
        {"LightCoral", cv::Scalar(240, 128, 128)},
        {"LightCyan", cv::Scalar(224, 255, 255)},
        {"LightGoldenRodYellow", cv::Scalar(250, 250, 210)},
        {"LightGray", cv::Scalar(211, 211, 211)},
        {"LightGreen", cv::Scalar(144, 238, 144)},
        {"LightPink", cv::Scalar(255, 182, 193)},
        {"LightSalmon", cv::Scalar(255, 160, 122)},
        {"LightSeaGreen", cv::Scalar(32, 178, 170)},
        {"LightSkyBlue", cv::Scalar(135, 206, 250)},
        {"LightSlateGray", cv::Scalar(119, 136, 153)},
        {"LightSteelBlue", cv::Scalar(176, 196, 222)},
        {"LightYellow", cv::Scalar(255, 255, 224)},
        {"Lime", cv::Scalar(0, 255, 0)},
        {"LimeGreen", cv::Scalar(50, 205, 50)},
        {"Linen", cv::Scalar(250, 240, 230)},
        {"Magenta", cv::Scalar(255, 0, 255)},
        {"Maroon", cv::Scalar(128, 0, 0)},
        {"MediumAquaMarine", cv::Scalar(102, 205, 170)},
        {"MediumBlue", cv::Scalar(0, 0, 205)},
        {"MediumOrchid", cv::Scalar(186, 85, 211)},
        {"MediumPurple", cv::Scalar(147, 112, 219)},
        {"MediumSeaGreen", cv::Scalar(60, 179, 113)},
        {"MediumSlateBlue", cv::Scalar(123, 104, 238)},
        {"MediumSpringGreen", cv::Scalar(0, 250, 154)},
        {"MediumTurquoise", cv::Scalar(72, 209, 204)},
        {"MediumVioletRed", cv::Scalar(199, 21, 133)},
        {"MidnightBlue", cv::Scalar(25, 25, 112)},
        {"MintCream", cv::Scalar(245, 255, 250)},
        {"MistyRose", cv::Scalar(255, 228, 225)},
        {"Moccasin", cv::Scalar(255, 228, 181)},
        {"NavajoWhite", cv::Scalar(255, 222, 173)},
        {"Navy", cv::Scalar(0, 0, 128)},
        {"OldLace", cv::Scalar(253, 245, 230)},
        {"Olive", cv::Scalar(128, 128, 0)},
        {"OliveDrab", cv::Scalar(107, 142, 35)},
        {"Orange", cv::Scalar(255, 165, 0)},
        {"OrangeRed", cv::Scalar(255, 69, 0)},
        {"Orchid", cv::Scalar(218, 112, 214)},
        {"PaleGoldenRod", cv::Scalar(238, 232, 170)},
        {"PaleGreen", cv::Scalar(152, 251, 152)},
        {"PaleTurquoise", cv::Scalar(175, 238, 238)},
        {"PaleVioletRed", cv::Scalar(219, 112, 147)},
        {"PapayaWhip", cv::Scalar(255, 239, 213)},
        {"PeachPuff", cv::Scalar(255, 218, 185)},
        {"Peru", cv::Scalar(205, 133, 63)},
        {"Pink", cv::Scalar(255, 192, 203)},
        {"Plum", cv::Scalar(221, 160, 221)},
        {"PowderBlue", cv::Scalar(176, 224, 230)},
        {"Purple", cv::Scalar(128, 0, 128)},
        {"RebeccaPurple", cv::Scalar(102, 51, 153)},
        {"Red", cv::Scalar(255, 0, 0)},
        {"RosyBrown", cv::Scalar(188, 143, 143)},
        {"RoyalBlue", cv::Scalar(65, 105, 225)},
        {"SaddleBrown", cv::Scalar(139, 69, 19)},
        {"Salmon", cv::Scalar(250, 128, 114)},
        {"SandyBrown", cv::Scalar(244, 164, 96)},
        {"SeaGreen", cv::Scalar(46, 139, 87)},
        {"SeaShell", cv::Scalar(255, 245, 238)},
        {"Sienna", cv::Scalar(160, 82, 45)},
        {"Silver", cv::Scalar(192, 192, 192)},
        {"SkyBlue", cv::Scalar(135, 206, 235)},
        {"SlateBlue", cv::Scalar(106, 90, 205)},
        {"SlateGray", cv::Scalar(112, 128, 144)},
        {"Snow", cv::Scalar(255, 250, 250)},
        {"SpringGreen", cv::Scalar(0, 255, 127)},
        {"SteelBlue", cv::Scalar(70, 130, 180)},
        {"Tan", cv::Scalar(210, 180, 140)},
        {"Teal", cv::Scalar(0, 128, 128)},
        {"Thistle", cv::Scalar(216, 191, 216)},
        {"Tomato", cv::Scalar(255, 99, 71)},
        {"Turquoise", cv::Scalar(64, 224, 208)},
        {"Violet", cv::Scalar(238, 130, 238)},
        {"Wheat", cv::Scalar(245, 222, 179)},
        {"White", cv::Scalar(255, 255, 255)},
        {"WhiteSmoke", cv::Scalar(245, 245, 245)},
        {"Yellow", cv::Scalar(255, 255, 0)},
        {"YellowGreen", cv::Scalar(154, 205, 50)},
    };

    PedestrianDetector::PedestrianDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , dump_images(false)
        , avg_fp_ms()
        , avg_count(0)
        , category_index()
        , graph_file()
        , checkpoint_path()
        , session(nullptr)
        , graph_def()
        , checkpointPathTensor(tensorflow::DT_STRING, tensorflow::TensorShape()) {

        on<Configuration>("PedestrianDetector.yaml").then([this](const Configuration& config) {
            dump_images = config["dump_images"].as<bool>();
            graph_file  = config["model"]["graph_file"].as<std::string>();

            // Load labels map
            if (!loadLabelMap(config["model"]["label_file"].as<std::string>())) {
                return;
            }

            // Create a tensorflow session.
            if (session) {
                session.reset(nullptr);
            }

            tensorflow::Session* sess;
            tensorflow::Status status = tensorflow::NewSession(tensorflow::SessionOptions(), &sess);
            if (!status.ok()) {
                log<NUClear::ERROR>(fmt::format("Failed to create a tensorflow session: {}", status.ToString()));
                return;
            }
            session.reset(sess);

            // Load the tensorflow graph.
            status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_file, &graph_def);
            if (!status.ok()) {
                log<NUClear::ERROR>(
                    fmt::format("Failed to load tensorflow graph {}: {}", graph_file, status.ToString()));
                session.reset(nullptr);
                return;
            }

            // Load graph into session.
            status = session->Create(graph_def);
            if (!status.ok()) {
                log<NUClear::ERROR>(
                    fmt::format("Failed to load tensorflow graph into sesssion: {}", status.ToString()));
                session.reset(nullptr);
                return;
            }
        });

        on<Trigger<ReprojectedImage>, With<Sensors>, Single>().then([this](const ReprojectedImage& image,
                                                                           const Sensors& sensors) {
            if (session && (image.camera_id > 1)) {
                // For benchmarking.
                auto start = NUClear::clock::now();

                // Convert image to tensorflow tensor.
                tensorflow::Tensor input_img(
                    tensorflow::DT_UINT8, tensorflow::TensorShape({1, image.dimensions.y(), image.dimensions.x(), 3}));
                std::copy(image.data.begin(), image.data.end(), input_img.flat<uint8_t>().data());

                log<NUClear::DEBUG>("1");

                // Actual detection.
                std::vector<tensorflow::Tensor> outputs;
                session->Run({std::make_pair("image_tensor:0", input_img)},
                             {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"},
                             {},
                             &outputs);

                log<NUClear::DEBUG>("2");

                // Convert output tensors into usable formats.
                std::vector<std::array<float, 4>> boxes;

                for (int i = 0; i < outputs[0].NumElements() / 4; i += 4) {
                    boxes.push_back({outputs[0].flat<float>().data()[i + 0],
                                     outputs[0].flat<float>().data()[i + 1],
                                     outputs[0].flat<float>().data()[i + 2],
                                     outputs[0].flat<float>().data()[i + 3]});
                }
                std::vector<float> scores(outputs[1].flat<float>().data(),
                                          outputs[1].flat<float>().data() + outputs[1].NumElements());
                std::vector<int> classes(outputs[2].flat<int>().data(),
                                         outputs[2].flat<int>().data() + outputs[2].NumElements());
                std::vector<int> num_detections(outputs[3].flat<int>().data(),
                                                outputs[3].flat<int>().data() + outputs[3].NumElements());

                log<NUClear::DEBUG>("3");

                auto end   = NUClear::clock::now();
                auto fp_ms = std::chrono::duration<double, std::milli>(end - start);
                avg_fp_ms += fp_ms;
                avg_count++;
                if (((avg_count - 1) % 100) == 0) {
                    log<NUClear::INFO>(fmt::format("Image reprojection time: {0:.4f} ms (avg: {1:.4f} ms)",
                                                   fp_ms.count(),
                                                   (avg_fp_ms / avg_count).count()));
                }

                BakedImage img;
                img.format        = image.format;
                img.dimensions    = image.dimensions;
                img.camera_id     = image.camera_id;
                img.serial_number = "PedestrianDetector";
                img.timestamp     = NUClear::clock::now();
                std::copy(image.data.begin(), image.data.end(), img.data.end());

                // Visualization of the results of a detection.
                visualizeBoxesAndLabelsOnImageArray(img, boxes, classes, scores, true, 8);

                if (dump_images) {
                    utility::vision::saveImage(fmt::format("pedestrian_detector-{}.ppm", avg_count), img);
                }

                // Find the person detection that has the greatest width based score
                // (the person that is closest to the camera with the highest confidence)
                int index             = -1;
                float max_width_score = 0.0;
                float max_score       = 0.0;

                for (size_t i = 0; i < scores.size(); i++) {
                    if (category_index[classes[i]].compare("person") == 0) {
                        if ((scores[i] * (boxes[i][3] - boxes[i][1])) > max_width_score) {
                            max_score       = scores[i];
                            index           = i;
                            max_width_score = scores[i] * (boxes[i][3] - boxes[i][1]);
                        }
                    }
                }

                // If there is a person that is close to the camera turn the head to look at it.
                if ((max_score >= 0.5f) && (index > -1)) {
                    // Figure out camera parameters
                    Eigen::Vector2f image_center  = (image.dimensions.cast<float>() - Eigen::Vector2f::Ones()) * 0.5f;
                    float cam_focal_length_pixels = image_center.x() / std::tan(image.FOV * M_PI / 360.0f);

                    // Approximate pedestrian head position as vertically in the middle and about 10% from the top
                    // of the box
                    float width  = (boxes[index][3] - boxes[index][1]) * image.dimensions[0];
                    float height = (boxes[index][2] - boxes[index][0]) * image.dimensions[1];
                    float x      = (boxes[index][1] * image.dimensions[0]) + width * 0.5;
                    float y      = (boxes[index][0] * image.dimensions[1]) + height * 0.1;

                    // Form unit vector from camera to head position
                    Eigen::Vector3f vec(cam_focal_length_pixels, image_center.x() - x, image_center.y() - y);
                    vec.normalize();

                    // Get camera to torso transform
                    Eigen::Matrix4f Htc      = sensors.forwardKinematics[ServoID::HEAD_PITCH].cast<float>();
                    Eigen::Vector4f body_vec = Htc * Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 0);
                    Eigen::Vector2f screen_angular(std::atan2(body_vec.y(), body_vec.x()),
                                                   std::atan2(body_vec.z(), body_vec.x()));

                    // Create and emit head movement command
                    HeadCommand head_command;
                    head_command.yaw        = screen_angular[0];
                    head_command.pitch      = screen_angular[1];
                    head_command.robotSpace = true;
                    emit(std::make_unique<HeadCommand>(head_command));

                    // Create and emit wave command
                    if (width > (0.25 * image.dimensions[0])) {
                        WaveRightCommand cmd;
                        cmd.pre_delay  = 0;
                        cmd.post_delay = 10;
                        emit(std::make_unique<WaveRightCommand>(cmd));
                    }
                }

                emit(std::make_unique<BakedImage>(img));
            }
        });
    }

    bool PedestrianDetector::loadLabelMap(const std::string& label_file) {
        category_index.clear();

        std::ifstream ifs(label_file);
        YAML::Node labels = YAML::Load(ifs);

        for (const auto& label : labels["labels"]) {
            std::string name = label["display_name"].as<std::string>();
            int id           = label["id"].as<int>();
            if (id < 1) {
                log<NUClear::ERROR>(fmt::format("Label '{}' has incorrent format: ID {} < 1", name, id));
                return false;
            }

            category_index[id] = name;
        }

        return true;
    }

    void PedestrianDetector::visualizeBoxesAndLabelsOnImageArray(BakedImage& image,
                                                                 const std::vector<std::array<float, 4>>& boxes,
                                                                 const std::vector<int>& classes,
                                                                 const std::vector<float>& scores,
                                                                 bool use_normalized_coordinates,
                                                                 size_t max_boxes_to_draw,
                                                                 float min_score_thresh,
                                                                 bool agnostic_mode,
                                                                 int line_thickness) {
        /**
          * @brief Overlay labeled boxes on an image with formatted scores and label names.
          *
          * @details This function groups boxes that correspond to the same location
          * and creates a display string for each detection and overlays these
          * on the image. Note that this function modifies the image in place, and returns
          * that same image.
          *
          * @param image uint8 numpy array with shape (img_height, img_width, 3)
          * @param boxes a numpy array of shape [N, 4]
          * @param classes a numpy array of shape [N]. Note that class indices are 1-based,
                   and match the keys in the label map.
          * @param scores a numpy array of shape [N] or None.  If scores=None, then
                   this function assumes that the boxes to be plotted are groundtruth
                   boxes and plot all boxes as black with no classes or scores.
          * @param category_index a dict containing category dictionaries (each holding
                   category index `id` and category name `name`) keyed by category indices.
          * @param use_normalized_coordinates whether boxes is to be interpreted as
                   normalized coordinates or not.
          * @param max_boxes_to_draw maximum number of boxes to visualize.  If None, draw
                   all boxes.
          * @param min_score_thresh minimum score threshold for a box to be visualized
          * @param agnostic_mode boolean (default: False) controlling whether to evaluate in
                   class-agnostic mode or not.  This mode will display scores but ignore
                   classes.
          * @param line_thickness integer (default: 4) controlling line width of the boxes.
         */

        // Create a display string(and color) for every box location, group any boxes
        // that correspond to the same location.

        std::map<std::array<float, 4>, cv::Scalar> box_to_color_map;
        std::map<std::array<float, 4>, std::vector<std::string>> box_to_display_str_map;

        for (size_t i = 0; i < std::min(max_boxes_to_draw, boxes.size()); i++) {
            if (scores.empty() || scores[i] > min_score_thresh) {
                std::string display_str;

                auto box = boxes[i];

                if (scores.empty()) {
                    box_to_color_map[box] = STANDARD_COLORS.at("black");
                }

                else {
                    if (!agnostic_mode) {
                        auto it                = category_index.find(classes[i]);
                        std::string class_name = "N/A";
                        if (it != category_index.end()) {
                            class_name = category_index[classes[i]];
                        }

                        display_str = fmt::format("{}: {}%", class_name, int(100 * scores[i]));
                    }

                    else {
                        display_str = fmt::format("score: {}%", int(100 * scores[i]));
                    }
                }

                box_to_display_str_map[box].push_back(display_str);

                if (agnostic_mode) {
                    box_to_color_map[box] = STANDARD_COLORS.at("DarkOrange");
                }

                else {
                    std::vector<std::string> keys;
                    std::transform(STANDARD_COLORS.begin(),
                                   STANDARD_COLORS.end(),
                                   std::back_inserter(keys),
                                   [](const std::pair<std::string, cv::Scalar>& a) { return a.first; });
                    box_to_color_map[box] = STANDARD_COLORS.at(keys[classes[i] % keys.size()]);
                }
            }
        }

        // Draw all boxes onto image.
        for (const auto& box : box_to_color_map) {
            drawBoundingBoxOnImage(image,
                                   box.first,
                                   box.second,
                                   line_thickness,
                                   box_to_display_str_map[box.first],
                                   use_normalized_coordinates);
        }
    }

    void PedestrianDetector::drawBoundingBoxOnImage(BakedImage& image,
                                                    const std::array<float, 4>& box,
                                                    const cv::Scalar& color,
                                                    int thickness,
                                                    const std::vector<std::string>& display_str_list,
                                                    bool use_normalized_coordinates) {
        /**
         * @brief Adds a bounding box to an image.
         *
         * @details Each string in display_str_list is displayed on a separate line above the
         * bounding box in black text on a rectangle filled with the input 'color'.
         * If the top of the bounding box extends to the edge of the image, the strings
         * are displayed below the bounding box.
         *
         * @param image a BakedImage object.
         * @param ymin ymin of bounding box.
         * @param xmin xmin of bounding box.
         * @param ymax ymax of bounding box.
         * @param xmax xmax of bounding box.
         * @param color color to draw bounding box. Default is red.
         * @param thickness line thickness. Default value is 4.
         * @param display_str_list list of strings to display in box (each to be shown on its own line).
         * @param use_normalized_coordinates If True (default), treat coordinates
         *   ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
         *   coordinates as absolute.
         */
        int top    = box[0] * (use_normalized_coordinates ? image.dimensions.y() : 1);
        int left   = box[1] * (use_normalized_coordinates ? image.dimensions.x() : 1);
        int bottom = box[2] * (use_normalized_coordinates ? image.dimensions.y() : 1);
        int right  = box[3] * (use_normalized_coordinates ? image.dimensions.x() : 1);

        cv::Mat img(image.dimensions.y(), image.dimensions.x(), CV_8UC3, image.data.data());
        cv::line(img, cv::Point(left, top), cv::Point(right, top), color, thickness);
        cv::line(img, cv::Point(right, top), cv::Point(right, bottom), color, thickness);
        cv::line(img, cv::Point(right, bottom), cv::Point(left, bottom), color, thickness);
        cv::line(img, cv::Point(left, bottom), cv::Point(left, top), color, thickness);

        int fontFace      = cv::FONT_HERSHEY_DUPLEX;
        float fontScale   = 1.0f;
        int fontThickness = 1;

        // If the total height of the display strings added to the top of the bounding
        // box exceeds the top of the image, stack the strings below the bounding box
        // instead of above.
        std::vector<cv::Size> display_str_heights;
        float total_display_str_height = 0.0f;
        for (const auto& display_str : display_str_list) {
            cv::Size test_size = cv::getTextSize(display_str, fontFace, fontScale, fontThickness, 0);
            display_str_heights.push_back(test_size);
            total_display_str_height += test_size.height;
        }

        // Each display_str has a top and bottom margin of 0.05x.
        total_display_str_height *= 1 + 2 * 0.05;

        float text_bottom;

        if (top > total_display_str_height) {
            text_bottom = top;
        }

        else {
            text_bottom = bottom + total_display_str_height;
        }

        // Reverse list and print from bottom to top.
        for (auto display_str = display_str_list.rbegin(); display_str != display_str_list.rend(); display_str++) {
            cv::Size test_size = cv::getTextSize(*display_str, fontFace, fontScale, fontThickness, 0);
            float margin       = std::ceil(0.05f * test_size.height);
            cv::rectangle(img,
                          cv::Point(left, text_bottom - test_size.height - 2 * margin),
                          cv::Point(left + test_size.width, text_bottom),
                          color);

            cv::Point org;
            org.x = left + margin;
            org.y = text_bottom - margin;
            cv::putText(img, *display_str, org, fontFace, fontScale, CV_RGB(0, 0, 0), fontThickness, 16);
            text_bottom -= test_size.height - 2 * margin;
        }
    }


}  // namespace vision

}  // namespace module
