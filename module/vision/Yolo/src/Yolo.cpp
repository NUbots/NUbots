#include "Yolo.hpp"

#include <chrono>
#include <getopt.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"

namespace module::vision {

    using namespace std;
    using namespace cv;

    using extension::Configuration;

    using message::input::Image;

    Yolo::Yolo(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Yolo.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Yolo.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then("Test Yolo", [this] {
            inf = Inference("/home/nubots/build/yolo.onnx", cv::Size(640, 640), "classes.txt", false);
        });

        on<Trigger<Image>, Single>().then([this](const Image& img) {
            // -------- Convert Image to cv::Mat --------
            const int width  = img.dimensions.x();
            const int height = img.dimensions.y();
            cv::Mat frame    = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));


            // -------- Run Inference --------
            auto start                    = std::chrono::high_resolution_clock::now();
            std::vector<Detection> output = inf.runInference(frame);
            auto stop                     = std::chrono::high_resolution_clock::now();
            auto duration                 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            int detections                = output.size();
            log<NUClear::DEBUG>("Inference took: ", duration.count(), "ms");
            log<NUClear::DEBUG>("Number of detections: ", detections);

            // -------- Draw Detections --------
            for (int i = 0; i < detections; ++i) {
                Detection detection = output[i];

                cv::Rect box     = detection.box;
                cv::Scalar color = detection.color;

                // Detection box with thinner line
                cv::rectangle(frame, box, color, 1);  // Thickness reduced to 1

                // Detection box text with smaller font
                std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
                float fontScale         = 0.5;  // Reduced font scale
                int thickness           = 1;    // Thickness for text, can be reduced if needed
                cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, fontScale, thickness, 0);
                cv::Rect textBox(box.x, box.y - 20, textSize.width + 10, textSize.height + 10);  // Adjusted size

                cv::rectangle(frame, textBox, color, cv::FILLED);
                cv::putText(frame,
                            classString,
                            cv::Point(box.x + 5, box.y - 10),
                            cv::FONT_HERSHEY_DUPLEX,
                            fontScale,
                            cv::Scalar(0, 0, 0),
                            thickness,
                            0);
            }

            float scale = 0.8;
            cv::resize(frame, frame, cv::Size(frame.cols * scale, frame.rows * scale));
            cv::imwrite("recordings/yolo.jpg", frame);
        });
    }

}  // namespace module::vision
