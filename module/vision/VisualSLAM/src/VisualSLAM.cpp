/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

// System.
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>
// Eigen.
#include <Eigen/Core>
#include <Eigen/Geometry>
// OpenCV.
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>
// NUBots.
#include "VisualSLAM.hpp"
#include "extension/Configuration.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "message/input/Image.hpp"
#include "message/vision/VisualSLAM.hpp"
#include "clock/clock.hpp"


namespace module::vision
{
    using extension::Configuration;

    VisualSLAM::VisualSLAM(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{}
    {
        on<Configuration>("VisualSLAM.yaml").then([this](const Configuration& config)
        {
            // Log level.
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            // Data logging settings.
            saveImages              = config["saveImages"].as<bool>();
            saveDataLog             = config["saveDataLog"].as<bool>();
            // Database streaming options.
            useImageDatabase        = config["useImageDatabase"].as<bool>();
            imageDatabaseDirectory  = config["imageDatabaseDirectory"].as<std::string>();
            // ORB-SLAM settings.
            slamSettings.push_back(std::make_pair("Vocabulary_File", config["Vocabulary_File"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_fx", config["Camera_fx"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_fy", config["Camera_fy"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_cx", config["Camera_cx"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_cy", config["Camera_cy"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_k1", config["Camera_k1"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_k2", config["Camera_k2"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_k3", config["Camera_k3"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_p1", config["Camera_p1"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_p2", config["Camera_p2"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Camera_fps", config["Camera_fps"].as<std::string>()));
            slamSettings.push_back(std::make_pair("ORBextractor_nFeatures", config["ORBextractor_nFeatures"].as<std::string>()));
            slamSettings.push_back(std::make_pair("ORBextractor_scaleFactor", config["ORBextractor_scaleFactor"].as<std::string>()));
            slamSettings.push_back(std::make_pair("ORBextractor_nLevels", config["ORBextractor_nLevels"].as<std::string>()));
            slamSettings.push_back(std::make_pair("ORBextractor_iniThFAST", config["ORBextractor_iniThFAST"].as<std::string>()));
            slamSettings.push_back(std::make_pair("ORBextractor_minThFAST", config["ORBextractor_minThFAST"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_KeyFrameSize", config["Viewer_KeyFrameSize"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_KeyFrameLineWidth", config["Viewer_KeyFrameLineWidth"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_GraphLineWidth", config["Viewer_GraphLineWidth"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_PointSize", config["Viewer_PointSize"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_CameraSize", config["Viewer_CameraSize"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_CameraLineWidth", config["Viewer_CameraLineWidth"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_ViewpointX", config["Viewer_ViewpointX"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_ViewpointY", config["Viewer_ViewpointY"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_ViewpointZ", config["Viewer_ViewpointZ"].as<std::string>()));
            slamSettings.push_back(std::make_pair("Viewer_ViewpointF", config["Viewer_ViewpointF"].as<std::string>()));
        });


        on<Startup>().then([this]
        {
            if (saveDataLog)
            {
                // Create log file for SLAM output.
                std::filesystem::create_directories("./logging");
                outputFileStream.open("./logging/log.txt");
            }


            // Initialise SLAM system - Pangolin viewer is currently not applicable in NUBots docker.
            slamSystem = std::make_unique<ORB_SLAM2::System>(slamSettings, false);


            // Running in live mode.
            if (useImageDatabase == false)
            {
                log<NUClear::DEBUG>(fmt::format("-------------------------------------------"));
                log<NUClear::DEBUG>(fmt::format("VSLAM RUNNING IN LIVE MODE"));
                log<NUClear::DEBUG>(fmt::format("-------------------------------------------"));
            }
            // Running in database mode.
            else
            {
                log<NUClear::DEBUG>(fmt::format("-------------------------------------------"));
                log<NUClear::DEBUG>(fmt::format("VSLAM RUNNING IN DATABASE MODE"));
                log<NUClear::DEBUG>(fmt::format("-------------------------------------------"));

                // Populate image dataset from local disk.
                imageDataset = LoadImageDatabase(imageDatabaseDirectory);
                // No images loaded.
                if (imageDataset.size() == 0)
                {
                    // Error handling.
                    log<NUClear::DEBUG>(fmt::format("Error: no images loaded from {} images.txt", imageDatabaseDirectory));
                    return;
                }
                else // Images loaded.
                {
                    log<NUClear::DEBUG>(fmt::format("Info: total of {} images loaded", imageDataset.size()));
                }


                // Loop until all images from local dataset have been emitted.
                for (size_t i = 0; i < imageDataset.size(); ++i)
                {
                    // Load image from disk - data will always be continuous.
                    cv::Mat cvImage = cv::imread(imageDataset[i], CV_LOAD_IMAGE_UNCHANGED);
                    // If image not found.
                    if (cvImage.empty())
                    {
                        // Error handling.
                        log<NUClear::DEBUG>(fmt::format("Error: could not load image: {}", imageDataset[i]));
                        break;
                    }

                    // Convert cv image matrix to a vector.
                    std::vector<uint8_t> data(cvImage.datastart, cvImage.dataend);

                    // Create proto message image from cv image.
                    std::unique_ptr<message::input::Image> protoImage = std::make_unique<message::input::Image>();
                    protoImage->format     = utility::vision::FOURCC::RGB3;
                    protoImage->dimensions = { cvImage.cols, cvImage.rows };
                    protoImage->data       = data;
                    protoImage->id         = 0;
                    protoImage->name       = imageDataset[i];
                    protoImage->timestamp  = NUClear::clock::now();

                    // Emit database camera image.
                    emit<Scope::DIRECT>(std::move(protoImage));
                }

                // Stop the slam system.
                slamSystem->Shutdown();
            }
        });


        on<Trigger<message::input::Image> , Single>().then([this](const message::input::Image& protoImage)
        {
            // Image frame tracker.
            static uint frame = 0;
            log<NUClear::DEBUG>(fmt::format("--- Image received : {} --- ", frame++));

            // Constant image data from protobuf image.
            const int imageID = protoImage.id;
            const std::string imageName = protoImage.name;
            const int imageDimensionsX = protoImage.dimensions.x();
            const int imageDimensionsY = protoImage.dimensions.y();
            const int imageFourColourCode = protoImage.format;
            const std::string imageFourColourName = utility::vision::fourcc(imageFourColourCode);
            const std::chrono::time_point imageTimePoint = protoImage.timestamp;
            //const uint8_t* imageData = const_cast<uint8_t*>(protoImage.data.data());
            //const Eigen::Affine3d Hcw(protoImage.Hcw);

            if (log_level <= NUClear::DEBUG)
            {
                // Debug log image data.
                log<NUClear::DEBUG>(fmt::format("ID: {}", imageID));
                log<NUClear::DEBUG>(fmt::format("Name: {}", imageName));
                log<NUClear::DEBUG>(fmt::format("Dimension X: {}", imageDimensionsX));
                log<NUClear::DEBUG>(fmt::format("Dimension Y: {}", imageDimensionsY));
                log<NUClear::DEBUG>(fmt::format("Four Colour Code: {}", imageFourColourCode));
                log<NUClear::DEBUG>(fmt::format("Four Colour Name: {}", imageFourColourName));
                log<NUClear::DEBUG>(fmt::format("Timestamp: {}", imageTimePoint.time_since_epoch().count()));

                //// Image lens:
                //const auto& lens = protoImage.lens;
                ////  UNKNOWN = 0, RECTILINEAR = 1, EQUIDISTANT = 2, EQUISOLID   = 3;
                //const int lensProjection = lens.projection;
                ///// The angular diameter that the lens covers (the area that light hits on the sensor), in radians.
                //const float lensFieldOfView = lens.fov;
                ///// Normalised focal length: focal length in pixels / image width.
                //const float lensFocalLength = lens.focal_length;
                ///// Normalised image centre offset: pixels from centre to optical axis / image width.
                //const float lensCentreX = lens.centre.x();
                //const float lensCentreY = lens.centre.y();
                //const float kX = lens.k.x();
                //const float kY = lens.k.y();
                //log<NUClear::DEBUG>(fmt::format("Lens Projection: {}", lensProjection));
                //log<NUClear::DEBUG>(fmt::format("Lens Field of View: {}", lensFieldOfView));
                //log<NUClear::DEBUG>(fmt::format("Lens Focal Length: {}", lensFocalLength));
                //log<NUClear::DEBUG>(fmt::format("Lens Center X: {}", lensCentreX));
                //log<NUClear::DEBUG>(fmt::format("Lens Center Y: {}", lensCentreY));
                //log<NUClear::DEBUG>(fmt::format("Lens K X: {}", kX));
                //log<NUClear::DEBUG>(fmt::format("Lens K Y: {}", kY));
            }


            // Cast protobuff input image to greyscale output image.
            cv::Mat cvImageInput;
            cv::Mat cvImageOutput;
            switch (imageFourColourCode)
            {
                case utility::vision::FOURCC::RGB3:
                case utility::vision::FOURCC::JPEG:
                    cvImageInput = cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC3, const_cast<uint8_t*>(protoImage.data.data()));
                    cv::cvtColor(cvImageInput, cvImageOutput, cv::COLOR_RGB2GRAY);
                    break;
                case utility::vision::fourcc("BGR3"):   // BGR3 not available in utility::vision::FOURCC.
                    cvImageInput = cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC3, const_cast<uint8_t*>(protoImage.data.data()));
                    cv::cvtColor(cvImageInput, cvImageOutput, cv::COLOR_BGR2GRAY);
                    break;
                case utility::vision::FOURCC::GREY:
                    cvImageInput = cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC1, const_cast<uint8_t*>(protoImage.data.data()));
                    cvImageOutput = cvImageInput.clone();
                    break;
                default:
                    log<NUClear::WARN>(fmt::format("Image format not supported: {}", imageFourColourName));
                    mutex.unlock();
                    return;
            }


            // Update the slam system tracking with the new black and white image.
            cv::Mat cameraTrackingPose = slamSystem->TrackMonocular
            (
                cvImageOutput,
                imageTimePoint.time_since_epoch().count()
            );


            if (log_level <= NUClear::DEBUG)
            {
                if (saveDataLog)
                {
                    // Save new data log - overwrites old data entries.
                    if (outputFileStream.is_open() && outputFileStream.good())
                    {
                        outputFileStream << frame << ";" << imageTimePoint.time_since_epoch().count() << ";";

                        if (cameraTrackingPose.empty())
                        {
                            outputFileStream << "EMPTY";
                        }
                        else
                        {
                            for (int i = 0; i < cameraTrackingPose.rows; i++)
                            {
                                for (int j = 0; j < cameraTrackingPose.cols; j++)
                                {
                                    outputFileStream << ";" << cameraTrackingPose.at<double>(i, j);
                                }
                            }
                        }

                        outputFileStream << ";" << std::endl;
                    }
                }

                if (saveImages)
                {
                    // Delete old images.
                    static bool deleteImages = true;
                    if (deleteImages)
                    {
                        std::filesystem::remove_all("./images");
                        std::filesystem::create_directories("./images/input");
                        std::filesystem::create_directories("./images/output");
                        deleteImages = false;
                    }

                    // Save new images.
                    log<NUClear::DEBUG>(fmt::format("Saving image: {}", frame));
                    cv::imwrite("./images/input/" + std::to_string(frame) + ".jpg", cvImageInput);
                    cv::imwrite("./images/output/" + std::to_string(frame) + ".jpg", cvImageOutput);
                }
            }


            if (!cameraTrackingPose.empty())
            {
                // Create VSLAM message for emission.
                std::unique_ptr<message::vision::VisualSLAM> message = std::make_unique<message::vision::VisualSLAM>();

                // Attach image timestamp to message.
                message->timestamp = imageTimePoint;
                // Cast OpenCV matrix to Eigen matrix.
                Eigen::Matrix<double, 4, 4> eigenMatrix;
                cv::cv2eigen(cameraTrackingPose, eigenMatrix);
                // Attach Eigen matrix to message.
                message->Hcw = eigenMatrix;

                // Emit message globally.
                //emit(std::move(message));
                // Emit message locally.
                emit<Scope::DIRECT>(std::move(message));
            }
        });


        // Message emission debug - set emission to emit<Scope::DIRECT>(std::move(message)) in tracking.
        on<Trigger<message::vision::VisualSLAM>>().then([this](const message::vision::VisualSLAM& slamMessage)
        {
            const Eigen::Affine3d Hcw(slamMessage.Hcw);
            const std::chrono::time_point messageTimePoint = slamMessage.timestamp;

            // Console log slamMessage Hcw matrix.
            std::stringstream ss;
            ss << Hcw.matrix();
            log<NUClear::DEBUG>(fmt::format("Tracking Data:\n-----------------------------------------------\n {} \n-----------------------------------------------", ss.str()));
        });
    }


    // Load image database from local disk.
    std::vector<std::string> VisualSLAM::LoadImageDatabase(const std::string& databaseDirectory)
    {
        // Vector of image file paths.
        std::vector<std::string> imageDatabase;

        // Loop over all files in the passed dataset directory.
        for (const filesystem::directory_entry& entry : std::filesystem::directory_iterator(databaseDirectory))
        {
            // Store the image file name.
            imageDatabase.push_back(entry.path().filename());
        }

        // Sort the vector numerically to ensure correct image order: 0.jpg, 1.jpg, 2.jpg, etc.
        std::sort(imageDatabase.begin(), imageDatabase.end(), [](const std::string& s1, const std::string& s2)
        {
            std::string::const_iterator it1 = s1.begin(), it2 = s2.begin();

            if (std::isdigit(s1[0]) && std::isdigit(s2[0]))
            {
                int n1, n2;
                std::stringstream ss(s1);
                ss >> n1;
                ss.clear();
                ss.str(s2);
                ss >> n2;
                if (n1 != n2) { return n1 < n2; }
                it1 = std::find_if(s1.begin(), s1.end(), [](char c){ return !std::isdigit(c); });
                it2 = std::find_if(s2.begin(), s2.end(), [](char c){ return !std::isdigit(c); });
            }

            return std::lexicographical_compare(it1, s1.end(), it2, s2.end());
        });

        for (std::string& entry : imageDatabase)
        {
            // Prepend the database directory to all file names.
            entry = filesystem::path(databaseDirectory) / filesystem::path(entry);
        }

        return imageDatabase;
    }
}
