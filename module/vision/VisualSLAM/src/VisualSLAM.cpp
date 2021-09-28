// System.
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <mutex>
// Local - NUBots.
#include "VisualSLAM.hpp"
#include "extension/Configuration.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "message/input/Image.hpp"
#include "clock/clock.hpp"
// Local - SLAM.
#include "ORB-SLAM/System.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>


namespace module::vision
{
    //////////////////////////////////////////////
    // SLAM components.
    //////////////////////////////////////////////
    std::unique_ptr<ORB_SLAM2::System> slamSystem;
    std::vector<std::pair<std::string, std::string>> slamSettings;
    //////////////////////////////////////////////

    //////////////////////////////////////////////
    // VisualSLAM.yaml settings for database.
    bool useImageDatabase;
    std::string imageDatabaseDirectory;
    // Local image database - vector of filepaths.
    std::vector<std::string> imageDataset;
    //////////////////////////////////////////////


    // !!! THE TODO LIST !!!
    // REPO:

    // TODO - log file for transformations.
        // Figure out how to get webots absolute transform.
        // Write transforms to file.
        // Logging flag in yaml.

    // TODO - patch transform emission into particle filter.
        // Work out how particle filter works / message NUbots team.

    // BONUS:
    // TODO - when to reset due to tracking loss.
    // TODO - shutdown slam system somehow.
    // TODO - add project to git branch.

    // PAPER:
    // TODO - write the damn lit paper.



    VisualSLAM::VisualSLAM(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{}
    {
        on<extension::Configuration>("VisualSLAM.yaml").then([this](const extension::Configuration& config)
        {
            // Log level.
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
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
                    protoImage->timestamp  = NUClear::clock::time_point(std::chrono::nanoseconds(i));

                    // Emit dataset camera image.
                    emit<Scope::DIRECT>(std::move(protoImage));
                }

                // Stop the slam system.
                slamSystem->Shutdown();
            }
        });


        on<Trigger<message::input::Image>>().then([this](const message::input::Image& protoImage)
        {
            // Image frame tracker.
            static uint frame = 0;
            log<NUClear::DEBUG>(fmt::format("--- Image received : {} --- ", frame++));

            // Skip next incoming message if already processing one.
            static std::mutex mutex;
            if (mutex.try_lock())
            {
                // Constant image data from protobuf image.
                const int imageID = protoImage.id;
                const std::string imageName = protoImage.name;
                const int imageDimensionsX = protoImage.dimensions.x();
                const int imageDimensionsY = protoImage.dimensions.y();
                const int imageFourColourCode = protoImage.format;
                const std::string imageFourColourName = utility::vision::fourcc(imageFourColourCode);
                const std::chrono::time_point imageTimePoint = protoImage.timestamp;
                //const uint8_t* imageData = const_cast<uint8_t*>(protoImage.data.data());

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

                    // Image lens:
                    const auto& lens = protoImage.lens;
                    //  UNKNOWN = 0, RECTILINEAR = 1, EQUIDISTANT = 2, EQUISOLID   = 3;
                    const int lensProjection = lens.projection;
                    /// The angular diameter that the lens covers (the area that light hits on the sensor), in radians.
                    const float lensFieldOfView = lens.fov;
                    /// Normalised focal length: focal length in pixels / image width.
                    const float lensFocalLength = lens.focal_length;
                    /// Normalised image centre offset: pixels from centre to optical axis / image width.
                    const float lensCentreX = lens.centre.x();
                    const float lensCentreY = lens.centre.y();
                    const float kX = lens.k.x();
                    const float kY = lens.k.y();

                    log<NUClear::DEBUG>(fmt::format("Lens Projection: {}", lensProjection));
                    log<NUClear::DEBUG>(fmt::format("Lens Field of View: {}", lensFieldOfView));
                    log<NUClear::DEBUG>(fmt::format("Lens Focal Length: {}", lensFocalLength));
                    log<NUClear::DEBUG>(fmt::format("Lens Center X: {}", lensCentreX));
                    log<NUClear::DEBUG>(fmt::format("Lens Center Y: {}", lensCentreY));
                    log<NUClear::DEBUG>(fmt::format("Lens K X: {}", kX));
                    log<NUClear::DEBUG>(fmt::format("Lens K Y: {}", kY));
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
                    static bool deleteImages = true;
                    if (deleteImages)
                    {
                        std::filesystem::remove_all("./images");
                        deleteImages = false;
                    }

                    static bool write = true;
                    if (write)
                    {
                        // docker cp 6d5c41befbd3:/home/nubots/build/images /home/brandon/Desktop/Development/ORB_SLAM2-master-3/images
                        log<NUClear::DEBUG>(fmt::format("Saving image: {}", frame));
                        std::filesystem::create_directories("./images/input");
                        std::filesystem::create_directories("./images/output"); //imageTimePoint.time_since_epoch().count()
                        cv::imwrite("./images/input/" + std::to_string(frame) + ".jpg", cvImageInput);   // inputImage_
                        cv::imwrite("./images/output/" + std::to_string(frame) + ".jpg", cvImageOutput); //outputImage_
                        //write = false;
                    }
                }


                if (cameraTrackingPose.empty())
                {
                    log<NUClear::DEBUG>("Tracking empty.");
                }
                else
                {
                    log<NUClear::DEBUG>("Tracking M:");
                    log<NUClear::DEBUG>(cameraTrackingPose);

                    //emit(std::move(Tcw));
                    //Eigen::Affine3d Hcw(image.Hcw);
                }


                // Image finished processing - allow next image to enter.
                mutex.unlock();
            }
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

        // Sort the vector numerically to ensure correct image order. E.g. 0.jpg, 1.jpg, 2.jpg, etc.
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
