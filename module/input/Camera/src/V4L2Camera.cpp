#include "Camera.h"

namespace module
{
    namespace input
    {
        using extension::Configuration;

        using message::input::CameraParameters;
        using message::input::Image;

        using FOURCC = utility::vision::FOURCC;

        V4L2Camera Camera::initiateV4L2Camera(const Configuration& config)
        {
            // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
            V4L2FrameRateHandle = on<Every<V4L2Camera::FRAMERATE, Per<std::chrono::seconds>>, Single>().then("Read V4L2Camera", [this] {

                for (auto& camera : V4L2Cameras)
                {
                    // If the camera is ready, get an image and emit it
                    if (camera.second.isStreaming())
                    {
                        emit(std::make_unique<Image>(camera.second.getImage()));
                    }
                }
            });

            V4L2SettingsHandle = on<Every<1, std::chrono::seconds>>().then("V4L2 Camera Setting Applicator", [this] {

                for (auto& camera : V4L2Cameras)
                {
                    if (camera.second.isStreaming())
                    {
                        // Set all other camera settings
                        for (auto& setting : camera.second.getConfig().config)
                        {
                            auto& settings = camera.second.getSettings();
                            auto it = settings.find(setting.first.as<std::string>());

                            if (it != settings.end())
                            {
                                if (camera.second.setSetting(it->second, setting.second.as<int>()) == false)
                                {
                                    log<NUClear::DEBUG>("Failed to set", it->first, "on camera", camera.first);
                                }
                            }
                        }
                    }
                }
            });

            auto cameraParameters = std::make_unique<CameraParameters>();
            double tanHalfFOV[2], imageCentre[2];

            //Generic camera parameters
            cameraParameters->imageSizePixels << config["imageWidth"].as<uint>(), config["imageHeight"].as<uint>();
            cameraParameters->FOV << config["FOV_X"].as<double>(), config["FOV_Y"].as<double>();
            //TODO: configure the offset? probably not necessary for pinhole
            cameraParameters->centreOffset = Eigen::Vector2i::Zero();

            //Pinhole specific
            cameraParameters->lens = CameraParameters::LensType::PINHOLE;
            tanHalfFOV[0]  = std::tan(cameraParameters->FOV[0] * 0.5);
            tanHalfFOV[1]  = std::tan(cameraParameters->FOV[1] * 0.5);
            imageCentre[0] = cameraParameters->imageSizePixels[0] * 0.5;
            imageCentre[1] = cameraParameters->imageSizePixels[1] * 0.5;
            cameraParameters->pinhole.distortionFactor = config["DISTORTION_FACTOR"].as<double>();
            cameraParameters->pinhole.pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]), (tanHalfFOV[1] / imageCentre[1]);
            cameraParameters->pinhole.focalLengthPixels = imageCentre[0] / tanHalfFOV[0];

            emit<Scope::DIRECT>(std::move(cameraParameters));

            log("Emitted pinhole camera parameters for camera", config["deviceID"].as<std::string>());

            try
            {
                // Recreate the camera device at the required resolution
                int width  = config["imageWidth"].as<uint>();
                int height = config["imageHeight"].as<uint>();
                std::string deviceID = config["deviceID"].as<std::string>();
                std::string format   = config["imageFormat"].as<std::string>();
                FOURCC fourcc = utility::vision::getFourCCFromDescription(format);

                log("Initialising driver for camera", deviceID);

                V4L2Camera camera(config, deviceID, cameraCount);

                camera.resetCamera(deviceID, format, fourcc, width, height);

                log("Applying settings for camera", deviceID);

                // Set all other camera settings
                for(auto& setting : config.config)
                {
                    auto& settings = camera.getSettings();
                    auto it = settings.find(setting.first.as<std::string>());

                    if(it != settings.end())
                    {
                        if (camera.setSetting(it->second, setting.second.as<int>()) == false)
                        {
                            log<NUClear::DEBUG>("Failed to set", it->first, "on camera", deviceID);
                        }
                    }
                }

                // Start the camera streaming video
                camera.startStreaming();

                log("Camera", deviceID, "is now streaming.");

                V4L2SettingsHandle.enable();
                V4L2FrameRateHandle.enable();

                return(std::move(camera));
            }

            catch(const std::exception& e)
            {
                NUClear::log<NUClear::DEBUG>(std::string("Exception while setting camera configuration: ") + e.what());
                throw e;
            }
        }

        void Camera::ShutdownV4L2Camera()
        {
            for (auto& camera : V4L2Cameras)
            {
                camera.second.closeCamera();
            }

            V4L2SettingsHandle.disable();
            V4L2FrameRateHandle.disable();
            V4L2Cameras.clear();
        }
    }
}
