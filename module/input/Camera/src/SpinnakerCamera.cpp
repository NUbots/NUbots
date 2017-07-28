#include "Camera.h"

#include <fmt/format.h>
#include <armadillo>
#include <cmath>
#include <exception>

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/Spinnaker.h"

namespace module {
namespace input {
    using message::input::CameraParameters;
    using extension::Configuration;
    using utility::support::Expression;
    using FOURCC = utility::vision::FOURCC;


    // void Camera::initiateSpinnakerCamera(const Configuration& config) {
    //     // if (!resetUSBDevice(config["usb"]["bus"].as<int>(), config["usb"]["device"].as<int>())) {
    //     // log<NUClear::FATAL>("Failed to reset Spinnaker camera with serial number",
    //     // config["deviceID"].as<std::string>());
    //     // return;
    //     //}

    //     try {
    //         if (!SpinnakerSystem) {
    //             SpinnakerSystem = Spinnaker::System::GetInstance();
    //         }

    //         // Register logging callback class
    //         SpinnakerSystem->RegisterLoggingEvent((Spinnaker::LoggingEvent&) (SpinnakerLoggingCallback));

    //         // Set callback priority level
    //         SpinnakerSystem->SetLoggingEventPriorityLevel(Spinnaker::LOG_LEVEL_ERROR);

    //         SpinnakerCamList = SpinnakerSystem->GetCameras(true, true);
    //         log<NUClear::DEBUG>("Found ", SpinnakerCamList.GetSize(), " cameras.");

    //         if (SpinnakerCamList.GetSize() < 1) {
    //             return;
    //         }

    //         std::string deviceID = config["deviceID"].as<std::string>();

    //         log<NUClear::DEBUG>("Processing camera", config.fileName, "with serial number", deviceID);

    //         // See if we already have this camera
    //         auto camera = SpinnakerCameras.find(deviceID);

    //         if ((camera != SpinnakerCameras.end())
    //             && (!camera->second->camera->IsValid() || !camera->second->camera->IsInitialized())) {
    //             SpinnakerCameras.erase(camera->first);
    //             camera = SpinnakerCameras.end();
    //         }

    //         if (camera == SpinnakerCameras.end()) {
    //             Spinnaker::CameraPtr newCamera = SpinnakerCamList.GetBySerial(deviceID);

    //             // Ensure we found the camera.
    //             if (newCamera) {
    //                 // Initlise the camera.
    //                 newCamera->Init();

    //                 // Add camera to list.
    //                 FOURCC fourcc =
    //                     utility::vision::getFourCCFromDescription(config["format"]["pixel"].as<std::string>());
    //                 camera =
    //                     SpinnakerCameras
    //                         .insert(std::make_pair(deviceID,
    //                                                std::make_unique<SpinnakerImageEvent>(config.fileName,
    //                                                                                      deviceID,
    //                                                                                      std::move(newCamera),
    //                                                                                      *this,
    //                                                                                      fourcc,
    //                                                                                      cameraCount,
    //                                                                                      config["is_left"].as<bool>())))
    //                         .first;
    //             }

    //             else {
    //                 log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number:", deviceID);
    //                 return;
    //             }
    //         }

    //         resetSpinnakerCamera(camera, config);
    //     }

    //     catch (Spinnaker::Exception& ex) {
    //         log<NUClear::WARN>(ex.what());
    //         log<NUClear::WARN>(ex.GetFullErrorMessage());
    //         log<NUClear::WARN>(ex.GetFileName());
    //         log<NUClear::WARN>(ex.GetFunctionName());
    //         log<NUClear::WARN>(ex.GetBuildDate());
    //         log<NUClear::WARN>(ex.GetBuildTime());
    //         log<NUClear::WARN>(ex.GetLineNumber());
    //     }

    //     catch (std::exception& ex) {
    //         log<NUClear::WARN>(ex.what());
    //     }

    //     auto cameraParameters = std::make_unique<CameraParameters>();

    //     // Generic camera parameters
    //     cameraParameters->imageSizePixels << config["format"]["width"].as<uint>(),
    //         config["format"]["height"].as<uint>();
    //     cameraParameters->FOV << config["lens"]["FOV"].as<double>(), config["lens"]["FOV"].as<double>();

    //     // Radial specific
    //     cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
    //     cameraParameters->radial.radiansPerPixel = config["lens"]["radiansPerPixel"].as<float>();
    //     cameraParameters->centreOffset           = convert<int, 2>(config["lens"]["centreOffset"].as<arma::ivec>());

    //     emit<Scope::DIRECT>(std::move(cameraParameters));

    //     log("Emitted radial camera parameters for camera", config["deviceID"].as<std::string>());
    // }

    void Camera::resetSpinnakerCamera(std::map<std::string, std::unique_ptr<SpinnakerImageEvent>>::iterator& camera,
                                      const Configuration& config) {
        //     try {
        //         if (camera->second->camera->IsStreaming()) {
        //             camera->second->camera->EndAcquisition();
        //         }

        //         // Get device node maps
        //         auto& nodeMap  = camera->second->camera->GetNodeMap();
        //         auto& sNodeMap = camera->second->camera->GetTLStreamNodeMap();

        //         // Set the width and height of the image.
        //         if (!utility::vision::setNumericParam(
        //                 sNodeMap, "StreamDefaultBufferCount", config["buffer_count"].as<int64_t>())) {
        //             log("Failed to set software buffer count for camera",
        //                 camera->first,
        //                 "to",
        //                 config["buffer_count"].as<int64_t>());
        //         }

        //         // Set the pixel format.
        //         if (!utility::vision::setEnumParam(nodeMap, "PixelFormat",
        //         config["format"]["pixel"].as<std::string>()))
        //         {
        //             log("Failed to set pixel format for camera",
        //                 camera->first,
        //                 "to",
        //                 config["format"]["pixel"].as<std::string>());
        //         }

        //         // Set the width and height of the image.
        //         if (!utility::vision::setNumericParam(nodeMap, "Width", config["format"]["width"].as<int64_t>())) {
        //             log("Failed to set image width for camera",
        //                 camera->first,
        //                 "to",
        //                 config["format"]["width"].as<int64_t>());
        //         }

        //         if (!utility::vision::setNumericParam(nodeMap, "Height", config["format"]["height"].as<int64_t>())) {
        //             log("Failed to set image height for camera",
        //                 camera->first,
        //                 "to",
        //                 config["format"]["height"].as<int64_t>());
        //         }

        //         // Set exposure.
        //         auto exposure = config["settings"]["exposure"].as<Expression>();

        //         if (!utility::vision::setEnumParam(
        //                 nodeMap, "ExposureAuto", (std::isfinite(exposure.value) ? "Off" : "Continuous"))) {
        //             log("Failed to set auto exposure mode for camera",
        //                 camera->first,
        //                 "to",
        //                 (std::isfinite(exposure.value) ? "'Off'" : "'Continuous'"));
        //         }

        //         if (std::isfinite(exposure.value)) {
        //             if (!utility::vision::setEnumParam(nodeMap, "ExposureMode", "Timed")) {
        //                 log("Failed to set exposure mode for camera", camera->first, "to 'Timed'");
        //             }

        //             if (!utility::vision::setNumericParam(nodeMap, "ExposureTime", exposure.value)) {
        //                 log("Failed to set exposure time for camera", camera->first, "to", exposure.value, "us.");
        //             }
        //         }

        //         // Set gain.
        //         auto gain = config["settings"]["gain"].as<Expression>();

        //         if (!utility::vision::setEnumParam(
        //                 nodeMap, "GainAuto", (std::isfinite(gain.value) ? "Off" : "Continuous"))) {
        //             log("Failed to set auto gain mode for camera",
        //                 camera->first,
        //                 "to",
        //                 (std::isfinite(gain.value) ? "'Off'" : "'Continuous'"));
        //         }

        //         if (std::isfinite(gain.value)) {
        //             if (!utility::vision::setNumericParam(nodeMap, "Gain", gain.value)) {
        //                 log("Failed to set gain time for camera", camera->first, "to", gain.value, "us.");
        //             }
        //         }

        //         // Set black level.
        //         auto blackLevel = config["settings"]["black_level"].as<Expression>();

        //         if (!utility::vision::setEnumParam(
        //                 nodeMap,
        //                 "BlackLevelAuto",
        //                 ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "Off" : "Continuous"))) {
        //             log("Failed to set auto black level for camera",
        //                 camera->first,
        //                 "to",
        //                 ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "'Off'" : "'Continuous'"));
        //         }

        //         if (std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) {
        //             if (!utility::vision::setNumericParam(nodeMap, "BlackLevel", blackLevel.value)) {
        //                 log("Failed to set black level for camera", camera->first, "to", blackLevel.value);
        //             }
        //         }

        //         // Set acquisition mode to continuous
        //         if (!utility::vision::setEnumParam(nodeMap, "AcquisitionMode", "Continuous")) {
        //             log("Failed to set acquisition mode for camera", camera->first, "to 'Continuous'.");
        //         }

        //         log("Camera",
        //             camera->first,
        //             "with local id",
        //             camera->second->cameraID,
        //             "set to continuous acquisition mode.");

        //         // Setup the event handler for image acquisition.
        //         camera->second->camera->RegisterEvent(*camera->second);

        //         log("Camera", camera->first, "with local id", camera->second->cameraID, "image event handler
        //         registered.");

        //         // Begin acquisition.
        //         camera->second->camera->BeginAcquisition();

        //         log("Camera", camera->first, "with local id", camera->second->cameraID, "image acquisition
        //         started.");
        //     }

        //     catch (Spinnaker::Exception& ex) {
        //         log<NUClear::WARN>(ex.what());
        //         log<NUClear::WARN>(ex.GetFullErrorMessage());
        //         log<NUClear::WARN>(ex.GetFileName());
        //         log<NUClear::WARN>(ex.GetFunctionName());
        //         log<NUClear::WARN>(ex.GetBuildDate());
        //         log<NUClear::WARN>(ex.GetBuildTime());
        //         log<NUClear::WARN>(ex.GetLineNumber());
        //     }
    }

    SpinnakerImageEvent::SpinnakerImageEvent(const std::string& name,
                                             const std::string& serialNumber,
                                             Spinnaker::CameraPtr&& camera,
                                             NUClear::Reactor& reactor,
                                             const utility::vision::FOURCC& fourcc,
                                             int cameraID,
                                             bool isLeft)
        : name(name)
        , serialNumber(serialNumber)
        , camera(std::move(camera))
        , reactor(reactor)
        , fourcc(fourcc)
        , cameraID(cameraID)
        , isLeft(isLeft) {}

    SpinnakerImageEvent::~SpinnakerImageEvent() {
        if (camera) {
            if (camera->IsStreaming()) {
                camera->EndAcquisition();
            }

            while (camera->GetNumImagesInUse() > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }

            camera->UnregisterEvent(*this);
            camera->DeInit();
        }
    }

    void SpinnakerImageEvent::OnImageEvent(Spinnaker::ImagePtr image) {
        // We have a complete image, emit it.
        if (!image->IsIncomplete()) {
            auto msg           = std::make_unique<ImageData>();
            msg->timestamp     = NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp()));
            msg->format        = static_cast<uint32_t>(fourcc);
            msg->serial_number = serialNumber;
            msg->camera_id     = cameraID;
            msg->dimensions << image->GetWidth(), image->GetHeight();
            msg->data.insert(msg->data.end(),
                             static_cast<uint8_t*>(image->GetData()),
                             static_cast<uint8_t*>(image->GetData()) + image->GetBufferSize());
            msg->isLeft = isLeft;

            reactor.emit<NUClear::dsl::word::emit::Direct>(msg);
        }

        if (image->GetWidth() > 10000 || image->GetHeight() > 10000) {
            NUClear::log<NUClear::ERROR>("Spinnaker Camera ",
                                         __FILE__,
                                         " (Line ",
                                         __LINE__,
                                         ")",
                                         "BAD IMAGE INITIALISATION - SHUTTING DOWN (TODO: handle better)");
            reactor.powerplant.shutdown();
        }
    }

    void SpinnakerLogCallback::OnLogEvent(Spinnaker::LoggingEventDataPtr loggingEventDataPtr) {
        switch (loggingEventDataPtr->GetPriority()) {
            case Spinnaker::LOG_LEVEL_ERROR: {
                reactor.log<NUClear::ERROR>(loggingEventDataPtr->GetLogMessage());
                break;
            }
            case Spinnaker::LOG_LEVEL_WARN: {
                reactor.log<NUClear::WARN>(loggingEventDataPtr->GetLogMessage());
                break;
            }
            case Spinnaker::LOG_LEVEL_NOTICE:
            case Spinnaker::LOG_LEVEL_INFO: {
                reactor.log<NUClear::INFO>(loggingEventDataPtr->GetLogMessage());
                break;
            }
#ifndef NDEBUG
            case Spinnaker::LOG_LEVEL_DEBUG: {
                reactor.log<NUClear::DEBUG>(loggingEventDataPtr->GetLogMessage());
                break;
            }
#endif
            default: break;
        }
    }


    void Camera::ShutdownSpinnakerCamera() {
        SpinnakerCameras.clear();
        SpinnakerCamList.Clear();

        if (SpinnakerSystem) {
            SpinnakerSystem->UnregisterLoggingEvent((Spinnaker::LoggingEvent&) (SpinnakerLoggingCallback));
            SpinnakerSystem->ReleaseInstance();
        }
    }
}  // namespace input
}  // namespace module
