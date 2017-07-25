#include "Camera.h"

#include <armadillo>
#include <cmath>

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/Spinnaker.h"

namespace module {
namespace input {

    using message::input::CameraParameters;
    using extension::Configuration;
    using utility::support::Expression;
    using FOURCC = utility::vision::FOURCC;

    void Camera::initiateSpinnakerCamera(const Configuration& config) {
        if (!SpinnakerSystem) {
            SpinnakerSystem = Spinnaker::System::GetInstance();
        }

        SpinnakerCamList = SpinnakerSystem->GetCameras(true, true);
        log<NUClear::DEBUG>("Found ", SpinnakerCamList.GetSize(), " cameras.");

        if (SpinnakerCamList.GetSize() < 1) {
            return;
        }

        std::string deviceID = config["deviceID"].as<std::string>();

        log<NUClear::DEBUG>("Processing camera", config.fileName, "with serial number", deviceID);

        // See if we already have this camera
        auto camera = SpinnakerCameras.find(deviceID);

        if (camera == SpinnakerCameras.end()) {
            try {
                Spinnaker::CameraPtr newCamera = SpinnakerCamList.GetBySerial(deviceID);

                // Ensure we found the camera.
                if (newCamera) {
                    // Initlise the camera.
                    newCamera->Init();

                    // Add camera to list.
                    FOURCC fourcc =
                        utility::vision::getFourCCFromDescription(config["format"]["pixel"].as<std::string>());
                    camera =
                        SpinnakerCameras
                            .insert(std::make_pair(deviceID,
                                                   std::make_unique<SpinnakerImageEvent>(config.fileName,
                                                                                         deviceID,
                                                                                         std::move(newCamera),
                                                                                         *this,
                                                                                         fourcc,
                                                                                         cameraCount,
                                                                                         config["isLeft"].as<bool>())))
                            .first;
                }

                else {
                    log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number:", deviceID);
                    return;
                }
            }

            catch (const Spinnaker::Exception& ex) {
                log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number:", deviceID);
                return;
            }
        }

        else {
            if (camera->second->camera->IsStreaming()) {
                camera->second->camera->EndAcquisition();
            }
        }

        resetSpinnakerCamera(camera, config);

        auto cameraParameters = std::make_unique<CameraParameters>();

        // Generic camera parameters
        cameraParameters->imageSizePixels << config["format"]["width"].as<uint>(),
            config["format"]["height"].as<uint>();
        cameraParameters->FOV << config["lens"]["FOV"].as<double>(), config["lens"]["FOV"].as<double>();

        // Radial specific
        cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
        cameraParameters->radial.radiansPerPixel = config["lens"]["radiansPerPixel"].as<float>();
        cameraParameters->centreOffset           = convert<int, 2>(config["lens"]["centreOffset"].as<arma::ivec>());

        emit<Scope::DIRECT>(std::move(cameraParameters));

        log("Emitted radial camera parameters for camera", config["deviceID"].as<std::string>());
    }

    void Camera::resetSpinnakerCamera(std::map<std::string, std::unique_ptr<SpinnakerImageEvent>>::iterator& camera,
                                      const Configuration& config) {
        if (camera->second->camera->IsStreaming()) {
            camera->second->camera->EndAcquisition();
        }

        // Get device node maps
        auto& nodeMap  = camera->second->camera->GetNodeMap();
        auto& sNodeMap = camera->second->camera->GetTLStreamNodeMap();

        // Set the width and height of the image.
        if (!utility::vision::setNumericParam(
                sNodeMap, "StreamDefaultBufferCount", config["buffer_count"].as<int64_t>())) {
            log("Failed to set software buffer count for camera",
                camera->first,
                "to",
                config["buffer_count"].as<int64_t>());
        }

        // Set the pixel format.
        if (!utility::vision::setEnumParam(nodeMap, "PixelFormat", config["format"]["pixel"].as<std::string>())) {
            log("Failed to set pixel format for camera",
                camera->first,
                "to",
                config["format"]["pixel"].as<std::string>());
        }

        // Set the width and height of the image.
        if (!utility::vision::setNumericParam(nodeMap, "Width", config["format"]["width"].as<int64_t>())) {
            log("Failed to set image width for camera", camera->first, "to", config["format"]["width"].as<int64_t>());
        }

        if (!utility::vision::setNumericParam(nodeMap, "Height", config["format"]["height"].as<int64_t>())) {
            log("Failed to set image height for camera", camera->first, "to", config["format"]["height"].as<int64_t>());
        }

        // Set exposure.
        auto exposure = config["settings"]["exposure"].as<Expression>();

        if (!utility::vision::setEnumParam(
                nodeMap, "ExposureAuto", (std::isfinite(exposure.value) ? "Off" : "Continuous"))) {
            log("Failed to set auto exposure mode for camera",
                camera->first,
                "to",
                (std::isfinite(exposure.value) ? "'Off'" : "'Continuous'"));
        }

        if (std::isfinite(exposure.value)) {
            if (!utility::vision::setEnumParam(nodeMap, "ExposureMode", "Timed")) {
                log("Failed to set exposure mode for camera", camera->first, "to 'Timed'");
            }

            if (!utility::vision::setNumericParam(nodeMap, "ExposureTime", exposure.value)) {
                log("Failed to set exposure time for camera", camera->first, "to", exposure.value, "us.");
            }
        }

        // Set gain.
        auto gain = config["settings"]["gain"].as<Expression>();

        if (!utility::vision::setEnumParam(nodeMap, "GainAuto", (std::isfinite(gain.value) ? "Off" : "Continuous"))) {
            log("Failed to set auto gain mode for camera",
                camera->first,
                "to",
                (std::isfinite(gain.value) ? "'Off'" : "'Continuous'"));
        }

        if (std::isfinite(gain.value)) {
            if (!utility::vision::setNumericParam(nodeMap, "Gain", gain.value)) {
                log("Failed to set gain time for camera", camera->first, "to", gain.value, "us.");
            }
        }

        /*
        // Set gamma.
        double gamma = config["settings"]["gamma"].as<double>();

        if (!setBooleanParam(nodeMap, "GammaEnabled", (gamma != 0.0)))
        {
            log("Failed to", ((gamma == 0.0) ? "disable" : "enable"), "gamma for camera", camera->first);
        }

        if (gamma != 0.0)
        {
            if (!utility::vision::setNumericParam(nodeMap, "Gamma", gamma))
            {
                log("Failed to set gamma for camera", camera->first, "to", gamma);
            }
        }
        */

        // Set black level.
        auto blackLevel = config["settings"]["black_level"].as<Expression>();

        if (!utility::vision::setEnumParam(
                nodeMap,
                "BlackLevelAuto",
                ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "Off" : "Continuous"))) {
            log("Failed to set auto black level for camera",
                camera->first,
                "to",
                ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "'Off'" : "'Continuous'"));
        }

        if (std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) {
            if (!utility::vision::setNumericParam(nodeMap, "BlackLevel", blackLevel.value)) {
                log("Failed to set black level for camera", camera->first, "to", blackLevel.value);
            }
        }

        /*
        // Set sharpness.
        auto sharpness = config["settings"]["sharpness"].as<Expression>();

    if (!utility::vision::setEnumParam(nodeMap, "SharpnessAuto", (std::isfinite(sharpness.value) ? "Off" :
    "Continuous")))
        {
        log("Failed to set auto sharpness for camera", camera->first, "to", (std::isfinite(sharpness.value) ?
    "'Off'" : "'Continuous'"));
        }

        if (std::isfinite(sharpness.value))
        {
            if (!utility::vision::setNumericParam(nodeMap, "Sharpness", sharpness.value))
            {
                log("Failed to set sharpness for camera", camera->first, "to", sharpness.value);
            }
        }

        // Set hue.
        auto hue = config["settings"]["hue"].as<Expression>();

        if (!utility::vision::setEnumParam(nodeMap, "SharpnessAuto", (std::isfinite(hue.value) ? "Off" : "Continuous")))
        {
        log("Failed to set auto hue for camera", camera->first, "to", (std::isfinite(hue.value) ? "'Off'" :
    "'Continuous'"));
        }

        if (std::isfinite(hue.value))
        {
            if (!utility::vision::setNumericParam(nodeMap, "Hue", hue.value))
            {
                log("Failed to set hue for camera", camera->first, "to", hue.value);
            }
        }
        */

        /*
        // Set saturation.
        auto saturation = config["settings"]["saturation"].as<Expression>();

    if (!utility::vision::setEnumParam(nodeMap, "SaturationAuto", (std::isfinite(saturation.value) ? "Off" :
    "Continuous")))
        {
        log("Failed to set auto saturation for camera", camera->first, "to", (std::isfinite(saturation.value) ?
    "'Off'" : "'Continuous'"));

            Spinnaker::GenApi::CEnumerationPtr enumName = nodeMap.GetNode("SaturationAuto");

            if (!IsAvailable(enumName))
            {
                log("Auto saturation: enum SaturationAuto not available.");
            }

            if (!IsWritable(enumName))
            {
                log("Auto saturation: enum SaturationAuto not writable.");
            }

            Spinnaker::GenApi::NodeList_t entries;
            enumName->GetEntries(entries);

            for (const auto& entry : entries)
            {
                log("Auto saturation: entry '", entry->GetName(), "'.");
            }
        }

        if (std::isfinite(saturation.value))
        {
            if (!utility::vision::setNumericParam(nodeMap, "Saturation", saturation.value))
            {
                log("Failed to set saturation for camera", camera->first, "to", saturation.value);
            }
        }

        // Set white balance.
        // int64_t to be (trivially) type compatible with Spinnaker libray.
        arma::Col<int64_t> whiteBalance = config["settings"]["white_balance"].as<arma::Col<int64_t>>();

    if (!utility::vision::setEnumParam(nodeMap, "BalanceWhiteAuto", (arma::all(whiteBalance) ? "Off" :
    "Continuous")))
        {
        log("Failed to set auto white balance for camera", camera->first, "to", (arma::all(whiteBalance) ? "'Off'" :
    "'Continuous'"));

            Spinnaker::GenApi::CEnumerationPtr enumName = nodeMap.GetNode("BalanceWhiteAuto");

            if (!IsAvailable(enumName))
            {
                log("Auto white balance: enum BalanceWhiteAuto not available.");
            }

            if (!IsWritable(enumName))
            {
                log("Auto white balance: enum BalanceWhiteAuto not writable.");
            }

            Spinnaker::GenApi::NodeList_t entries;
            enumName->GetEntries(entries);

            for (const auto& entry : entries)
            {
                log("Auto white balance: entry '", entry->GetName(), "'.");
            }
        }

        // If both elements of the vector are non-zero then we are setting a manual value.
        if (arma::all(whiteBalance))
        {
            if (!utility::vision::setEnumParam(nodeMap, "BalanceRatioSelector", "Red"))
            {
                log("Failed to set white balance ratio selector for camera", camera->first, "to 'Red (V)'");
            }

            if (!utility::vision::setNumericParam(nodeMap, "BalanceRatio", whiteBalance[0]))
            {
                log("Failed to set saturation for camera", camera->first, "to", whiteBalance[0]);
            }

            if (!utility::vision::setEnumParam(nodeMap, "BalanceRatioSelector", "Blue"))
            {
                log("Failed to set white balance ratio selector for camera", camera->first, "to 'Blue (U)'");
            }

            if (!utility::vision::setNumericParam(nodeMap, "BalanceRatio", whiteBalance[1]))
            {
                log("Failed to set saturation for camera", camera->first, "to", whiteBalance[1]);
            }
        }
        */

        // Set acquisition mode to continuous
        if (!utility::vision::setEnumParam(nodeMap, "AcquisitionMode", "Continuous")) {
            log("Failed to set acquisition mode for camera", camera->first, "to 'Continuous'.");
        }

        log("Camera", camera->first, "with local id", camera->second->cameraID, "set to continuous acquisition mode.");

        // Setup the event handler for image acquisition.
        camera->second->camera->RegisterEvent(*camera->second);

        log("Camera", camera->first, "with local id", camera->second->cameraID, "image event handler registered.");

        // Begin acquisition.
        camera->second->camera->BeginAcquisition();

        log("Camera", camera->first, "with local id", camera->second->cameraID, "image acquisition started.");
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

        image->Release();
    }

    void Camera::ShutdownSpinnakerCamera() {
        SpinnakerCameras.clear();
        SpinnakerCamList.Clear();

        if (SpinnakerSystem) {
            SpinnakerSystem->ReleaseInstance();
        }
    }
}  // namespace input
}  // namespace module
