#include "SpinnakerCamera.h"

#include <cmath>
#include <armadillo>

#include "extension/Configuration.h"
#include "utility/support/yaml_expression.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/Spinnaker.h"

namespace module {
namespace input {

    using extension::Configuration;
    using utility::support::Expression;
    using namespace utility::vision;

    SpinnakerCamera::SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), system(Spinnaker::System::GetInstance()), camList(system->GetCameras(true, true)), cameras() {

        on<Shutdown>().then("SpinnakerCamera Shutdown", [this] {
            NUClear::log("SpinnakerCamera Shutdown");

            cameras.clear();

            if (system)
            {
                system->ReleaseInstance();
            }
        });

        on<Configuration>("Cameras").then([this] (const Configuration& config)
        {
            log<NUClear::DEBUG>("Found ", camList.GetSize(), " cameras.");

            if (camList.GetSize() < 1)
            {
                return;
            }

            std::string serialNumber = config["device"]["serial"].as<std::string>();

            log<NUClear::DEBUG>("Processing camera", config.fileName, "with serial number", serialNumber);

            // See if we already have this camera
            auto camera = cameras.find(serialNumber);

            if (camera == cameras.end())
            {
                try 
                {
                    Spinnaker::CameraPtr newCamera = camList.GetBySerial(serialNumber);

                    // Ensure we found the camera.
                    if (newCamera)
                    {
                        // Initlise the camera.
                        newCamera->Init();

                        // Add camera to list.
                        FOURCC fourcc = getFourCCFromDescription(config["format"]["pixel"].as<std::string>());
                        camera = cameras.insert(std::make_pair(serialNumber, std::make_unique<ImageEvent>(config.fileName, serialNumber, std::move(newCamera), *this, fourcc))).first;
                        log<NUClear::DEBUG>("Camera ", serialNumber, " added to map.");
                    }

                    else
                    {
                        log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number: ", serialNumber);
                        return;
                    }
                }

                catch(const Spinnaker::Exception& ex) 
                {
                    log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number: ", serialNumber);
                    return;
                }
            }

            else
            {
                camera->second->camera->EndAcquisition();
            }

            // Get device node maps
            auto& nodeMap  = camera->second->camera->GetNodeMap();
            auto& sNodeMap = camera->second->camera->GetTLStreamNodeMap();

            // Set the width and height of the image.
            if (!setNumericParam(sNodeMap, "StreamDefaultBufferCount", config["buffer_count"].as<int64_t>()))
            {
                log("Failed to set software buffer count for camera", camera->first, "to", config["buffer_count"].as<int64_t>());
            }

            // Set the pixel format.
            if(!setEnumParam(nodeMap, "PixelFormat", config["format"]["pixel"].as<std::string>()))
            {
                log("Failed to set pixel format for camera", camera->first, "to", config["format"]["pixel"].as<int64_t>());
            }

            // Set the width and height of the image.
            if (!setNumericParam(nodeMap, "Width", config["format"]["width"].as<int64_t>()))
            {
                log("Failed to set image width for camera", camera->first, "to", config["format"]["width"].as<int64_t>());
            }

            if (!setNumericParam(nodeMap, "Height", config["format"]["height"].as<int64_t>()))
            {
                log("Failed to set image height for camera", camera->first, "to", config["format"]["height"].as<int64_t>());
            }

            // Set exposure.
            auto exposure = config["settings"]["exposure"].as<Expression>();

            if (!setEnumParam(nodeMap, "ExposureAuto", (std::isfinite(exposure.value) ? "Off" : "Continuous")))
            {
                log("Failed to set auto exposure mode for camera", camera->first, "to", (std::isfinite(exposure.value) ? "'Off'" : "'Continuous'"));
            }

            if (std::isfinite(exposure.value))
            {
                if (!setEnumParam(nodeMap, "ExposureMode", "Timed"))
                {
                    log("Failed to set exposure mode for camera", camera->first, "to 'Timed'");
                }

                if (!setNumericParam(nodeMap, "ExposureTime", exposure.value))
                {
                    log("Failed to set exposure time for camera", camera->first, "to", exposure.value, "us.");
                }
            }

            // Set gain.
            auto gain = config["settings"]["gain"].as<Expression>();

            if (!setEnumParam(nodeMap, "GainAuto", (std::isfinite(gain.value) ? "Off" : "Continuous")))
            {
                log("Failed to set auto gain mode for camera", camera->first, "to", (std::isfinite(gain.value) ? "'Off'" : "'Continuous'"));
            }

            if (std::isfinite(gain.value))
            {
                if (!setNumericParam(nodeMap, "Gain", gain.value))
                {
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
                if (!setNumericParam(nodeMap, "Gamma", gamma))
                {
                    log("Failed to set gamma for camera", camera->first, "to", gamma);
                }
            }
            */

            // Set black level.
            auto blackLevel = config["settings"]["black_level"].as<Expression>();

            if (!setEnumParam(nodeMap, "BlackLevelAuto", ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "Off" : "Continuous")))
            {
                log("Failed to set auto black level for camera", camera->first, "to", ((std::isfinite(blackLevel.value) && (blackLevel.value != 0.0)) ? "'Off'" : "'Continuous'"));
            }

            if (std::isfinite(blackLevel.value) && (blackLevel.value != 0.0))
            {
                if (!setNumericParam(nodeMap, "BlackLevel", blackLevel.value))
                {
                    log("Failed to set black level for camera", camera->first, "to", blackLevel.value);
                }
            }

            /*
            // Set sharpness.
            auto sharpness = config["settings"]["sharpness"].as<Expression>();

            if (!setEnumParam(nodeMap, "SharpnessAuto", (std::isfinite(sharpness.value) ? "Off" : "Continuous")))
            {
                log("Failed to set auto sharpness for camera", camera->first, "to", (std::isfinite(sharpness.value) ? "'Off'" : "'Continuous'"));
            }

            if (std::isfinite(sharpness.value))
            {
                if (!setNumericParam(nodeMap, "Sharpness", sharpness.value))
                {
                    log("Failed to set sharpness for camera", camera->first, "to", sharpness.value);
                }
            }

            // Set hue.
            auto hue = config["settings"]["hue"].as<Expression>();

            if (!setEnumParam(nodeMap, "SharpnessAuto", (std::isfinite(hue.value) ? "Off" : "Continuous")))
            {
                log("Failed to set auto hue for camera", camera->first, "to", (std::isfinite(hue.value) ? "'Off'" : "'Continuous'"));
            }

            if (std::isfinite(hue.value))
            {
                if (!setNumericParam(nodeMap, "Hue", hue.value))
                {
                    log("Failed to set hue for camera", camera->first, "to", hue.value);
                }
            }
            */

            /*
            // Set saturation.
            auto saturation = config["settings"]["saturation"].as<Expression>();

            if (!setEnumParam(nodeMap, "SaturationAuto", (std::isfinite(saturation.value) ? "Off" : "Continuous")))
            {
                log("Failed to set auto saturation for camera", camera->first, "to", (std::isfinite(saturation.value) ? "'Off'" : "'Continuous'"));

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
                if (!setNumericParam(nodeMap, "Saturation", saturation.value))
                {
                    log("Failed to set saturation for camera", camera->first, "to", saturation.value);
                }
            }

            // Set white balance.
            // int64_t to be (trivially) type compatible with Spinnaker libray.
            arma::Col<int64_t> whiteBalance = config["settings"]["white_balance"].as<arma::Col<int64_t>>();

            if (!setEnumParam(nodeMap, "BalanceWhiteAuto", (arma::all(whiteBalance) ? "Off" : "Continuous")))
            {
                log("Failed to set auto white balance for camera", camera->first, "to", (arma::all(whiteBalance) ? "'Off'" : "'Continuous'"));

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
                if (!setEnumParam(nodeMap, "BalanceRatioSelector", "Red"))
                {
                    log("Failed to set white balance ratio selector for camera", camera->first, "to 'Red (V)'");
                }

                if (!setNumericParam(nodeMap, "BalanceRatio", whiteBalance[0]))
                {
                    log("Failed to set saturation for camera", camera->first, "to", whiteBalance[0]);
                }

                if (!setEnumParam(nodeMap, "BalanceRatioSelector", "Blue"))
                {
                    log("Failed to set white balance ratio selector for camera", camera->first, "to 'Blue (U)'");
                }

                if (!setNumericParam(nodeMap, "BalanceRatio", whiteBalance[1]))
                {
                    log("Failed to set saturation for camera", camera->first, "to", whiteBalance[1]);
                }
            }
            */

            // Set acquisition mode to continuous
            if(!setEnumParam(nodeMap, "AcquisitionMode", "Continuous"))
            {
                log("Failed to set acquisition mode for camera", camera->first, "to 'Continuous'.");
            }

            log("Camera ", camera->first, " set to continuous acquisition mode.");

            // Setup the event handler for image acquisition.
            camera->second->camera->RegisterEvent(*camera->second);

            log("Camera ", camera->first, " image event handler registered.");

            // Begin acquisition.
            camera->second->camera->BeginAcquisition();

            log("Camera ", camera->first, " image acquisition started.");
        });
    }

}
}
