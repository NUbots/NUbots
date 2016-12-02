#include "SpinnakerCamera.h"

#include "message/support/Configuration.h"

namespace module {
namespace input {

    using message::support::Configuration;
    using namespace utility::vision;

    SpinnakerCamera::SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), system(Spinnaker::System::GetInstance()), camList(system->GetCameras(true, true)), cameras() {

        on<Shutdown>().then("SpinnakerCamera Shutdown", [this] {
            cameras.clear();

            if (system)
            {
                system->ReleaseInstance();
            }
        });

        on<Configuration>("Cameras").then([this] (const Configuration& config)
        {
            log("Found ", camList.GetSize(), " cameras.");

            if (camList.GetSize() < 1)
            {
                return;
            }

            std::string serialNumber = config["device"]["serial"].as<std::string>();

            log("Processing camera with serial number ", serialNumber);

            // See if we already have this camera
            auto camera = cameras.find(serialNumber);

            if (camera == cameras.end())
            {
                Spinnaker::CameraPtr newCamera = camList.GetBySerial(serialNumber);

                // Ensure we found the camera.
                if (newCamera)
                {
                    // Initlise the camera.
                    newCamera->Init();

                    // Add camera to list.
                    FOURCC fourcc = getFourCCFromDescription(config["format"]["pixel"].as<std::string>());
                    camera = cameras.insert(std::make_pair(serialNumber, std::make_unique<ImageEvent>(serialNumber, std::move(newCamera), *this, fourcc))).first;
                    log("Camera ", serialNumber, " added to map.");
                }

                else
                {
                    log("Failed to find camera with serial number: ", serialNumber);
                    return;
                }
            }

            else
            {
                camera->second->camera->EndAcquisition();
            }

            // Get device node map.
            auto& nodeMap = camera->second->camera->GetNodeMap();

            // Set the pixel format.
            Spinnaker::GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");

            if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
            {
                // Retrieve the desired entry node from the enumeration node
                std::string format = config["format"]["pixel"].as<std::string>();
                Spinnaker::GenApi::CEnumEntryPtr newPixelFormat = ptrPixelFormat->GetEntryByName(format.c_str());

                if (IsAvailable(newPixelFormat) && IsReadable(newPixelFormat))
                {
                    ptrPixelFormat->SetIntValue(newPixelFormat->GetValue());
                    
                    log("Pixel format for camera ", camera->first," set to ", format);
                }

                else
                {
                    log("Failed to set pixel format to ", format, " for camera ", camera->first);
                    log("PixelFormat enum entry is '", format, "' available? ", IsAvailable(newPixelFormat) ? "yes" : "no");
                    log("PixelFormat enum entry is '", format, "' writable? ", IsWritable(newPixelFormat) ? "yes" : "no");
                }
            }

            else
            {
                log("Failed to retrieve pixel format for camera ", camera->first);
                log("PixelFormat enum entry is available? ", IsAvailable(ptrPixelFormat) ? "yes" : "no");
                log("PixelFormat enum entry is writable? ", IsWritable(ptrPixelFormat) ? "yes" : "no");
            }

            // Set the width and height of the image.
            Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");

            if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
            {
                int64_t width = config["format"]["width"].as<int>();

                // Ensure the width is a multiple of the increment.
                if ((width % ptrWidth->GetInc()) != 0)
                {
                    width = std::min(ptrWidth->GetMax(), std::max(ptrWidth->GetMin(), width - (width % ptrWidth->GetInc())));
                }

                ptrWidth->SetValue(width);
                log("Image width for camera ", camera->first," set to ", width);
            }

            else
            {
                log("Failed to retrieve image width for camera ", camera->first);
                log("Width entry is available? ", IsAvailable(ptrWidth) ? "yes" : "no");
                log("Width entry is writable? ", IsWritable(ptrWidth) ? "yes" : "no");
            }

            Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");

            if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
            {
                int64_t height = config["format"]["height"].as<int>();

                // Ensure the height is a multiple of the increment.
                if ((height % ptrHeight->GetInc()) != 0)
                {
                    height = std::min(ptrHeight->GetMax(), std::max(ptrHeight->GetMin(), height - (height % ptrHeight->GetInc())));
                }

                ptrHeight->SetValue(height);
                log("Image height for camera ", camera->first," set to ", height);
            }

            else
            {
                log("Failed to retrieve image height for camera ", camera->first);
                log("Height entry is available? ", IsAvailable(ptrHeight) ? "yes" : "no");
                log("Height entry is writable? ", IsWritable(ptrHeight) ? "yes" : "no");
            }

            // Set acquisition mode to continuous
            Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");

            if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
            {
                log("Failed to retrieve acquisition mode for camera ", camera->first);
                return;
            }

            Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
            
            if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
            {
                log("Failed to retrieve continuous acquisition mode entry for camera ", camera->first);
                return;
            }

            ptrAcquisitionMode->SetIntValue(ptrAcquisitionModeContinuous->GetValue());

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
