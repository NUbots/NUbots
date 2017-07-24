#include "Masking.h"

#include "extension/Configuration.h"
#include "utility/vision/ImageMasking.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;
    using message::input::CameraParameterSet;
    using utility::vision::MaskClass;
    using message::vision::ImageMask;

    Masking::Masking(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Masking.yaml").then([this](const Configuration& config) {
            image_extension = config["image_extension"].as<std::string>();
        });

        on<Trigger<CameraParameterSet>>().then([this](const CameraParameterSet& set) {
            // Use configuration here from file ImageMask.yaml
            auto masks = std::make_unique<std::map<std::string, ImageMask>>();
            for (auto& c : set.cams) {
                bool success   = false;
                ImageMask mask = createMaskForCamera(c.second, success);
                if (success) {
                    (*masks)[mask.cameraName] = mask;
                }
            }
            emit(masks);
        });
    }

    ImageMask Masking::createMaskForCamera(const CameraParameters& cam, bool& success) {
        std::string filename = "config/" + cam.cameraName + "Mask." + image_extension;
        log("Loading camera mask from file", filename);

        ImageMask mask;
        success = false;

        // Open and check file
        std::ifstream file(filename);
        if (file.is_open()) {
            std::string buff;
            std::getline(file, buff);
            if (buff.compare("P1") != 0) {
                log<NUClear::ERROR>("Mask ", filename, " is wrong format.");
                return mask;
            }
            // Load data
            std::getline(file, buff);
            int width, height;
            file >> width >> height;
            // If wrong size
            if (width != cam.imageSizePixels[0] || width != cam.imageSizePixels[0]) {
                log<NUClear::ERROR>("Mask ",
                                    filename,
                                    " has size ",
                                    width,
                                    height,
                                    " but camera settings are ",
                                    cam.imageSizePixels[0],
                                    cam.imageSizePixels[1]);
                return mask;
            }
            mask.values.resize(width, height);
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    int valid;
                    file >> valid;
                    mask.values(i, j) = char(valid > 0 ? MaskClass::UNMASKED : MaskClass::MASKED);
                }
            }
        }
        else {
            return mask;
        }
        mask.cameraName = cam.cameraName;

        log("Loading succeeded!");
        success = true;
        return mask;
    }

}  // namespace vision
}  // namespace module
