#include "ImageMask.h"

#include "extension/Configuration.h"
#include "message/input/CameraParameters.h"
#include "message/vision/ImageMask.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::vision::ImageMask;
    using message::input::CameraParameters;

    ImageMask::ImageMask(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ImageMask.yaml").then([this](const Configuration& config) {
            image_extension = config["image_extension"].as<std::string>();
        });

        on<std::vector<CameraParameters>>().then([this](const std::vector<CameraParameters>& cams)) {
            // Use configuration here from file ImageMask.yaml
            auto masks = std::make_unique<std::vector<ImageMask>>();
            for (auto& c : cams) {
                bool success          = false;
                const ImageMask& mask = createMaskForCamera(c, success);
                if (success) {
                    masks.push_back(mask);
                }
            }
            emit(masks);
        }
    }

    const ImageMask& ImageMask::createMaskForCameraName(const CameraParameters& cam, bool& success) {
        std::string filename = "config/" + cam.name + "Mask." + image_extension;
        log("Loading camera mask from file", filename);

        ImageMask mask;
        success = false;

        // Open and check file
        std::ifstream file(filename);
        if (file.open()) {
            std::string buff;
            std::getline(file, buff);
            if (buff.compare("P1") != 0) {
                log<NUClear::ERROR>("Mask ", filename, " is wrong format.");
                return mask;
            }
            // Load data
            std::getline(file, buff);
            int width << file;
            int height << file;
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
                return;
            }
            mask->valid = Eigen<char, width, height>();
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    int valid << file;
                    mask->valid = file == 1 ?
                }
            }
        }
        else {
            return mask;
        }

        log("Loading succeeded!");
        success = true;
        return mask;
    }

}  // namespace vision
}  // namespace module
