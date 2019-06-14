#include "ImageCompressor.h"

#include "extension/Configuration.h"
#include "message/output/CompressedImage.h"
#include "utility/vision/fourcc.h"

namespace module {
namespace output {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;
    using utility::vision::fourcc;

    void ImageCompressor::compress(const Image& image, const TJPF& format) {

        // A compressor per thread
        static thread_local tjhandle compressor = tjInitCompress();

        long unsigned int jpeg_size = 0;
        uint8_t* compressed         = nullptr;

        tjCompress2(compressor,
                    image.data.data(),
                    image.dimensions.x(),
                    0,
                    image.dimensions.y(),
                    format,
                    &compressed,
                    &jpeg_size,
                    TJSAMP_444,  // Output chromiance sampling
                    quality,
                    TJFLAG_FASTDCT);

        auto msg = std::make_unique<CompressedImage>();

        msg->format            = fourcc("JPEG");
        msg->dimensions.x()    = image.dimensions.x();
        msg->dimensions.y()    = image.dimensions.y();
        msg->camera_id         = image.camera_id;
        msg->name              = image.name;
        msg->timestamp         = image.timestamp;
        msg->Hcw               = image.Hcw;
        msg->lens.projection   = int(image.lens.projection);
        msg->lens.focal_length = image.lens.focal_length;
        msg->lens.fov          = image.lens.fov;

        msg->data.assign(compressed, compressed + jpeg_size);

        emit(msg);

        tjFree(compressed);
    }

    ImageCompressor::ImageCompressor(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Configuration>("ImageCompressor.yaml").then([this](const Configuration& config) {
            // Store our config
            quality = config["quality"];
        });

        on<Trigger<Image>, Buffer<4>>().then([this](const Image& image) {
            // If this is the first time using the compressor, we need to make one
            switch (image.format) {
                // Bayer formats
                case fourcc("BGGR"):
                case fourcc("RGGB"):
                case fourcc("GRBG"):
                case fourcc("GBRG"): {
                    // TODO compress 4 JPEG images, one for each quadrant
                } break;

                // Regular convertible formats
                case fourcc("RGB3"):
                case fourcc("RGB8"): compress(image, TJPF_RGB); break;
                case fourcc("BGR8"): compress(image, TJPF_BGR); break;
                case fourcc("GRAY"):
                case fourcc("GREY"): compress(image, TJPF_GRAY); break;
            }
        });
    }  // namespace output
}  // namespace output
}  // namespace module
