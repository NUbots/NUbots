#include "Compressor.hpp"

#include <fmt/format.h>
#include <turbojpeg.h>

#include "utility/vision/fourcc.hpp"

namespace module::output::compressor::turbojpeg {

    Compressor::Compressor(const int& quality, const uint32_t& width, const uint32_t& height, const uint32_t& format)
        : quality(quality), width(width), height(height), format(format) {

        // If this is a mosaic format, build a mosaic table
        if (utility::vision::Mosaic::size(format) > 1) {
            mosaic = utility::vision::Mosaic(width, height, format);
        }
    }

    std::vector<uint8_t> Compressor::compress(const std::vector<uint8_t>& data) {
        TJSAMP tj_sampling = TJSAMP_444;
        TJPF tj_format     = TJPF_RGB;

        switch (format) {
            case utility::vision::fourcc("BGGR"):
            case utility::vision::fourcc("RGGB"):
            case utility::vision::fourcc("GRBG"):
            case utility::vision::fourcc("GBRG"):
            case utility::vision::fourcc("PY  "):
            case utility::vision::fourcc("PBG8"):
            case utility::vision::fourcc("PRG8"):
            case utility::vision::fourcc("PGR8"):
            case utility::vision::fourcc("PGB8"):
            case utility::vision::fourcc("GRAY"):
            case utility::vision::fourcc("GREY"):
            case utility::vision::fourcc("Y8  "):
            case utility::vision::fourcc("Y16 "):
                tj_sampling = TJSAMP_GRAY;
                tj_format   = TJPF_GRAY;
                break;
            case utility::vision::fourcc("BGR8"):
            case utility::vision::fourcc("BGR3"):
                tj_sampling = TJSAMP_444;
                tj_format   = TJPF_BGR;
                break;
            case utility::vision::fourcc("BGRA"):
                tj_sampling = TJSAMP_444;
                tj_format   = TJPF_BGRA;
                break;
            case utility::vision::fourcc("RGB8"):
            case utility::vision::fourcc("RGB3"):
                tj_sampling = TJSAMP_444;
                tj_format   = TJPF_RGB;
                break;
            case utility::vision::fourcc("RGBA"):
                tj_sampling = TJSAMP_444;
                tj_format   = TJPF_RGBA;
                break;
            default:
                throw std::runtime_error(fmt::format("Format {} is not supported by the turbojpeg compressor",
                                                     utility::vision::fourcc(format)));
        }

        // A compressor per thread
        // tjInitCompress returns and allocated tjHandle (which is a void*)
        auto deleter = [](void* ptr) {
            if (ptr != nullptr) {
                tjDestroy(ptr);
            }
        };
        static thread_local std::unique_ptr<void, decltype(deleter)> compressor(tjInitCompress(), deleter);

        unsigned long jpeg_size = 0;  // NOLINT(google-runtime-int) this is matching JPEG Turbo api
        uint8_t* compressed     = nullptr;

        // Downcast 16 bit greyscale
        if (format == utility::vision::fourcc("Y16 ")) {
            std::vector<uint8_t> cast(width * height);
            for (uint32_t i = 0; i < width * height; ++i) {
                cast[i] = data[i * 2 + 1];
            }

            tjCompress2(compressor.get(),
                        cast.data(),
                        int(width),
                        0,
                        int(height),
                        tj_format,
                        &compressed,
                        &jpeg_size,
                        tj_sampling,  // Output chrominance sampling
                        quality,
                        TJFLAG_FASTDCT);
        }
        // Mosaic table to rearrange with
        else if (mosaic) {
            // Permute our bytes into a new order
            std::vector<uint8_t> permuted = mosaic.permute(data);

            tjCompress2(compressor.get(),
                        permuted.data(),
                        int(width),
                        0,
                        int(height),
                        tj_format,
                        &compressed,
                        &jpeg_size,
                        tj_sampling,  // Output chrominance sampling
                        quality,
                        TJFLAG_FASTDCT);
        }
        else {
            tjCompress2(compressor.get(),
                        data.data(),
                        int(width),
                        0,
                        int(height),
                        tj_format,
                        &compressed,
                        &jpeg_size,
                        tj_sampling,  // Output chrominance sampling
                        quality,
                        TJFLAG_FASTDCT);
        }

        std::vector<uint8_t> output(compressed, compressed + jpeg_size);
        tjFree(compressed);

        return output;
    }

}  // namespace module::output::compressor::turbojpeg
