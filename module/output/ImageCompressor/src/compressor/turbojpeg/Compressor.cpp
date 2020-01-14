#include "Compressor.h"

#include <fmt/format.h>
#include <turbojpeg.h>

#include "../mosaic.h"
#include "utility/vision/fourcc.h"

namespace module {
namespace output {
    namespace compressor {
        namespace turbojpeg {

            Compressor::Compressor(const int& quality,
                                   const uint32_t& width,
                                   const uint32_t& height,
                                   const uint32_t& format)
                : quality(quality) {

                // If this is a mosaic format, build a mosaic table
                if (mosaic::mosaic_size(format) > 0) {
                    mosaic_table = mosaic::build_table(width, height, format);
                }
            }
            Compressor::~Compressor() {}

            std::vector<uint8_t> Compressor::compress(const std::vector<uint8_t>& data,
                                                      const uint32_t& width,
                                                      const uint32_t& height,
                                                      const uint32_t& format) {
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
                static thread_local tjhandle compressor = tjInitCompress();

                long unsigned int jpeg_size = 0;
                uint8_t* compressed         = nullptr;

                // Downcast 16 bit greyscale
                if (format == utility::vision::fourcc("Y16 ")) {
                    std::vector<uint8_t> cast(width * height);
                    for (uint32_t i = 0; i < width * height; ++i) {
                        cast[i] = data[i * 2 + 1];
                    }

                    tjCompress2(compressor,
                                cast.data(),
                                width,
                                0,
                                height,
                                tj_format,
                                &compressed,
                                &jpeg_size,
                                tj_sampling,  // Output chrominance sampling
                                quality,
                                TJFLAG_FASTDCT);
                }
                // No mosaic table to rearrange with
                else if (mosaic_table.empty()) {
                    tjCompress2(compressor,
                                data.data(),
                                width,
                                0,
                                height,
                                tj_format,
                                &compressed,
                                &jpeg_size,
                                tj_sampling,  // Output chrominance sampling
                                quality,
                                TJFLAG_FASTDCT);
                }
                else {
                    // Permute our bytes into a new order
                    std::vector<uint8_t> permuted(width * height);
                    for (uint32_t i = 0; i < data.size(); ++i) {
                        permuted[mosaic_table[i]] = data[i];
                    }

                    tjCompress2(compressor,
                                permuted.data(),
                                width,
                                0,
                                height,
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

        }  // namespace turbojpeg
    }      // namespace compressor
}  // namespace output
}  // namespace module
