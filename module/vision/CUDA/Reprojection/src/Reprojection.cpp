#include <fmt/format.h>
#include <cmath>
#include <vector>

#include "Reprojection.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/vision/ReprojectedImage.h"

#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {
    namespace CUDA {

        using extension::Configuration;

        using message::input::CameraParameters;
        using message::input::Image;
        using message::vision::ReprojectedImage;

        using utility::math::vision::pinhole::getCamFromScreen;
        using utility::math::vision::radial::projectCamSpaceToScreen;

        Reprojection::Reprojection(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), dump_images(false), avg_fp_ms(), avg_count(0), device() {

            on<Configuration>("Reprojection.yaml").then([this](const Configuration& config) {
                arma::vec dimensions = config["output"]["dimensions"].as<arma::vec>();
                output_dimensions    = arma::conv_to<arma::uvec>::from(dimensions);
                tan_half_FOV         = std::tan(config["output"]["FOV"].as<double>() * M_PI / 360.0);
                dump_images          = config["dump_images"].as<bool>();
                int num_devices;

                // Get a count of the number of devices available on the system.
                if (cudaCheckError(cudaGetDeviceCount(&num_devices)) || (num_devices < 1)) {
                    if (num_devices < 1) {
                        std::cerr << fmt::format("Did not find any CDUA compatible devices on system!.");
                    }

                    return;
                }

                device = 0;

                // Make sure the device is recent enough.
                if ((CUDA_VERSION < 4000) || (getCudaComputeCapability(device).first <= 1)) {
                    std::cerr << fmt::format("Error: CUDA device is not capable of handling 2D textures.") << std::endl;
                }

                // Set the device we want to use.
                if (cudaCheckError(cudaSetDevice(device))) {
                    return;
                }
            });

            on<Trigger<Image>, With<CameraParameters>, Single>().then(
                "Image Reprojection", [this](const Image& image, const CameraParameters& cam) {
                    if (image.camera_id < 2) {
                        // Set the device we want to use.
                        if (cudaCheckError(cudaSetDevice(device))) {
                            log<NUClear::ERROR>(fmt::format("Failed to set CUDA device to {}.", device));
                            return;
                        }

                        // For benchmarking.
                        auto start = NUClear::clock::now();

                        // Figure out cameras focal length in pixels.
                        arma::vec2 input_center     = {(image.dimensions.x() - 1.0) * 0.5,
                                                   (image.dimensions.y() - 1.0) * 0.5};
                        arma::vec2 output_center    = arma::conv_to<arma::vec>::from(output_dimensions - 1.0) * 0.5;
                        double camFocalLengthPixels = arma::norm(output_center) / tan_half_FOV;

                        // Create our output message.
                        ReprojectedImage msg;
                        msg.format        = utility::vision::FOURCC::RGB3;
                        msg.dimensions    = convert<unsigned int, 2>(output_dimensions);
                        msg.camera_id     = image.camera_id + 2;
                        msg.serial_number = image.serial_number;
                        msg.timestamp     = image.timestamp;
                        msg.Hcw           = image.Hcw;
                        msg.data          = std::vector<uint8_t>(output_dimensions[0] * output_dimensions[1] * 3, 0);

                        // Invoke kernel.
                        if (cudaCheckError(launchKernel(image.data.data(),
                                                        image.format,
                                                        cam.radial.radiansPerPixel,
                                                        make_uint2(image.dimensions.x(), image.dimensions.y()),
                                                        make_uint2(output_dimensions[0], output_dimensions[1]),
                                                        camFocalLengthPixels,
                                                        msg.data.data()))) {
                            log<NUClear::ERROR>("Failed to run CUDA reprojection kernel.");
                            return;
                        }

                        if (cudaCheckError(cudaDeviceSynchronize())) {
                            log<NUClear::ERROR>("Failed to synchronise CUDA device.");
                            return;
                        }

                        // Dump image to file.
                        if (dump_images) {
                            utility::vision::saveImage("reprojected_image.ppm", msg);
                        }

                        // Emit our reprojected image.
                        emit(std::make_unique<ReprojectedImage>(msg));

                        auto end   = NUClear::clock::now();
                        auto fp_ms = std::chrono::duration<double, std::milli>(end - start);
                        avg_fp_ms += fp_ms;
                        avg_count++;
                        if (((avg_count - 1) % 100) == 0) {
                            log<NUClear::INFO>(fmt::format("Image reprojection time: {0:.4f} ms (avg: {1:.4f} ms)",
                                                           fp_ms.count(),
                                                           (avg_fp_ms / avg_count).count()));
                        }
                    }
                });
        }

        bool Reprojection::cudaCheckError(const cudaError_t& err) {
            if (cudaSuccess != err) {
                log<NUClear::ERROR>(fmt::format("CUDA ERROR: {}", cudaGetErrorString(err)));
                return true;
            }

            return false;
        }

        std::pair<int, int> Reprojection::getCudaComputeCapability(int device) {
            cudaDeviceProp properties;
            if (cudaCheckError(cudaGetDeviceProperties(&properties, device))) {
                return std::make_pair(0, 0);
            }

            return std::make_pair(properties.major, properties.minor);
        }

    }  // namespace CUDA
}  // namespace vision
}  // namespace module
