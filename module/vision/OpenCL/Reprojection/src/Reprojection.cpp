#include <fmt/format.h>
#include <cmath>
#include <vector>

#include "Reprojection.h"

#include "extension/Configuration.h"
#include "extension/OpenCL.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/vision/ReprojectedImage.h"

#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {
    namespace OpenCL {

        using extension::Configuration;
        using extension::OpenCL;

        using message::input::CameraParameters;
        using message::input::Image;
        using message::vision::ReprojectedImage;

        using utility::math::vision::pinhole::getCamFromScreen;
        using utility::math::vision::radial::projectCamSpaceToScreen;

        Reprojection::Reprojection(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , dump_images(false)
            , avg_fp_ms()
            , avg_count(0)
            , platform()
            , device()
            , context()
            , command_queue()
            , program()
            , use_gpu(false)
            , reproject(true) {

            on<Configuration>("Reprojection.yaml").then([this](const Configuration& config) {
                arma::vec dimensions = config["output"]["dimensions"].as<arma::vec>();
                output_dimensions    = arma::conv_to<arma::uvec>::from(dimensions);
                FOV                  = config["output"]["FOV"].as<double>();
                tan_half_FOV         = std::tan(FOV * M_PI / 360.0);
                dump_images          = config["dump_images"].as<bool>();
                use_gpu              = config["use_gpu"].as<bool>();
                reproject            = config["reproject"].as<bool>();

                // Query the system for OpenCL platforms.
                std::vector<cl::Platform> all_platforms;
                cl::Platform::get(&all_platforms);
                if (all_platforms.empty()) {
                    log<NUClear::ERROR>("No OpenCL platforms found. Check OpenCL Installation");
                    throw std::runtime_error("No OpenCL platforms found. Check OpenCL Installation");
                }

                log<NUClear::INFO>(fmt::format("Found {} platform(s).", all_platforms.size()));

                std::vector<cl::Device> all_devices;

                // Query each platform until we find a CPU/GPU device (dependant on use_gpu).
                for (auto& current_platform : all_platforms) {
                    log<NUClear::INFO>(fmt::format("Probing OpenCL platform {} {} for a {} device ...",
                                                   current_platform.getInfo<CL_PLATFORM_NAME>(),
                                                   current_platform.getInfo<CL_PLATFORM_VERSION>(),
                                                   (use_gpu ? "GPU" : "CPU")));

                    current_platform.getDevices((use_gpu ? CL_DEVICE_TYPE_GPU : CL_DEVICE_TYPE_CPU), &all_devices);

                    // If we found a platform, select it and continue on.
                    if (!all_devices.empty()) {
                        platform = current_platform;
                        break;
                    }

                    all_devices.clear();
                }

                if (all_devices.empty()) {
                    log<NUClear::ERROR>("No devices found. Check OpenCL installation!");
                    throw std::runtime_error("No devices found. Check OpenCL installation!");
                }

                log<NUClear::INFO>(fmt::format("Using OpenCL platform: {} {}",
                                               platform.getInfo<CL_PLATFORM_NAME>(),
                                               platform.getInfo<CL_PLATFORM_VERSION>()));

                // Choose our default device.
                device = all_devices.front();
                log<NUClear::INFO>(fmt::format("Using OpenCL device: {}", device.getInfo<CL_DEVICE_NAME>()));

                // Make a context for this device
                context = cl::Context({device});

                // Create two queues, one for memory transfers and one for execution
                command_queue = cl::CommandQueue(context, device);
            });

            // Build our OpenCL kernel
            on<OpenCL>("reprojection.cl", context).then([this](const OpenCL& ocl) {
                program = ocl.program;

                debayer = cl::KernelFunctor<const cl::Image2D&,  // The input image
                                            const cl::Sampler&,  // The input image sampler
                                            uint,                // The format of the image
                                            cl::Image2D&>        // The output equirectanguler image
                    (program, "debayer");

                reprojection = cl::KernelFunctor<
                    const cl::Image2D&,          // The input image
                    const cl::Sampler&,          // The input image sampler
                    float,                       // The number radians spanned by a single pixel in the
                                                 // spherical image
                    const cl::array<float, 2>&,  // The coordinates of the center of the spherical image
                    float,                       // The focal length of the camera in pixels, for the
                                                 // equirectangular image
                    cl::Image2D&>                // The output equirectanguler image
                    (program, "projectSphericalToRectilinear");

                command_queue = cl::CommandQueue(context, CL_QUEUE_PROFILING_ENABLE);
            });

            on<Trigger<Image>, With<CameraParameters>, Single>().then(
                "Image Reprojection", [this](const Image& image, const CameraParameters& cam) {
                    if (image.camera_id < 2) {
                        // For benchmarking.
                        auto start = NUClear::clock::now();

                        // Figure out cameras focal length in pixels.
                        arma::vec2 input_center  = {(image.dimensions.x() - 1) * 0.5, (image.dimensions.y() - 1) * 0.5};
                        arma::vec2 output_center = arma::conv_to<arma::vec>::from(output_dimensions - 1) * 0.5;
                        double camFocalLengthPixels = arma::norm(output_center) / tan_half_FOV;

                        // Create input image and prepare it for loading on to the device.
                        cl::Image2D input_image(context,
                                                CL_MEM_READ_ONLY,
                                                cl::ImageFormat(CL_R, CL_UNORM_INT8),
                                                image.dimensions[0],
                                                image.dimensions[1],
                                                0,
                                                nullptr,
                                                nullptr);

                        cl::array<size_t, 3> origin = {0};
                        cl::array<size_t, 3> region = {image.dimensions.x(), image.dimensions.y(), 1};
                        cl::Event img_event;
                        command_queue.enqueueWriteImage(input_image,
                                                        false,
                                                        origin,
                                                        region,
                                                        0,
                                                        0,
                                                        const_cast<uint8_t*>(image.data.data()),  // **shudders**
                                                        nullptr,
                                                        &img_event);

                        // Create output image.
                        cl::Image2D debayered_image(context,
                                                    CL_MEM_READ_WRITE,
                                                    cl::ImageFormat(CL_RGBA, CL_UNORM_INT8),
                                                    image.dimensions[0],
                                                    image.dimensions[1],
                                                    0,
                                                    nullptr,
                                                    nullptr);

                        cl::Image2D reprojected_image(context,
                                                      CL_MEM_WRITE_ONLY,
                                                      cl::ImageFormat(CL_RGBA, CL_UNORM_INT8),
                                                      output_dimensions[0],
                                                      output_dimensions[1],
                                                      0,
                                                      nullptr,
                                                      nullptr);

                        // Create a sampler for the input image.
                        cl::Sampler sampler(context, false, CL_ADDRESS_CLAMP_TO_EDGE, CL_FILTER_NEAREST);

                        // Run the kernel.
                        cl::Event debayered =
                            debayer(cl::EnqueueArgs(command_queue,
                                                    std::vector<cl::Event>({img_event}),
                                                    cl::NDRange(image.dimensions[0], image.dimensions[1])),
                                    input_image,
                                    sampler,
                                    image.format,
                                    debayered_image);

                        cl::Event reprojected;
                        if (reproject) {
                            reprojected =
                                reprojection(cl::EnqueueArgs(command_queue,
                                                             std::vector<cl::Event>({debayered}),
                                                             cl::NDRange(output_dimensions[0], output_dimensions[1])),
                                             debayered_image,
                                             sampler,
                                             cam.radial.radiansPerPixel,
                                             std::array<float, 2>{float(input_center[0]), float(input_center[1])},
                                             camFocalLengthPixels,
                                             reprojected_image);
                        }

                        // Create our output message.
                        ReprojectedImage msg[2];
                        msg[0].format        = utility::vision::FOURCC::RGB3;
                        msg[0].dimensions    = image.dimensions;
                        msg[0].camera_id     = image.camera_id + 2;
                        msg[0].serial_number = image.serial_number;
                        msg[0].timestamp     = image.timestamp;
                        msg[0].Hcw           = image.Hcw;
                        msg[0].FOV           = FOV;

                        msg[1].format        = utility::vision::FOURCC::RGB3;
                        msg[1].dimensions    = convert<unsigned int, 2>(output_dimensions);
                        msg[1].camera_id     = image.camera_id + 2;
                        msg[1].serial_number = image.serial_number;
                        msg[1].timestamp     = image.timestamp;
                        msg[1].Hcw           = image.Hcw;
                        msg[1].FOV           = FOV;

                        // Pixel data is read out as RGBA pixels. We need to convert these to RGB pixels.
                        std::vector<uint8_t> data;

                        // Get the result back to the host
                        std::vector<cl::Event> events;
                        cl::Event grab_img;

                        region[0] = image.dimensions[0];
                        region[1] = image.dimensions[1];
                        events.push_back(debayered);
                        data.resize(image.dimensions[0] * image.dimensions[1] * 4, 0);
                        command_queue.enqueueReadImage(
                            debayered_image, false, origin, region, 0, 0, data.data(), &events, &grab_img);

                        msg[0].data = std::vector<uint8_t>(image.dimensions.x() * image.dimensions.y() * 3, 0);

                        grab_img.wait();

                        for (size_t i = 0, j = 0; i < data.size(); i += 4, j += 3) {
                            msg[0].data[j + 0] = data[i + 0];
                            msg[0].data[j + 1] = data[i + 1];
                            msg[0].data[j + 2] = data[i + 2];
                        }

                        if (reproject) {
                            region[0] = output_dimensions[0];
                            region[1] = output_dimensions[1];
                            events.clear();
                            events.push_back(reprojected);
                            data.resize(output_dimensions[0] * output_dimensions[1] * 4, 0);
                            command_queue.enqueueReadImage(
                                reprojected_image, false, origin, region, 0, 0, data.data(), &events, &grab_img);

                            msg[1].data = std::vector<uint8_t>(output_dimensions[0] * output_dimensions[1] * 3, 0);
                            grab_img.wait();

                            for (size_t i = 0, j = 0; i < data.size(); i += 4, j += 3) {
                                msg[1].data[j + 0] = data[i + 0];
                                msg[1].data[j + 1] = data[i + 1];
                                msg[1].data[j + 2] = data[i + 2];
                            }
                        }

                        // Dump image to file.
                        if (dump_images) {
                            utility::vision::saveImage(fmt::format("debayered_image-{}.ppm", avg_count), msg[0]);

                            if (reproject) {
                                utility::vision::saveImage(fmt::format("reprojected_image-{}.ppm", avg_count), msg[1]);
                            }
                        }

                        // Emit our reprojected image.
                        if (reproject) {
                            emit(std::make_unique<ReprojectedImage>(msg[1]));
                        }

                        else {
                            emit(std::make_unique<ReprojectedImage>(msg[0]));
                        }

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

    }  // namespace OpenCL
}  // namespace vision
}  // namespace module
