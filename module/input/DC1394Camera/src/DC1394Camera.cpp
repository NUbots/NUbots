/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DC1394Camera.h"

#include "message/input/Image.h"
#include "extension/Configuration.h"
#include "utility/error/dc1394_error_category.h"

namespace module {
namespace input {

    using extension::Configuration;
    using message::input::Image;

    DC1394Camera::DC1394Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
        context(dc1394_new(), dc1394_free), cameras() {

        on<Configuration>("Cameras/").then([this] (const Configuration& config)
        {
            // Our error variable
            dc1394error_t err;

            // Get the device
            uint64_t deviceId = config["device"].as<uint64_t>();

            // See if we already have this camera
            auto camera = cameras.find(deviceId);

            // If we don't have a camera then make a new one
            if (camera == cameras.end()) 
            {
                // Make a new camera
                auto newCam = std::unique_ptr<dc1394camera_t, std::function<void (dc1394camera_t*)>>(dc1394_camera_new(context.get(), deviceId), dc1394_camera_free);

                if(newCam) 
                {
                    // Reset the camera to ensure it is in a known state.
                    err = dc1394_camera_reset(newCam.get());

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to issue IIDC reset to camera: ", dc1394_error_get_string(err));
                        return;
                    }

                    auto err1 = dc1394_camera_set_power(newCam.get(), DC1394_OFF);
                    auto err2 = dc1394_camera_set_power(newCam.get(), DC1394_ON);

                    if ((err1 != DC1394_SUCCESS) || (err2 != DC1394_SUCCESS))
                    {
                        NUClear::log("Failed to power cycle the camera: ", dc1394_error_get_string(err));
                        return;
                    }

                    // Make sure we are not operating in legacy mode.
                    // This should should be necessary to use ISO speeds greater than S400.
                    err = dc1394_video_set_operation_mode(newCam.get(), DC1394_OPERATION_MODE_1394B);

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set 1394B operation mode: ", dc1394_error_get_string(err));
                        return;
                    }

                    // Speed    Cycle period    Max. packet size    Max. bandwidth per ISO channel
                    // [Mb/s]  [µs]             [B]                         [MB/s]  [MiB/s]
                    // S100    125              1024                        8.192   7.8125
                    // S200    125              2048                        16.384  15.6250
                    // S400    125              4096                        32.768  31.2500
                    // S800    125              8192                        65.536  62.5000
                    // S1600   125              16384                       131.072 125.0000
                    // S3200   125              32768                       262.144 250.000
                    // Packet payload size is calculated as follows (using a 1280x1024 colour image as example).
                    // Packet payload size = frame size [bytes] times frame rate [Hz] divided by 8000 Hz
                    //                     = 1280x1024x3 * 60 Hz / 8000 Hz
                    //                     = 29491.2 B
                    //                     = 28.8 KB
                    uint width      = config["format"]["width"].as<uint>();
                    uint height     = config["format"]["height"].as<uint>();
                    uint fps        = config["format"]["fps"].as<uint>();
                    uint min, max;
                    dc1394_format7_get_packet_parameters(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, &min, &max);
                    uint packetSize = std::max(min, std::min((width * height * 3 * fps) / 8000, max));

                    fps = (8000 * packetSize) / (width * height * 3);

                    log("packetSize ", packetSize);
                    log("actual fps ", fps);

                    if (packetSize <= 1024)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_100);
                        log("ISO Speed: S100");
                    }

                    else if (packetSize <= 2048)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_200);
                        log("ISO Speed: S200");
                    }

                    else if (packetSize <= 4096)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_400);
                        log("ISO Speed: S400");
                    }

                    else if (packetSize <= 8192)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_800);
                        log("ISO Speed: S800");
                    }

                    else if (packetSize <= 16384)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_1600);
                        log("ISO Speed: S1600");
                    }

                    else if (packetSize <= 32768)
                    {
                        err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_3200);
                        log("ISO Speed: S3200");
                    }

                    else
                    {
                        NUClear::log("Frame settings result in an invalid payload packet size.");
                        return;
                    }

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set ISO speed with: ", dc1394_error_get_string(err));
                        return;
                    }

                    // Set video mode to Format-7 Mode 0.
                    // Mode 0 allows only for specifying a region of interest, and does not perform 
                    // any binning. Frame rate increases when ROI size is reduced.
                    err = dc1394_video_set_mode(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0);

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set format7 mode 0: ", dc1394_error_get_string(err));
                        return;
                    }

                    log("Format7 Mode 0 set.");
                    std::string colourCoding = config["format"]["output_format"].as<std::string>();

                    if (colourCoding.compare("RGB8") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_RGB8);
                        log("Colour Coding: RGB8");
                    }

                    else if (colourCoding.compare("RGB16") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_RGB16);
                        log("Colour Coding: RGB16");
                    }

                    else if (colourCoding.compare("YUV411") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_YUV411);
                        log("Colour Coding: YUV411");
                    }

                    else if (colourCoding.compare("YUV422") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_YUV422);
                        log("Colour Coding: YUV422");
                    }

                    else if (colourCoding.compare("YUV444") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_YUV444);
                        log("Colour Coding: YUV444");
                    }

                    else if (colourCoding.compare("RAW8") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_RAW8);
                        log("Colour Coding: RAW8");
                    }

                    else if (colourCoding.compare("RAW16") == 0)
                    {
                        err = dc1394_format7_set_color_coding(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_RAW16);
                        log("Colour Coding: RAW16");
                    }

                    else
                    {
                        NUClear::log("Invalid colour coding specified.");
                        return;
                    }

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set format7 mode 0 colour coding: ", dc1394_error_get_string(err));
                        return;
                    }

                    err = dc1394_format7_set_image_position(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, 0, 0);

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set format7 mode 0 image position: ", dc1394_error_get_string(err));
                        return;
                    }

                    err = dc1394_format7_set_image_size(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, width, height);

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set format7 mode 0 image size: ", dc1394_error_get_string(err));
                        return;
                    }

                    err = dc1394_format7_set_packet_size(newCam.get(), DC1394_VIDEO_MODE_FORMAT7_0, packetSize);

                    if (err != DC1394_SUCCESS)
                    {
                        NUClear::log("Failed to set format7 mode 0 packet size: ", dc1394_error_get_string(err));
                        return;
                    }

                    // Add our camera to the list
                    camera = cameras.insert(std::make_pair(deviceId, std::move(newCam))).first;
                }

                else
                {
                    // TODO output an error
                    NUClear::log("Failed to allocate a new camera for device with ID: ", deviceId);
                    return;
                }
            }

            bool autoExposure = config["settings"]["exposure"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_EXPOSURE, 
                                autoExposure ? DC1394_FEATURE_MODE_AUTO 
                                              : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto exposure: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoExposure) 
            {
                float exposure = config["settings"]["exposure"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_EXPOSURE, exposure);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set exposure value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoGain = config["settings"]["gain"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_GAIN, 
                                autoGain ? DC1394_FEATURE_MODE_AUTO 
                                         : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto gain: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoGain) 
            {
                float gain = config["settings"]["gain"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_GAIN, gain);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set gain value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoBrightness = config["settings"]["brightness"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_BRIGHTNESS, 
                                autoBrightness ? DC1394_FEATURE_MODE_AUTO 
                                               : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto brightness: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoBrightness) 
            {
                float brightness = config["settings"]["brightness"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_BRIGHTNESS, brightness);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set brightness value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoSharpness = config["settings"]["sharpness"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_SHARPNESS, 
                                autoSharpness ? DC1394_FEATURE_MODE_AUTO 
                                              : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto sharpness: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoSharpness) 
            {
                float sharpness = config["settings"]["sharpness"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_SHARPNESS, sharpness);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set sharpness value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoWhiteBalance = config["settings"]["white_balance"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_SHARPNESS, 
                                autoWhiteBalance ? DC1394_FEATURE_MODE_AUTO 
                                                 : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto white balance: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoWhiteBalance) 
            {
                uint u_b_value = config["settings"]["white_balance"]["u_b_value"].as<uint>();
                uint v_r_value = config["settings"]["white_balance"]["v_r_value"].as<uint>();
                err = dc1394_feature_whitebalance_set_value(camera->second.get(), u_b_value, v_r_value);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set white balance value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoHue = config["settings"]["hue"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_HUE, 
                                autoHue ? DC1394_FEATURE_MODE_AUTO 
                                        : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto hue: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoHue) 
            {
                float hue = config["settings"]["hue"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_HUE, hue);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set hue value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoSaturation = config["settings"]["saturation"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_SATURATION, 
                                autoSaturation ? DC1394_FEATURE_MODE_AUTO 
                                               : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto saturation: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoSaturation) 
            {
                float saturation = config["settings"]["saturation"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_SATURATION, saturation);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set saturation value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoGamma = config["settings"]["gamma"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_GAMMA, 
                                autoGamma ? DC1394_FEATURE_MODE_AUTO 
                                          : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto gamma: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoGamma) 
            {
                float gamma = config["settings"]["gamma"]["value"].as<float>();
                err = dc1394_feature_set_absolute_value(camera->second.get(), DC1394_FEATURE_GAMMA, gamma);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set gamma value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            bool autoTemp = config["settings"]["temperature"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_TEMPERATURE, 
                                autoTemp ? DC1394_FEATURE_MODE_AUTO 
                                         : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set auto temperature: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoTemp) 
            {
                uint temperature = config["settings"]["temperature"]["value"].as<uint>();
                err = dc1394_feature_temperature_set_value(camera->second.get(), temperature);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set temperature value: ", dc1394_error_get_string(err));
                    return;
                }
            }
            
            bool autoWhiteShading = config["settings"]["white_shading"]["auto"].as<bool>();
            err = dc1394_feature_set_mode(camera->second.get(), DC1394_FEATURE_WHITE_SHADING,
                                autoWhiteShading ? DC1394_FEATURE_MODE_AUTO 
                                                 : DC1394_FEATURE_MODE_MANUAL);
            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set white shading delay: ", dc1394_error_get_string(err));
                return;
            }

            if (!autoWhiteShading) 
            {
                uint r_value = config["settings"]["white_shading"]["r_value"].as<uint>();
                uint g_value = config["settings"]["white_shading"]["g_value"].as<uint>();
                uint b_value = config["settings"]["white_shading"]["b_value"].as<uint>();
                err = dc1394_feature_whiteshading_set_value(camera->second.get(), r_value, g_value, b_value);
                if (err != DC1394_SUCCESS)
                {
                    log("Failed to set white shading value: ", dc1394_error_get_string(err));
                    return;
                }
            }

            err = dc1394_capture_setup(camera->second.get(), 4, DC1394_CAPTURE_FLAGS_DEFAULT);

            if (err != DC1394_SUCCESS) 
            {
                log("Failed to set setup capture with: ", dc1394_error_get_string(err));
                return;
            }

            err = dc1394_video_set_transmission(camera->second.get(), DC1394_ON);

            if (err != DC1394_SUCCESS)
            {
                log("Failed to set video transmission with: ", dc1394_error_get_string(err));
                return;
            }

            on<IO>(dc1394_capture_get_fileno(camera->second.get()), IO::READ).then("Image Capture", [this, &camera]
            {
                if (camera->second) 
                {
                    dc1394video_frame_t* frame;
                    dc1394error_t err = dc1394_capture_dequeue(camera->second.get(), DC1394_CAPTURE_POLICY_WAIT, &frame);

                    if (err != DC1394_SUCCESS)
                    {
                        log("Failed to capture image with: ", dc1394_error_get_string(err));
                        return;
                    }

                    auto timestamp = NUClear::clock::time_point(std::chrono::microseconds(frame->timestamp));
                    std::vector<uint8_t> data(frame->image, frame->image + frame->image_bytes);

                    emit(std::make_unique<Image>(frame->size[0], frame->size[1], timestamp, std::move(data)));

                    dc1394_capture_enqueue(camera->second.get(), frame);
                }
            });
        });
    }
}
}

