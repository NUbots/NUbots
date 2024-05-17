#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"

#include "utility/vision/fourcc.hpp"

namespace module::platform {

    using extension::Configuration;

    using message::input::Image;

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();
        });

        on<Always, Once>().then("Start Mujoco", [this] {
            initOpenGL();
            initMuJoCo(cfg.world_path.c_str());

            // set rendering to offscreen buffer
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            if (con.currentBuffer != mjFB_OFFSCREEN) {
                log<NUClear::WARN>("Offscreen rendering not supported, using default/window framebuffer");
            }

            // get size of active renderbuffer
            mjrRect viewport = mjr_maxViewport(&con);
            int W            = viewport.width;
            int H            = viewport.height;

            // allocate rgb and depth buffers
            unsigned char* rgb = (unsigned char*) std::malloc(3 * W * H);
            float* depth       = (float*) std::malloc(sizeof(float) * W * H);
            if (!rgb || !depth) {
                log<NUClear::ERROR>("Could not allocate buffers");
            }

            int adddepth = 1;

            // main loop
            double frametime = 0;
            int framecount   = 0;
            double duration  = 5;
            double fps       = 30;
            while (d->time < duration) {
                // render new frame if it is time (or first frame)
                if ((d->time - frametime) > 1 / fps || frametime == 0) {
                    // update abstract scene
                    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

                    // render scene in offscreen buffer
                    mjr_render(viewport, &scn, &con);

                    // add time stamp in upper-left corner
                    char stamp[50];
                    mju::sprintf_arr(stamp, "Time = %.3f", d->time);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

                    // read rgb and depth buffers
                    mjr_readPixels(rgb, depth, viewport, &con);

                    auto image            = std::make_unique<message::input::Image>();
                    image->timestamp      = NUClear::clock::now();
                    image->dimensions.x() = W;
                    image->dimensions.y() = H;

                    // Flip the image
                    for (int row = 0; row < H / 2; ++row) {
                        for (int col = 0; col < W; ++col) {
                            for (int channel = 0; channel < 3; ++channel) {
                                std::swap(rgb[(row * W + col) * 3 + channel],
                                          rgb[((H - 1 - row) * W + col) * 3 + channel]);
                            }
                        }
                    }

                    std::vector<uint8_t> image_data(rgb, rgb + 3 * W * H);
                    image->data.assign(image_data.begin(), image_data.end());
                    image->format = utility::vision::fourcc("RGB3");
                    image->Hcw    = Eigen::Isometry3d::Identity();
                    image->id     = 1;
                    image->name   = "Mujoco";
                    emit(std::move(image));

                    // insert subsampled depth image in lower-left corner of rgb image
                    if (true) {
                        const int NS = 3;  // depth image sub-sampling
                        for (int r = 0; r < H; r += NS) {
                            for (int c = 0; c < W; c += NS) {
                                int adr      = (r / NS) * W + c / NS;
                                rgb[3 * adr] = rgb[3 * adr + 1] = rgb[3 * adr + 2] =
                                    (unsigned char) ((1.0f - depth[r * W + c]) * 255.0f);
                            }
                        }
                    }

                    // print every 10 frames: '.' if ok, 'x' if OpenGL error
                    if (((framecount++) % 10) == 0) {
                        if (mjr_getError()) {
                            log<NUClear::ERROR>("OpenGL error detected");
                        }
                    }

                    // save simulation time
                    frametime = d->time;
                }

                // advance simulation
                auto now = std::chrono::high_resolution_clock::now();
                mj_step(m, d);
                auto end = std::chrono::high_resolution_clock::now();
                // Calculate the elapsed time in milliseconds
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - now).count();
            }

            // close file, free buffers
            std::free(rgb);
            std::free(depth);

            // close MuJoCo and OpenGL
            closeMuJoCo();
            closeOpenGL();
        });

        on<Always>().then("Simulate", [this] {

        });
    }

}  // namespace module::platform
