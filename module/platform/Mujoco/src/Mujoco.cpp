#include "Mujoco.hpp"

#include "extension/Configuration.hpp"


namespace module::platform {

    using extension::Configuration;

    using message::input::Image;

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();
        });

        on<Startup>().then("Start Mujoco", [this] {
            // init opengl
            initOpenGL();

            // init mujoco
            initMuJoCo(cfg.world_path.c_str());

            // set rendering to offscreen buffer
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            if (con.currentBuffer != mjFB_OFFSCREEN) {
                log<NUClear::WARN>("Offscreen rendering not supported, using default/window framebuffer");
            }

            // get size of active renderbuffer
            viewport = mjr_maxViewport(&con);
            W        = viewport.width;
            H        = viewport.height;

            // allocate rgb and depth buffers
            rgb   = (unsigned char*) std::malloc(3 * W * H);
            depth = (float*) std::malloc(sizeof(float) * W * H);
            if (!rgb) {
                log<NUClear::ERROR>("Could not allocate buffers");
            }

            // render
            emit(std::make_unique<Render>());
        });

        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Priority::HIGH, Single>().then(
            "Simulator Update Loop",
            [this] {
                double next_sim_step = d->time + 1.0 / UPDATE_FREQUENCY;
                while (d->time < next_sim_step) {
                    // TODO: ctrl

                    // advance simulation
                    mj_step(m, d);
                }

                // TODO: sensors

                // render
                emit(std::make_unique<Render>());
            });

        on<Trigger<Render>, Single>().then("Render Mujoco", [this] {
            auto now = NUClear::clock::now();
            // std::unique_lock<std::mutex> lock(render_mutex);
            glfwMakeContextCurrent(window);

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

            // flip the image
            for (int row = 0; row < H / 2; ++row) {
                for (int col = 0; col < W; ++col) {
                    for (int channel = 0; channel < 3; ++channel) {
                        std::swap(rgb[(row * W + col) * 3 + channel], rgb[((H - 1 - row) * W + col) * 3 + channel]);
                    }
                }
            }

            std::vector<uint8_t> image_data(rgb, rgb + 3 * W * H);
            image->data.assign(image_data.begin(), image_data.end());
            image->format = utility::vision::fourcc("RGB3");
            image->name   = "Mujoco";
            emit(std::move(image));
            auto end      = NUClear::clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - now).count();
            log<NUClear::DEBUG>("Rendering took", duration, "ms");
        });

        on<Shutdown>().then("Shutdown Mujoco", [this] {
            // free buffers
            std::free(rgb);
            std::free(depth);
            closeMuJoCo();
            closeOpenGL();
        });
    }

}  // namespace module::platform
