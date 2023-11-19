#include "Mujoco.hpp"

#include "extension/Configuration.hpp"

namespace module::platform {

    using extension::Configuration;

    Mujoco::Mujoco(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Mujoco.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Mujoco.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.world_path  = config["world_path"].as<std::string>();
        });

        on<Startup>().then("Start Mujoco", [this] {
            // Start Mujoco world
            log<NUClear::INFO>("Mujoco started");

            // Load the model
            char error[1000] = "Could not load binary model";
            m                = mj_loadXML(cfg.world_path.c_str(), 0, error, 1000);
            if (!m) {
                mju_error_s("Load model error: %s", error);
            }

            // Make data
            d = mj_makeData(m);

            // Initialize MuJoCo visualization
            if (!glfwInit()) {
                mju_error("Could not initialize GLFW");
            }

            // create window, make OpenGL context current, request v-sync
            window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            // initialize visualization data structures
            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjv_defaultScene(&scn);
            mjr_defaultContext(&con);

            // create scene and context
            mjv_makeScene(m, &scn, 2000);
            mjr_makeContext(m, &con, mjFONTSCALE_150);

            // install GLFW mouse and keyboard callbacks
            glfwSetWindowUserPointer(window, this);

            while (!glfwWindowShouldClose(window)) {
                mjtNum simstart = d->time;
                while (d->time - simstart < 1.0 / 60.0) {
                    mj_step(m, d);
                }

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                // update scene and render
                mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
                mjr_render(viewport, &scn, &con);

                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
            }

            // free visualization storage
            mjv_freeScene(&scn);
            mjr_freeContext(&con);

            // free MuJoCo model and data
            mj_deleteData(d);
            mj_deleteModel(m);
        });

        // on<Trigger<Sensors>>().then("Update Mujoco", [this](const Sensors& sensors) {
        //     // TODO: Update Mujoco world with sensor data, object positions, etc.
        //     log<NUClear::INFO>("Mujoco updated");
        // });

        on<Every<1, Per<std::chrono::seconds>>>().then("Simulate", [this] {
            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
        });
    }

}  // namespace module::platform
