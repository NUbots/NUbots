#ifndef MODULE_PLATFORM_MUJOCO_HPP
#define MODULE_PLATFORM_MUJOCO_HPP

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mujoco/mujoco.h>
#include <nuclear>

#include "array_safety.h"

#include "message/input/Image.hpp"

#include "utility/vision/fourcc.hpp"

namespace mju = ::mujoco::sample_util;

namespace module::platform {

    struct Render {};

    class Mujoco : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string world_path = "";
        } cfg;

        static constexpr int UPDATE_FREQUENCY = 100;

    public:
        /// @brief Called by the powerplant to build and setup the Mujoco reactor.
        explicit Mujoco(std::unique_ptr<NUClear::Environment> environment);

        // MuJoCo model and data
        mjModel* m = 0;
        mjData* d  = 0;

        unsigned char* rgb = nullptr;
        float* depth       = nullptr;

        std::mutex render_mutex;

        // MuJoCo visualization
        mjvScene scn;
        mjvCamera cam;
        mjvOption opt;
        mjrContext con;
        mjrRect viewport;
        GLFWwindow* window;

        int framecount = 0;
        int W          = 800;
        int H          = 800;

        // load model, init simulation and rendering
        void initMuJoCo(const char* filename) {
            // load and compile
            char error[1000] = "Could not load binary model";
            if (std::strlen(filename) > 4 && !std::strcmp(filename + std::strlen(filename) - 4, ".mjb")) {
                m = mj_loadModel(filename, 0);
            }
            else {
                m = mj_loadXML(filename, 0, error, 1000);
            }
            if (!m) {
                mju_error("Load model error: %s", error);
            }

            // make data, run one computation to initialize all fields
            d = mj_makeData(m);
            mj_forward(m, d);

            // initialize visualization data structures
            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjv_defaultScene(&scn);
            mjr_defaultContext(&con);

            // create scene and context
            mjv_makeScene(m, &scn, 2000);
            mjr_makeContext(m, &con, 200);

            // default free camera
            mjv_defaultFreeCamera(m, &cam);
        }

        // deallocate everything
        void closeMuJoCo(void) {
            mj_deleteData(d);
            mj_deleteModel(m);
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
        }


        // create OpenGL context/window
        void initOpenGL(void) {
            // init GLFW
            if (!glfwInit()) {
                mju_error("Could not initialize GLFW");
            }

            // create invisible window, single-buffered
            glfwWindowHint(GLFW_VISIBLE, 0);
            glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
            window = glfwCreateWindow(W, H, "Invisible window", NULL, NULL);
            if (!window) {
                mju_error("Could not create GLFW window");
            }

            // make context current
            glfwMakeContextCurrent(window);
        }


        // close OpenGL context/window
        void closeOpenGL(void) {
            // terminate GLFW
            glfwTerminate();
        }
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MUJOCO_HPP
