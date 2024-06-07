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
        int W          = 1920;
        int H          = 1080;

        /// @brief The maximum velocity allowed by the NUgus motors in webots
        double max_velocity_mx64;
        double max_velocity_mx106;

        /// @brief The number of time ticks which have passed since the last IO::READ trigger
        double sim_delta = 0;
        /// @brief The number of milliseconds which have passed since the last IO::READ trigger
        double real_delta = 0;
        /// @brief The current simulation time in ticks
        double current_sim_time = 0;
        /// @brief The current real time in milliseconds (unix time)
        NUClear::clock::time_point current_real_time = NUClear::clock::now();
        /// @brief Interpolation factor to smooth clock. 0.0 is no smoothing (raw updates from Webots), 1.0 takes no
        /// updates from Webots
        double clock_smoothing = 0.6;
        /// @brief Real time factor of the simulation clock
        double rtf = 1.0;

        /// @brief Current state of a servo
        struct ServoState {
            /// @brief ID of the servo
            int id;

            /// @brief Name of the servo
            std::string name;

            double p_gain = 32.0 / 255.0;
            // `i` and `d` gains are always 0
            static constexpr double i_gain = 0.0;
            double d_gain                  = 0.0;

            double moving_speed  = 0.0;
            double goal_position = 0.0;
            double torque        = 0.0;  // 0.0 to 1.0

            /// Values that are read from the simulator
            double present_position = 0.0;
            double present_speed    = 0.0;
        };

        /// @brief Our current servo states
        std::map<std::string, ServoState> servo_state;

        void render(void) {
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
        }
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MUJOCO_HPP
