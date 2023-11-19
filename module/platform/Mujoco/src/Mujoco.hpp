#ifndef MODULE_PLATFORM_MUJOCO_HPP
#define MODULE_PLATFORM_MUJOCO_HPP

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>
#include <nuclear>

namespace module::platform {

    class Mujoco : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string world_path = "";
        } cfg;

        // MuJoCo data structures
        mjModel* m = NULL;  // MuJoCo model
        mjData* d  = NULL;  // MuJoCo data
        mjvCamera cam;      // abstract camera
        mjvOption opt;      // visualization options
        mjvScene scn;       // abstract scene
        mjrContext con;     // custom GPU context

    public:
        /// @brief Called by the powerplant to build and setup the Mujoco reactor.
        explicit Mujoco(std::unique_ptr<NUClear::Environment> environment);

        // mouse interaction
        bool button_left   = false;
        bool button_middle = false;
        bool button_right  = false;
        double lastx       = 0;
        double lasty       = 0;

        GLFWwindow* window;


        // keyboard callback
        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
            // backspace: reset simulation
            if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
                mj_resetData(m, d);
                mj_forward(m, d);
            }
        }

        // mouse button callback
        void mouse_button(GLFWwindow* window, int button, int act, int mods) {
            // update button state
            button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
            button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

            // update mouse position
            glfwGetCursorPos(window, &lastx, &lasty);
        }

        // mouse move callback
        void mouse_move(GLFWwindow* window, double xpos, double ypos) {
            // no buttons down: nothing to do
            if (!button_left && !button_middle && !button_right) {
                return;
            }

            // compute mouse displacement, save
            double dx = xpos - lastx;
            double dy = ypos - lasty;
            lastx     = xpos;
            lasty     = ypos;

            // get current window size
            int width, height;
            glfwGetWindowSize(window, &width, &height);

            // get shift key state
            bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
                              || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

            // determine action based on mouse button
            mjtMouse action;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            }
            else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            }
            else {
                action = mjMOUSE_ZOOM;
            }

            // move camera
            mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
        }

        // scroll callback
        void scroll(GLFWwindow* window, double xoffset, double yoffset) {
            // emulate vertical mouse motion = 5% of window height
            mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
        }

        // Static member functions
        static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods) {
            Mujoco* instance = static_cast<Mujoco*>(glfwGetWindowUserPointer(window));
            if (instance) {
                instance->keyboard(window, key, scancode, act, mods);
            }
        }

        static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
            Mujoco* instance = static_cast<Mujoco*>(glfwGetWindowUserPointer(window));
            if (instance) {
                instance->mouse_move(window, xpos, ypos);
            }
        }
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MUJOCO_HPP
