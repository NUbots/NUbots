#ifndef MODULE_PLATFORM_MUJOCO_HPP
#define MODULE_PLATFORM_MUJOCO_HPP

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mujoco/mujoco.h>
#include <nuclear>

#include "array_safety.h"
namespace mju = ::mujoco::sample_util;


namespace module::platform {

    class Mujoco : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string world_path = "";
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the Mujoco reactor.
        explicit Mujoco(std::unique_ptr<NUClear::Environment> environment);

        // MuJoCo model and data
        mjModel* m = 0;
        mjData* d  = 0;

        // MuJoCo visualization
        mjvScene scn;
        mjvCamera cam;
        mjvOption opt;
        mjrContext con;

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
            //------------------------ EGL
#if defined(MJ_EGL)
            // desired config
            const EGLint configAttribs[] = {EGL_RED_SIZE,
                                            8,
                                            EGL_GREEN_SIZE,
                                            8,
                                            EGL_BLUE_SIZE,
                                            8,
                                            EGL_ALPHA_SIZE,
                                            8,
                                            EGL_DEPTH_SIZE,
                                            24,
                                            EGL_STENCIL_SIZE,
                                            8,
                                            EGL_COLOR_BUFFER_TYPE,
                                            EGL_RGB_BUFFER,
                                            EGL_SURFACE_TYPE,
                                            EGL_PBUFFER_BIT,
                                            EGL_RENDERABLE_TYPE,
                                            EGL_OPENGL_BIT,
                                            EGL_NONE};

            // get default display
            EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if (eglDpy == EGL_NO_DISPLAY) {
                mju_error("Could not get EGL display, error 0x%x\n", eglGetError());
            }

            // initialize
            EGLint major, minor;
            if (eglInitialize(eglDpy, &major, &minor) != EGL_TRUE) {
                mju_error("Could not initialize EGL, error 0x%x\n", eglGetError());
            }

            // choose config
            EGLint numConfigs;
            EGLConfig eglCfg;
            if (eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs) != EGL_TRUE) {
                mju_error("Could not choose EGL config, error 0x%x\n", eglGetError());
            }

            // bind OpenGL API
            if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
                mju_error("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());
            }

            // create context
            EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
            if (eglCtx == EGL_NO_CONTEXT) {
                mju_error("Could not create EGL context, error 0x%x\n", eglGetError());
            }

            // make context current, no surface (let OpenGL handle FBO)
            if (eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx) != EGL_TRUE) {
                mju_error("Could not make EGL context current, error 0x%x\n", eglGetError());
            }

                //------------------------ OSMESA
#elif defined(MJ_OSMESA)
            // create context
            ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
            if (!ctx) {
                mju_error("OSMesa context creation failed");
            }

            // make current
            if (!OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, 800, 800)) {
                mju_error("OSMesa make current failed");
            }

                //------------------------ GLFW
#else
            // init GLFW
            if (!glfwInit()) {
                mju_error("Could not initialize GLFW");
            }

            // create invisible window, single-buffered
            glfwWindowHint(GLFW_VISIBLE, 0);
            glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
            GLFWwindow* window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
            if (!window) {
                mju_error("Could not create GLFW window");
            }

            // make context current
            glfwMakeContextCurrent(window);
#endif
        }


        // close OpenGL context/window
        void closeOpenGL(void) {
            //------------------------ EGL
#if defined(MJ_EGL)
            // get current display
            EGLDisplay eglDpy = eglGetCurrentDisplay();
            if (eglDpy == EGL_NO_DISPLAY) {
                return;
            }

            // get current context
            EGLContext eglCtx = eglGetCurrentContext();

            // release context
            eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

            // destroy context if valid
            if (eglCtx != EGL_NO_CONTEXT) {
                eglDestroyContext(eglDpy, eglCtx);
            }

            // terminate display
            eglTerminate(eglDpy);

                //------------------------ OSMESA
#elif defined(MJ_OSMESA)
            OSMesaDestroyContext(ctx);

                //------------------------ GLFW
#else
    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
            glfwTerminate();
    #endif
#endif
        }
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MUJOCO_HPP
