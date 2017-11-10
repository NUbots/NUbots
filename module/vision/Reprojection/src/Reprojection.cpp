#include <fmt/format.h>
#include <cmath>

#include "Reprojection.h"

#undef Success
#undef None
#undef Status
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

    using extension::Configuration;

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::ReprojectedImage;

    using utility::math::vision::pinhole::getCamFromScreen;
    using utility::math::vision::radial::projectCamSpaceToScreen;

    Reprojection::Reprojection(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , avg_fp_ms()
        , avg_count(0)
        , fd(-1)
        , gbm(NULL)
        , egl_display(NULL)
        , core_context(EGL_NO_CONTEXT)
        , shader_program(0)
        , vao(0)
        , vbo(0)
        , ebo(0)
        , fbo(0)
        , fbo_colour_tex(0)
        , fbo_depth_tex(0)
        , img_tex(0) {

        on<Configuration>("Reprojection.yaml").then([this](const Configuration& config) {
            arma::vec dimensions = config["output"]["dimensions"].as<arma::vec>();
            output_dimensions    = arma::conv_to<arma::uvec>::from(dimensions);
            tan_half_FOV         = std::tan(config["output"]["FOV"].as<double>() * M_PI / 360.0);
        });

        on<Startup, MainThread>().then([this]() {
            // Load render-node and associate it with a GBM device
            fd = open("/dev/dri/renderD128", O_RDWR);

            if (fd <= 0) {
                log<NUClear::WARN>("Failed to open render-node '/dev/dri/renderD128'.");
                return;
            }

            gbm = gbm_create_device(fd);

            if (gbm == NULL) {
                log<NUClear::WARN>("Failed to associate render-node '/dev/dri/renderD128' with GBM device.");
                CleanUp();
                return;
            }

            // Setup EGL from the GBM device.
            egl_display = eglGetPlatformDisplay(EGL_PLATFORM_GBM_MESA, gbm, NULL);

            if (egl_display == NULL) {
                log<NUClear::WARN>("Failed to setup EGL Display from GBM device.");
                CleanUp();
                return;
            }

            bool result = eglInitialize(egl_display, NULL, NULL);

            if (!result) {
                log<NUClear::WARN>("Failed to initialise EGL Display.");
                CleanUp();
                return;
            }

            std::string egl_extension_st = std::string(eglQueryString(egl_display, EGL_EXTENSIONS));
            if ((egl_extension_st.find("EGL_KHR_create_context") == std::string::npos)
                || (egl_extension_st.find("EGL_KHR_surfaceless_context") == std::string::npos)) {
                log<NUClear::WARN>("EGL Display does not support KHR contexts.");
                CleanUp();
                return;
            }

            static constexpr EGLint config_attribs[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, EGL_NONE};
            EGLConfig cfg;
            EGLint count;

            result = eglChooseConfig(egl_display, config_attribs, &cfg, 1, &count);
            if (!result) {
                log<NUClear::WARN>("EGL Display configuration failed.");
                CleanUp();
                return;
            }

            result = eglBindAPI(EGL_OPENGL_ES_API);
            if (!result) {
                log<NUClear::WARN>("EGL failed to bind API.");
                CleanUp();
                return;
            }

            static constexpr EGLint attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
            core_context                      = eglCreateContext(egl_display, cfg, EGL_NO_CONTEXT, attribs);
            if (core_context == EGL_NO_CONTEXT) {
                log<NUClear::WARN>("EGL failed to create context.");
                CleanUp();
                return;
            }

            result = eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, core_context);
            if (!result) {
                log<NUClear::WARN>("EGL failed to make context current.");
                CleanUp();
                return;
            }

#ifndef NDEBUG
            /* print some compute limits (not strictly necessary) */
            GLint work_group_count[3] = {0};
            for (unsigned i = 0; i < 3; i++) {
                glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, i, &work_group_count[i]);
            }

            log<NUClear::DEBUG>(fmt::format("GL_MAX_COMPUTE_WORK_GROUP_COUNT: {}, {}, {}",
                                            work_group_count[0],
                                            work_group_count[1],
                                            work_group_count[2]));

            GLint work_group_size[3] = {0};
            for (unsigned i = 0; i < 3; i++) {
                glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, i, &work_group_size[i]);
            }

            log<NUClear::DEBUG>(fmt::format("GL_MAX_COMPUTE_WORK_GROUP_SIZE: {}, {}, {}",
                                            work_group_size[0],
                                            work_group_size[1],
                                            work_group_size[2]));

            GLint max_invocations;
            glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &max_invocations);
            log<NUClear::DEBUG>(fmt::format("GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS: {}", max_invocations));

            GLint mem_size;
            glGetIntegerv(GL_MAX_COMPUTE_SHARED_MEMORY_SIZE, &mem_size);
            log<NUClear::DEBUG>(fmt::format("GL_MAX_COMPUTE_SHARED_MEMORY_SIZE: {}", mem_size));
#endif

            // Create the shader program.
            shader_program =
                loadShaderProgram({{"reprojection.vs", GL_VERTEX_SHADER}, {"reprojection.fs", GL_FRAGMENT_SHADER}});

            if (!glIsProgram(shader_program)) {
                log<NUClear::WARN>("Failed to link shader program.");
                CleanUp();
                return;
            }

            glUseProgram(shader_program);

            if (checkOpenGLError() != GL_NO_ERROR) {
                log<NUClear::WARN>("Failed to use shader program.");
                CleanUp();
                return;
            }

            glEnable(GL_DEPTH_TEST);

            // Create a framebuffer to render to.
            glGenFramebuffers(1, &fbo);

            // Bind the frame buffer.
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);

            // Create the colour texture.
            glGenTextures(1, &fbo_colour_tex);
            glBindTexture(GL_TEXTURE_2D, fbo_colour_tex);
            if (checkOpenGLError() != GL_NO_ERROR) {
                log<NUClear::WARN>("Failed to create colour texture.");
                CleanUp();
                return;
            }
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGB, output_dimensions[0], output_dimensions[1], 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
            if (checkOpenGLError() != GL_NO_ERROR) {
                log<NUClear::WARN>("Failed to create colour texture.");
                CleanUp();
                return;
            }
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            if (checkOpenGLError() != GL_NO_ERROR) {
                log<NUClear::WARN>("Failed to create colour texture.");
                CleanUp();
                return;
            }
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_colour_tex, 0);

            if (checkOpenGLError() != GL_NO_ERROR) {
                log<NUClear::WARN>("Failed to create colour texture.");
                CleanUp();
                return;
            }

            // Create the depth texture.
            // glGenTextures(1, &fbo_depth_tex);
            // glBindTexture(GL_TEXTURE_2D, fbo_depth_tex);
            // if (checkOpenGLError() != GL_NO_ERROR) {
            //     log<NUClear::WARN>("1) Failed to create depth texture.");
            //     CleanUp();
            //     return;
            // }
            // glTexImage2D(GL_TEXTURE_2D,
            //              0,
            //              GL_RGB,
            //              output_dimensions[0],
            //              output_dimensions[1],
            //              0,
            //              GL_DEPTH_COMPONENT,
            //              GL_UNSIGNED_BYTE,
            //              0);
            // if (checkOpenGLError() != GL_NO_ERROR) {
            //     log<NUClear::WARN>("2) Failed to create depth texture.");
            //     CleanUp();
            //     return;
            // }
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            // if (checkOpenGLError() != GL_NO_ERROR) {
            //     log<NUClear::WARN>("3) Failed to create depth texture.");
            //     CleanUp();
            //     return;
            // }
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // if (checkOpenGLError() != GL_NO_ERROR) {
            //     log<NUClear::WARN>("4) Failed to create depth texture.");
            //     CleanUp();
            //     return;
            // }
            // glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, fbo_colour_tex, 0);

            // if (checkOpenGLError() != GL_NO_ERROR) {
            //     log<NUClear::WARN>("5) Failed to create depth texture.");
            //     CleanUp();
            //     return;
            // }

            // Always check that our framebuffer is ok
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                log<NUClear::WARN>("Failed to generate framebuffer.");
                CleanUp();
                return;
            }

            pbo_size = 3 * output_dimensions[0] * output_dimensions[1];
            glGenBuffers(1, &pbo_read);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_read);
            glBufferData(GL_PIXEL_PACK_BUFFER, pbo_size, 0, GL_DYNAMIC_READ);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

            // Create Vertex Array Object (VAO).
            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            // Create a Vertex Buffer Object (VBO).
            glGenBuffers(1, &vbo);

            // Specify the vertices.
            // clang-format off
            float vertices[] = {
                0,                           float(output_dimensions[1]), 0, 0, 0, 1, 0, 1,
                float(output_dimensions[0]), float(output_dimensions[1]), 0, 0, 0, 1, 1, 1,
                0,                           0,                           0, 0, 0, 1, 0, 0,
                float(output_dimensions[0]), 0,                           0, 0, 0, 1, 1, 0
            };
            GLuint indices[] = {
                0, 2, 1,
                2, 3, 1
            };
            // clang-format on

            // Copy vertex data into the VBO.
            // GL_STATIC_DRAW: Copy vertex data to graphics card once, then redraw many times.
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            // Create an Element Buffer Object (EBO).
            glGenBuffers(1, &ebo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLuint), indices, GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            GLint posAttrib = glGetAttribLocation(shader_program, "position");
            glEnableVertexAttribArray(posAttrib);
            glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), 0);

            GLint uvAttrib = glGetAttribLocation(shader_program, "uv");
            glEnableVertexAttribArray(uvAttrib);
            glVertexAttribPointer(uvAttrib, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*) (6 * sizeof(GLfloat)));

            // Create model, view and projection matrices.
            GLint uniProj  = glGetUniformLocation(shader_program, "proj");
            GLint uniView  = glGetUniformLocation(shader_program, "view");
            GLint uniModel = glGetUniformLocation(shader_program, "model");

            glm::mat4 view =
                glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            glm::mat4 proj  = glm::ortho(0.0f, 800.0f, 600.0f, 0.0f, 0.1f, 10.0f);
            glm::mat4 model = glm::mat4(1.0);

            glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
            glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));

            uniImageFormat          = glGetUniformLocation(shader_program, "imageFormat");
            uniImageWidth           = glGetUniformLocation(shader_program, "imageWidth");
            uniImageHeight          = glGetUniformLocation(shader_program, "imageHeight");
            uniResolution           = glGetUniformLocation(shader_program, "resolution");
            uniFirstRed             = glGetUniformLocation(shader_program, "firstRed");
            uniRadiansPerPixel      = glGetUniformLocation(shader_program, "radiansPerPixel");
            uniCamFocalLengthPixels = glGetUniformLocation(shader_program, "camFocalLengthPixels");

            // Generate the image texture.
            glGenTextures(1, &img_tex);
        });

        on<Shutdown, MainThread>().then([this] { CleanUp(); });

        on<Trigger<Image>, With<CameraParameters>, Single, MainThread>().then(
            "Image Reprojection", [this](const Image& image, const CameraParameters& cam) {

                if (glIsBuffer(vbo) && glIsBuffer(ebo) && glIsFramebuffer(fbo) && glIsTexture(fbo_colour_tex)
                    && glIsVertexArray(vao)) {

                    auto start = NUClear::clock::now();

                    arma::vec2 output_center    = arma::conv_to<arma::vec>::from(output_dimensions - 1) * 0.5;
                    double camFocalLengthPixels = arma::norm(output_center) / tan_half_FOV;

                    GLfloat dims[2] = {GLfloat(image.dimensions[0]), GLfloat(image.dimensions[1])};
                    GLfloat red[2]  = {0.0f, 0.0f};

                    switch (image.format) {
                        case utility::vision::FOURCC::GRBG:
                            red[0] = 1.0f;
                            red[1] = 0.0f;
                            break;
                        case utility::vision::FOURCC::GBRG:
                            red[0] = 0.0f;
                            red[1] = 1.0f;
                            break;
                        case utility::vision::FOURCC::BGGR:
                            red[0] = 1.0f;
                            red[1] = 1.0f;
                            break;
                        case utility::vision::FOURCC::RGGB:
                            red[0] = 0.0f;
                            red[1] = 0.0f;
                            break;
                        default: break;
                    }

                    glUniform1i(uniImageFormat, image.format);
                    glUniform1i(uniImageWidth, output_dimensions[0]);
                    glUniform1i(uniImageHeight, output_dimensions[1]);
                    glUniform2fv(uniResolution, 1, dims);
                    glUniform2fv(uniFirstRed, 1, red);
                    glUniform1f(uniRadiansPerPixel, cam.radial.radiansPerPixel);
                    glUniform1f(uniCamFocalLengthPixels, camFocalLengthPixels);

                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("Failed to set uniforms.");
                        return;
                    }

                    // Load the texture.
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, img_tex);
                    glTexImage2D(GL_TEXTURE_2D,
                                 0,
                                 GL_RED,
                                 image.dimensions[0],
                                 image.dimensions[1],
                                 0,
                                 GL_RED,
                                 GL_UNSIGNED_BYTE,
                                 image.data.data());
                    glUniform1i(glGetUniformLocation(shader_program, "rawImage"), 0);

                    // Set texture parameters.
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("Failed to load image texture.");
                        return;
                    }

                    // Clear the screen.
                    // glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                    GLenum temp = GL_COLOR_ATTACHMENT0;
                    glDrawBuffers(1, &temp);
                    glViewport(0, 0, output_dimensions[0], output_dimensions[1]);
                    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
                    glBindVertexArray(vao);

                    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("Failed to draw scene.");
                        return;
                    }

                    // Block until rendering is complete.
                    glFinish();

                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("Failed to render scene to framebuffer.");
                        return;
                    }

                    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                        log<NUClear::WARN>("2) Framebuffer is not complete.");
                        return;
                    }

                    ReprojectedImage msg;
                    msg.format        = utility::vision::FOURCC::RGB3;
                    msg.dimensions    = convert<unsigned int, 2>(output_dimensions);
                    msg.camera_id     = image.camera_id;
                    msg.serial_number = image.serial_number;
                    msg.timestamp     = image.timestamp;
                    msg.Hcw           = image.Hcw;
                    msg.data          = std::vector<uint8_t>(output_dimensions[0] * output_dimensions[1] * 3, 0);

                    // Read texture data out of the framebuffer object.
                    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                    glPixelStorei(GL_PACK_ALIGNMENT, 1);
                    glReadBuffer(GL_COLOR_ATTACHMENT0);
                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("0) Failed to extract reprojected image.");
                    }

                    auto data = std::vector<uint8_t>(output_dimensions[0] * output_dimensions[1] * 4, 0);
                    glReadPixels(
                        0, 0, output_dimensions[0], output_dimensions[1], GL_RGBA, GL_UNSIGNED_BYTE, data.data());
                    for (size_t i = 0, j = 0; i < data.size(); i += 4, j += 3) {
                        msg.data[j + 0] = data[i + 0];
                        msg.data[j + 1] = data[i + 1];
                        msg.data[j + 2] = data[i + 2];
                    }

                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("1) Failed to extract reprojected image.");
                    }

                    // glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_read);
                    // void* ptr = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, pbo_size, GL_MAP_READ_BIT);
                    // std::memcpy(msg.data.data(), ptr, pbo_size);
                    // glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

                    // if (checkOpenGLError() != GL_NO_ERROR) {
                    //     log<NUClear::WARN>("2) Failed to extract reprojected image.");
                    // }

                    // glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

                    utility::vision::saveImage("reprojected_image.ppm", msg);


                    if (checkOpenGLError() != GL_NO_ERROR) {
                        log<NUClear::WARN>("Failed to extract reprojected image.");
                    }

                    else {
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
                }
            });
    }

    GLenum Reprojection::checkOpenGLError() {
        GLenum err = glGetError();

#ifndef NDEBUG
        switch (err) {
            case GL_NO_ERROR: break;
            case GL_INVALID_ENUM:
                log<NUClear::DEBUG>("An unacceptable value is specified for an enumerated argument.");
                break;
            case GL_INVALID_VALUE: log<NUClear::DEBUG>("A numeric argument is out of range."); break;
            case GL_INVALID_OPERATION:
                log<NUClear::DEBUG>("The specified operation is not allowed in the current state.");
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:
                log<NUClear::DEBUG>(
                    "The command is trying to render to or read from the framebuffer while the currently bound "
                    "framebuffer is not framebuffer complete.");
                break;
            case GL_OUT_OF_MEMORY:
                log<NUClear::DEBUG>("There is not enough memory left to execute the command.");
                break;
            default: log<NUClear::DEBUG>("An unknown OpenGL error occurred."); break;
        }
#endif

        return err;
    }

    GLuint Reprojection::loadShaderProgram(const std::vector<std::pair<std::string, GLuint>>& shaders) {
        GLuint shaderProgram = glCreateProgram();
        std::vector<GLuint> handles;
        for (const auto& shader : shaders) {
            GLuint handle;
            if (!loadShader(&handle, shader.second, shader.first)) {
                log<NUClear::WARN>("Failed to compile shader.");
                return 0;
            }

            handles.push_back(handle);
            glAttachShader(shaderProgram, handle);
        }

        // Bind fragment shader output to the correct output buffer.
        // glBindFragDataLocation(shaderProgram, 0, "outColour");

        // Link the shader program.
        glLinkProgram(shaderProgram);

        if (!getProgramLinkStatus(shaderProgram)) {
            log<NUClear::WARN>("Failed to link shader program.");
            for (const auto& shader : shaders) {
                std::cerr << shader.first << std::endl;
            }
            std::cerr << std::endl;
            return 0;
        }

        for (const auto& handle : handles) {
            glDetachShader(shaderProgram, handle);
            glDeleteShader(handle);
        }

        return shaderProgram;
    }

    bool Reprojection::loadShader(GLuint* shader, GLenum type, const std::string& file) {
        // Create the vertex shader.
        std::ifstream sourceStream(file);
        std::string source;

        sourceStream.seekg(0, std::ios::end);
        source.reserve(sourceStream.tellg());
        sourceStream.seekg(0, std::ios::beg);

        source.assign((std::istreambuf_iterator<char>(sourceStream)), std::istreambuf_iterator<char>());

        sourceStream.close();

        // Compile shader.
        const char* shaderSource = source.c_str();
        *shader                  = glCreateShader(type);
        glShaderSource(*shader, 1, &shaderSource, nullptr);
        glCompileShader(*shader);

        return (getShaderCompileStatus(*shader));
    }

    bool Reprojection::getShaderCompileStatus(GLuint shader) {
        // Get status
        GLint status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

        if (status == GL_TRUE) {
            return true;
        }

        else {
            // Get the length of the compile log.
            GLint logLength;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

            // Get log.
            std::vector<char> buffer(logLength);
            glGetShaderInfoLog(shader, logLength, nullptr, buffer.data());
            log<NUClear::WARN>("Shader failed to compile.");
            log<NUClear::WARN>("Compile Log:", std::string(buffer.begin(), buffer.end()));

            return false;
        }
    }

    bool Reprojection::getProgramLinkStatus(GLuint program) {
        // Get status
        GLint status;
        glGetProgramiv(program, GL_LINK_STATUS, &status);

        if (status == GL_TRUE) {
            return true;
        }

        else {
            // Get the length of the compile log.
            GLint logLength;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);

            // Get log.
            std::vector<char> buffer(logLength);
            glGetProgramInfoLog(program, logLength, &logLength, buffer.data());
            log<NUClear::WARN>("Program failed to link.");
            log<NUClear::WARN>("Compile Log:", std::string(buffer.begin(), buffer.end()));

            return false;
        }
    }

    void Reprojection::CleanUp() {
        if (glIsTexture(fbo_colour_tex)) {
            glDeleteTextures(1, &fbo_colour_tex);
        }

        if (glIsTexture(fbo_depth_tex)) {
            glDeleteTextures(1, &fbo_depth_tex);
        }

        if (glIsTexture(img_tex)) {
            glDeleteTextures(1, &img_tex);
        }

        if (glIsBuffer(ebo)) {
            glDeleteBuffers(1, &ebo);
        }

        if (glIsBuffer(vbo)) {
            glDeleteBuffers(1, &vbo);
        }

        if (glIsFramebuffer(fbo)) {
            glDeleteFramebuffers(1, &fbo);
        }

        if (glIsRenderbuffer(fbo)) {
            glDeleteRenderbuffers(1, &fbo);
        }

        if (glIsVertexArray(vao)) {
            glDeleteVertexArrays(1, &vao);
        }

        if (glIsProgram(shader_program)) {
            glDeleteProgram(shader_program);
        }

        if (core_context != EGL_NO_CONTEXT) {
            eglDestroyContext(egl_display, core_context);
        }

        if (egl_display != NULL) {
            eglTerminate(egl_display);
        }

        if (gbm != NULL) {
            gbm_device_destroy(gbm);
        }

        if (fd > 0) {
            close(fd);
        }
    }
}  // namespace vision
}  // namespace module
