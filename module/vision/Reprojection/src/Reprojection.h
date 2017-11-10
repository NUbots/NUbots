#ifndef MODULE_VISION_REPROJECTION_H
#define MODULE_VISION_REPROJECTION_H

#include <armadillo>
#include <string>
#include <vector>

#include <nuclear>

extern "C" {
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <fcntl.h>
#include <gbm.h>
}

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace module {
namespace vision {

    class Reprojection : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Reprojection reactor.
        explicit Reprojection(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool dump_images;
        std::chrono::duration<double, std::milli> avg_fp_ms;
        size_t avg_count;

        int32_t fd;
        struct gbm_device* gbm;
        EGLDisplay egl_display;
        EGLContext core_context;

        GLuint shader_program;
        GLuint vao, vbo, ebo, fbo, fbo_colour_tex, fbo_depth_tex, img_tex, rbo;
        GLuint pbo_size, pbo_read;
        GLint uniImageFormat;
        GLint uniImageWidth;
        GLint uniImageHeight;
        GLint uniResolution;
        GLint uniFirstRed;
        GLint uniRadiansPerPixel;
        GLint uniCamFocalLengthPixels;

        arma::uvec2 output_dimensions;
        double tan_half_FOV;

        GLenum checkOpenGLError();
        GLuint loadShaderProgram(const std::vector<std::pair<std::string, GLuint>>& shaders);
        bool loadShader(GLuint* shader, GLenum type, const std::string& file);
        bool getShaderCompileStatus(GLuint shader);
        bool getProgramLinkStatus(GLuint program);

        void CleanUp();
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_REPROJECTION_H
