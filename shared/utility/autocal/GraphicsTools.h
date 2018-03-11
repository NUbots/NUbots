#ifndef UTILITY_AUTOCAL_GRAPHICSTOOLS_H
#define UTILITY_AUTOCAL_GRAPHICSTOOLS_H

#ifdef __APPLE__
// #include <OpenGL/gl.h>
#include <GL/glew.h>

#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include "glm/ext.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "opencv2/opencv.hpp"

#include "utility/autocal/SensorPlant.h"
#include "utility/math/matrix/Transform3D.h"


static void checkGLError() {

    GLenum error;
    while ((error = glGetError()) != GL_NO_ERROR) {
        std::cout << "GL Error = " << std::endl;
        switch (error) {
            case 0: std::cout << "Internal error in glGetError()" << std::endl; break;

            case GL_INVALID_ENUM: std::cout << "Invalid enum" << std::endl; break;

            case GL_INVALID_VALUE: std::cout << "Invalid value" << std::endl; break;

            case GL_INVALID_OPERATION: std::cout << "Invalid operation" << std::endl; break;

            case GL_STACK_OVERFLOW: std::cout << "Stack overflow" << std::endl; break;

            case GL_STACK_UNDERFLOW: std::cout << "Stack underflow" << std::endl; break;

            case GL_OUT_OF_MEMORY: std::cout << "Out of memory" << std::endl; break;

            case GL_TABLE_TOO_LARGE: std::cout << "Table too large" << std::endl; break;

            default: std::cout << "Unknown error " << error << std::endl;
        }
    }
}


static bool setUpGLEW() {
    // Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum err       = glewInit();

    // If GLEW hasn't initialized
    if (err != GLEW_OK) {
        std::cout << "GLEW Error: " << glewGetErrorString(err) << std::endl;
        return false;
    }

    return true;
}

static bool setUpOpenGL() {
    bool success = setUpGLEW();
    if (!success) return success;

    // Set a background color
    glClearColor(0.0f, 0.0f, 1.0f, 0.0f);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    return success;
}

static void drawBasis(float scale) {
    GLUquadricObj* quadratic;
    quadratic                = gluNewQuadric();
    float coneRadius         = 0.2 * scale;
    float coneHeight         = 0.6 * scale;
    int numberOfConeSegments = 10;

    glDisable(GL_TEXTURE_2D);

    glEnable(GL_LIGHTING);
    // glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0f);
    // glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0f);
    // glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0f);
    GLfloat diff[4] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diff);
    // glLightfv(GL_LIGHT0, GL_AMBIENT, diff);
    // glLightfv(GL_LIGHT0, GL_SPECULAR, diff);
    // mat x
    GLfloat x_diff[4] = {1.0, 0.0, 0.0, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, x_diff);
    // cylinder x
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments, 1);
    glRotatef(-90, 0, 1, 0);
    // cone x
    glTranslatef(scale, 0, 0);
    glRotatef(90, 0, 1, 0);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glRotatef(-90, 0, 1, 0);
    glTranslatef(-scale, 0, 0);

    // Material y
    GLfloat y_diff[4] = {0.0, 1.0, 0.0, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, y_diff);

    // cylinder y
    glRotatef(-90, 1, 0, 0);
    gluCylinder(quadratic, coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments, 1);
    glRotatef(90, 1, 0, 0);

    // cone y
    glTranslatef(0, scale, 0);
    glRotatef(-90, 1, 0, 0);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glRotatef(90, 1, 0, 0);
    glTranslatef(0, -scale, 0);

    // mat z
    GLfloat z_diff[4] = {0.0, 0.0, 1.0, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, z_diff);
    // cylinder z
    gluCylinder(quadratic, coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments, 1);

    // cone z
    glTranslatef(0, 0, scale);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glDisable(GL_LIGHTING);
    delete quadratic;
}

static void drawCrossHair() {
    float w = 0.03;     // crosshair width
    float g = w / 4.0;  // crosshair gap


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(2.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);

    // Diag
    glVertex2f(w, w);
    glVertex2f(-w, -w);

    glVertex2f(-w, w);
    glVertex2f(w, -w);

    // Top right
    glVertex2f(w, w);
    glVertex2f(w, g);

    glVertex2f(g, w);
    glVertex2f(w, w);

    // bottom right
    glVertex2f(w, -w);
    glVertex2f(w, -g);

    glVertex2f(g, -w);
    glVertex2f(w, -w);

    // Bottom left
    glVertex2f(-w, -w);
    glVertex2f(-w, -g);

    glVertex2f(-g, -w);
    glVertex2f(-w, -w);

    // top left
    glVertex2f(-w, w);
    glVertex2f(-w, g);

    glVertex2f(-g, w);
    glVertex2f(-w, w);


    glEnd();
}

bool drawCamera(CvCapture* video, float verticalFOV) {
    IplImage* image = cvQueryFrame(video);
    if (image == NULL) {
        std::cout << "no images left in video file" << std::endl;
        return false;
    }

    GLenum format;
    switch (image->nChannels) {
        case 1: format = GL_LUMINANCE; break;
        case 2: format = GL_LUMINANCE_ALPHA; break;
        case 3: format = GL_BGR; break;
        default: break;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->width, image->height, 0, format, GL_UNSIGNED_BYTE, image->imageData);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* Draw the camera image, filling the screen */
    glColor3f(1., 1., 1.);
    glBegin(GL_QUADS);
    // BL
    glTexCoord2f(1., 1.);
    glVertex2f(-1., -1.);
    // BR
    glTexCoord2f(0., 1.);
    glVertex2f(1., -1.);
    // TR
    glTexCoord2f(0., 0.);
    glVertex2f(1., 1.);
    // TL
    glTexCoord2f(1., 0.);
    glVertex2f(-1., 1.);
    glEnd();

    // setup to draw on top of image
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glm::mat4 proj =

        // glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT,
        // image->width, image->height, 0.01f, 10.0f);

        glm::perspective(float(verticalFOV * 3.14159 / 180.0),        // VERTICAL FOV
                         float(image->width) / float(image->height),  // aspect ratio
                         0.01f,                                       // near plane distance (min z)
                         10.0f                                        // Far plane distance (max z)
        );
    glLoadMatrixf(glm::value_ptr(proj));

    checkGLError();

    return true;
}

struct ResultsFrame {
    int groundMatched = 0;
    int matched       = 0;
    float distance;
};

void drawSensorStreams(autocal::SensorPlant& sensorPlant,
                       std::string referenceFrame,
                       std::string matchStreamRange,
                       autocal::TimeStamp t,
                       const std::vector<std::pair<int, int>>& matches = std::vector<std::pair<int, int>>()) {
    float basisScale                   = 0.05;
    autocal::MocapRecording& recording = sensorPlant.mocapRecording;

    // record timeseries results for this frame:
    std::map<int, ResultsFrame> results;
    auto refFrame   = sensorPlant.getGroundTruth(referenceFrame, referenceFrame, t);
    auto rangeFrame = sensorPlant.getGroundTruth(matchStreamRange, referenceFrame, t);
    std::cout << t << " ";
    for (auto& pair : refFrame.rigidBodies) {
        Transform3D refPose = pair.second.pose;
        for (auto& rb : rangeFrame.rigidBodies) {
            auto& res         = results[rb.first];
            auto& rigidBodyID = rb.first;
            Transform3D pose  = rb.second.pose;

            res.distance      = arma::norm(pose.translation() - refPose.translation());
            res.groundMatched = int(res.distance < 0.2);
            bool drawMatch    = false;
            for (auto& m : matches) {
                drawMatch = drawMatch || (m.second == rigidBodyID);
            }
            res.matched = int(drawMatch);

            std::cout << res.distance << " " << res.groundMatched << " " << res.matched << " ";
        }
    }
    std::cout << std::endl;

    for (auto& stream : recording.streams) {
        std::string name                  = stream.first;
        autocal::MocapStream::Frame frame = sensorPlant.getGroundTruth(name, referenceFrame, t);
        bool drawMatches                  = (name == matchStreamRange);
        for (auto& pair : frame.rigidBodies) {
            // Get Rigid Body data
            auto& rigidBodyID = pair.first;
            auto& rigidBody   = pair.second;
            // 4x4 matrix pose
            Transform3D pose = rigidBody.pose;
            // std::cout << "pose " << rigidBodyID << " = \n" << pose << std::endl;
            // Load pose into opengl as view matrix
            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixd(pose.memptr());
            // Draw a sphere if it matches
            // TODO:recode this garbage!
            bool drawMatch = false;
            for (auto& m : matches) {
                drawMatch = drawMatch || (m.second == rigidBodyID);
            }
            if (drawMatches) {
                if (drawMatch) {
                    glEnable(GL_LIGHTING);
                    GLfloat diff[4] = {1.0, 1.0, 1.0, 1.0};
                    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
                    glutSolidSphere(0.5 * basisScale, 10, 10);
                }
            }
            drawBasis(basisScale);
        }
    }
}

#endif
