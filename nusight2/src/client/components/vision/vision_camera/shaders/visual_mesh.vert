precision highp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform mat4 Hcw;
uniform float focalLength;
uniform vec2 centre;
uniform vec2 k;
uniform int projection;
uniform float imageAspectRatio;

attribute vec3 position;

attribute float ball;
attribute float goal;
attribute float field;
attribute float fieldLine;
attribute float robot;
attribute float environment;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
varying float vRobot;
varying float vEnvironment;

#include "../../../camera/objects/shaders/projection.glsl"

void main() {
    // Rotate vector into camera space and project into image space
    // Correct for OpenGL coordinate system and aspect ratio
    // Multiply by 2.0 to get a coordinate since the width of the "image (the camera planes)" is -1 to 1 (width of 2.0)
    vec2 pos = project((Hcw * vec4(position, 0)).xyz, focalLength, centre, k, projection)
               * vec2(-1.0, viewSize.x / viewSize.y) * 2.0;

    vBall        = ball;
    vGoal        = goal;
    vFieldLine   = fieldLine;
    vRobot       = robot;
    vField       = field;
    vEnvironment = environment;

    // Aspect ratio of GL view
    float viewRatio = viewSize.x / viewSize.y;

    // In the case where the GL view's aspect ratio is wider than the image's,
    // the point should be scaled to align with the image.
    float scale = (viewRatio > imageAspectRatio) ? (imageAspectRatio / viewRatio) : 1.0;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos * scale, 0.0, 1.0);
}
