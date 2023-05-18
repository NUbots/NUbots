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

attribute vec3 position;

attribute float ball;
attribute float goal;
attribute float field;
attribute float fieldLine;
attribute float environment;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
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
    vField       = field;
    vEnvironment = environment;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
