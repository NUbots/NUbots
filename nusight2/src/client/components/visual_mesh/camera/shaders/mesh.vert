precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

attribute vec2 position;
attribute float ball;
attribute float goal;
attribute float field;
attribute float fieldLine;
attribute float robot;
attribute float environment;
attribute vec2 uv;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
varying float vEnvironment;
varying vec2 vUv;

#define M_PI 3.1415926535897932384626433832795

void main() {
    // Forward our varyings
    vUv = uv;

    // Classifications
    vBall        = ball;
    vGoal        = goal;
    vFieldLine   = fieldLine;
    vField       = field;
    vRobot       = robot;
    vEnvironment = environment;

    // Calculate our position in the mesh
    float theta = M_PI * 2.0 * position.y;
    vec2 pos    = vec2(cos(theta) * position.x, sin(theta) * position.x);

    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
