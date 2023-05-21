precision highp float;
precision lowp int;

uniform vec2 dimensions;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
varying float vEnvironment;

void main() {

    vec3 colour = vec3(1.0, 0.0, 0.0) * vBall + vec3(1.0, 1.0, 0.0) * vGoal + vec3(0.0, 0.0, 1.0) * vFieldLine
                  + vec3(0.0, 1.0, 0.0) * vField + vec3(0.0, 0.0, 0.0) * vEnvironment;

    gl_FragColor = vec4(colour, 0.5 * (1.0 - vEnvironment));
}
