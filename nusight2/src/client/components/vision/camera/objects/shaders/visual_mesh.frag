precision highp float;
precision lowp int;

uniform vec2 dimensions;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
varying float vRobot;
varying float vEnvironment;

void main() {

    vec3 colour = vBall * vec3(1.0, 0.0, 0.0)            // Red
                  + vGoal * vec3(1.0, 1.0, 0.0)          // Yellow
                  + vFieldLine * vec3(0.2, 0.2, 1.0)     // Light Blue
                  + vField * vec3(0.0, 1.0, 0.0)         // Green
                  + vRobot * vec3(1.0, 0.0, 1.0)         // Magenta
                  + vEnvironment * vec3(0.0, 0.0, 0.0);  // Black

    gl_FragColor = vec4(colour, 0.5 * (1.0 - vEnvironment));
}
