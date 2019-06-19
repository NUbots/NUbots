precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 dimensions;

attribute vec2 position;

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

void main() {
//   vClassification = classification;

  // Calculate our position in the mesh
  vec2 pos = 2.0 * ((position / dimensions) - 0.5);
  pos.y *= -1.0;

  vBall = ball;
  vGoal = goal;
  vFieldLine = fieldLine;
  vField = field;
  vEnvironment = environment;

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
