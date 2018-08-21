precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 dimensions;

attribute vec2 position;

void main() {
//   vClassification = classification;

  // Calculate our position in the mesh
  vec2 pos = 2.0 * ((position / dimensions) - 0.5);
  pos.y *= -1.0;

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
