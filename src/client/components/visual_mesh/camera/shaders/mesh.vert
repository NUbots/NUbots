precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

attribute vec2 position;
attribute vec3 classification;
attribute vec2 uv;

varying vec2 vUv;
varying vec3 vClassification;

#define M_PI 3.1415926535897932384626433832795

void main() {
  // Forward our varyings
  vUv = uv;
  vClassification = classification;

  // Calculate our position in the mesh
  float theta = M_PI * 2.0 * position.y;
  vec2 pos = vec2(cos(theta) * position.x, sin(theta) * position.x);

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}

