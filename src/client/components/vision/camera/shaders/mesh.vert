precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 dimensions;

attribute vec2 position;
attribute float class0;
attribute float class1;
attribute float class2;
attribute float class3;
attribute float class4;

varying float vClass0;
varying float vClass1;
varying float vClass2;
varying float vClass3;
varying float vClass4;

void main() {
//   vClassification = classification;

  // Calculate our position in the mesh
  vec2 pos = 2.0 * ((position / dimensions) - 0.5);
  pos.y *= -1.0;

  vClass0 = class0;
  vClass1 = class1;
  vClass2 = class2;
  vClass3 = class3;
  vClass4 = class4;

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
