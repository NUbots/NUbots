precision lowp float;
precision lowp int;

uniform vec2 dimensions;

varying float vClass0;
varying float vClass1;
varying float vClass2;
varying float vClass3;
varying float vClass4;

void main() {
  gl_FragColor = vec4(vClass0, vClass3, vClass2, 0.5);
}
