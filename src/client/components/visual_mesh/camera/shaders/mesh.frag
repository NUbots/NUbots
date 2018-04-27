precision lowp float;
precision lowp int;

uniform sampler2D image;
uniform vec2 dimensions;

varying vec3 vClassification;
varying vec2 vUv;

void main() {
  vec4 color = texture2D(image, vUv / dimensions);
  color += vec4(vClassification.x, 0, 0, 0);

  gl_FragColor = vec4(color.xyz, 1);
}
