precision highp float;
precision lowp int;

uniform float lineWidth;

varying vec2 vUv;
varying vec4 vColor;

void main() {
    gl_FragColor = vColor;
}
